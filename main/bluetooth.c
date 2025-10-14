//
// Created by Adam Gilbert on 10/11/25.
//
#include "esp_nimble_hci.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include <stdint.h>
#include "alarm.h"

/* ---------- App config ---------- */

#define DEVICE_NAME          "DOORBELL"
#define MAGIC_STRING         "OPEN_SESAME"

void ble_store_config_init(void);

#define BOOT_BTN_GPIO    GPIO_NUM_2
#define PAIR_WINDOW_MS   30000

static bool s_pair_window_open = false;
static esp_timer_handle_t s_pair_timer;
static QueueHandle_t s_btnq;

static nvs_handle_t storage_handle;

// Keep a tiny RAM cache of whitelisted peers
#define MAX_WL 10
static ble_addr_t s_wl[MAX_WL];
static size_t s_wl_count = 0;


static const char *TAG = "BLE_STRING_LISTENER";

/* Custom 128-bit UUIDs (little-endian in code, but define as macros for clarity) */
static const ble_uuid128_t SERVICE_UUID = BLE_UUID128_INIT(
    0x6a,0x2e,0x4d,0x4b,0x84,0x04,0x46,0x0e,0xa8,0x33,0x7e,0x1e,0x00,0x01,0xab,0xcd
);
static const ble_uuid128_t CHAR_RX_UUID = BLE_UUID128_INIT(
    0x6a,0x2e,0x4d,0x4b,0x84,0x04,0x46,0x0e,0xa8,0x33,0x7e,0x1e,0x00,0x02,0xab,0xcd
);

/* Forward decls */
static int  gatt_chr_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                               struct ble_gatt_access_ctxt *ctxt, void *arg);
static void ble_app_advertise();
static int  ble_gap_event_cb(struct ble_gap_event *event, void *arg);


/* ---------------- GATT DB ----------------
 * One primary service with one writable characteristic.
 */
const struct ble_gatt_svc_def gatt_svcs[] = {
    {
        /*** Service: Custom RX ***/
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &SERVICE_UUID.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
                {
                    .uuid = &CHAR_RX_UUID.u,
                    .access_cb = gatt_chr_access_cb,
                    .val_handle = NULL,
                    .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP | BLE_GATT_CHR_F_WRITE_ENC,
                },
                { 0 } /* no more characteristics */
        }
    },
    { 0 } /* no more services */
};

static void wl_clear(void) { s_wl_count = 0; }

static void wl_add(const ble_addr_t *addr) {
    if (!addr || s_wl_count >= MAX_WL) {
        ESP_LOGW(TAG, "Not enough room to add new BLE address");
        return;
    }
    s_wl[s_wl_count++] = *addr;
}

static bool wl_contains(const ble_addr_t *addr) {
    for (int i = 0; i < s_wl_count; i++) {
        bool is_equal = true;
        for (int j = 0; j < 5; j++) {
            if (s_wl[i].val[j] != addr->val[j]) {
                is_equal = false;
                break;
            }
        }
        if (is_equal) {
            return true;
        }
    }
    return false;
}

static void wl_apply(void) {
    // Program the controller whitelist with our current list
    ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_set_blob(storage_handle, "ble_addresses", &s_wl[0], sizeof(ble_addr_t)*s_wl_count));
    ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_commit(storage_handle));
}

static void wl_reload_from_store(void) {
    wl_clear();
    // Iterate stored bonds and add their identity addrs to RAM cache
    // int num_peers;
    // if (ble_store_util_bonded_peers(&s_wl[0], &num_peers, MAX_WL) == 0) {
    //     s_wl_count = num_peers;
    //     wl_apply();
    // }

    // Read from nvs
    size_t required_size;
    ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_get_blob(storage_handle, "ble_addresses", NULL, &required_size));
    size_t num_entries = required_size/sizeof(ble_addr_t);

    if (num_entries > MAX_WL) {
        ESP_LOGE(TAG, "Too many BLE addresses!");
        return;
    }

    ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_get_blob(storage_handle, "ble_addresses", &s_wl[0], &required_size));
    s_wl_count = num_entries;
    ESP_LOGI(TAG, "Restored %d known ble addresses", s_wl_count);


}


/* Utility: copy incoming write data out of os_mbuf into a C buffer */
static int om_to_cbuf(struct os_mbuf *om, uint8_t *buf, uint16_t buf_len, uint16_t *out_len)
{
    uint16_t om_len = OS_MBUF_PKTLEN(om);
    if (om_len > buf_len) {
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }
    int rc = ble_hs_mbuf_to_flat(om, buf, buf_len, out_len);
    return rc == 0 ? 0 : BLE_ATT_ERR_UNLIKELY;
}

/* Characteristic access callback: handles writes to our RX characteristic. */
static int gatt_chr_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                              struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    switch (ctxt->op) {
        case BLE_GATT_ACCESS_OP_WRITE_CHR: {
            uint8_t tmp[128];
            uint16_t len = 0;
            int rc = om_to_cbuf(ctxt->om, tmp, sizeof(tmp), &len);
            if (rc != 0) {
                ESP_LOGW(TAG, "Write too long or conversion error (rc=%d)", rc);
                return rc;
            }
            /* Null-terminate so we can treat as string safely. */
            if (len >= sizeof(tmp)) len = sizeof(tmp) - 1;
            tmp[len] = '\0';

            ESP_LOGI(TAG, "Received write (%u bytes): '%s'", (unsigned)len, (char*)tmp);

            if (strcmp((const char*)tmp, MAGIC_STRING) == 0) {
                ESP_LOGI(TAG, "Doorbell requested...");
                const player_cmd_t cmd = CMD_PLAY_DOORBELL;
                xQueueSend(s_cmd_q, &cmd, pdMS_TO_TICKS(500));
            } else {
                ESP_LOGI(TAG, "Not a match.");
            }
            return 0; /* success */
        }
        default:
            return BLE_ATT_ERR_REQ_NOT_SUPPORTED;
    }
}

static void close_pair_window(void* arg)
{
    s_pair_window_open = false;
    ESP_LOGI(TAG, "Pairing window CLOSED");
    // Return to whitelist-only advertising
}

static void open_pair_window(void)
{
    s_pair_window_open = true;
    ESP_LOGI(TAG, "Pairing window OPEN for %d ms", PAIR_WINDOW_MS);

    // (Re)arm the 30s one-shot
    if (s_pair_timer) esp_timer_stop(s_pair_timer);
    esp_timer_start_once(s_pair_timer, PAIR_WINDOW_MS * 1000);
}

/* GAP event handler: connection/disconnection, advertising restarts, MTU updates, etc. */
static int ble_gap_event_cb(struct ble_gap_event *event, void *arg)
{
    ESP_LOGD(TAG, "Got event: %d", event->type);
    switch (event->type) {
        case BLE_GAP_EVENT_CONNECT: {
            struct ble_gap_conn_desc d;
            if (ble_gap_conn_find(event->connect.conn_handle, &d) == 0) {
                if (!s_pair_window_open) {
                    if (!wl_contains(&d.peer_id_addr)) {
                        ESP_LOGI(TAG, "Disconnecting from peer id, not in whitelist %d:%d:%d:%d:%d:%d", &d.peer_id_addr.val[0], &d.peer_id_addr.val[1], &d.peer_id_addr.val[2], &d.peer_id_addr.val[3], &d.peer_id_addr.val[4], &d.peer_id_addr.val[5]);
                        ble_gap_terminate(event->connect.conn_handle, 0x05);
                        return 0;
                    }
                }
                if (event->connect.status == 0 && !wl_contains(&d.peer_id_addr)) {
                    wl_add(&d.peer_id_addr);
                    wl_apply();
                    ESP_LOGI(TAG, "Connected -> added to whitelist %d:%d:%d:%d:%d:%d", &d.peer_id_addr.val[0], &d.peer_id_addr.val[1], &d.peer_id_addr.val[2], &d.peer_id_addr.val[3], &d.peer_id_addr.val[4], &d.peer_id_addr.val[5]);
                }
                if (event->connect.status == 0) {
                    if (ble_gap_conn_find(event->connect.conn_handle, &d) == 0 && !d.sec_state.encrypted) {
                        ble_gap_security_initiate(event->connect.conn_handle);
                    }
                }
                if (event->connect.status == 0) {
                    ESP_LOGI(TAG, "Connected; conn_handle=%d", event->connect.conn_handle);
                } else {
                    ESP_LOGI(TAG, "Connect failed; status=%d. Restarting adv.", event->connect.status);
                    ble_app_advertise();
                }
            }
            return 0;
        }

        case BLE_GAP_EVENT_DISCONNECT: {
            ESP_LOGI(TAG, "Disconnected; reason=%d", event->disconnect.reason);
            /* Resume advertising so the Android app can reconnect */
            if (!ble_gap_adv_active())
                ble_app_advertise();
            return 0;
        }

        case BLE_GAP_EVENT_ADV_COMPLETE: {
            ESP_LOGI(TAG, "Advertising complete; reason=%d. Restarting adv.", event->adv_complete.reason);
            ble_app_advertise();
            return 0;
        }

        case BLE_GAP_EVENT_REPEAT_PAIRING: {
            struct ble_gap_conn_desc d;
            ble_addr_t addr;
            if (ble_gap_conn_find(event->repeat_pairing.conn_handle, &d) == 0) {
                addr = d.peer_id_addr;
                ESP_LOGI(TAG, "Repair: keys deleted");
                ble_store_util_delete_peer(&addr);
                return 1; // allow new keys to be generated
            }
            return 0;
        }


        case BLE_GAP_EVENT_ENC_CHANGE: {
            // Called after encryption/pairing state changes. On success while window open,
            // add the peer to our whitelist so it can connect later when the window closes.
            if (event->enc_change.status == 0 && s_pair_window_open) {
                struct ble_gap_conn_desc d;
                if (ble_gap_conn_find(event->enc_change.conn_handle, &d) == 0) {
                    wl_add(&d.peer_id_addr);
                    wl_apply();
                    ESP_LOGI(TAG, "Bonded -> added to whitelist %d:%d:%d:%d:%d:%d", &d.peer_id_addr.val[0], &d.peer_id_addr.val[1], &d.peer_id_addr.val[2], &d.peer_id_addr.val[3], &d.peer_id_addr.val[4], &d.peer_id_addr.val[5]);
                }
            }
            return 0;
        }

        case BLE_GAP_EVENT_MTU: {
            ESP_LOGI(TAG, "MTU updated; channel=%d mtu=%d",
                     event->mtu.channel_id, event->mtu.value);
            return 0;
        }

        default:
            return 0;
    }
}

void ble_security_init(void) {
    // Headless device: no input / no output
    ble_hs_cfg.sm_io_cap  = BLE_HS_IO_NO_INPUT_OUTPUT;

    // Light security: bond & encrypt; don't require MITM
    ble_hs_cfg.sm_bonding = 1;   // remember keys in NVS
    ble_hs_cfg.sm_mitm    = 0;   // no MITM (inevitable with no IO anyway)
    ble_hs_cfg.sm_sc      = 1;   // prefer LE Secure Connections when phone supports it

    // Exchange enc + identity keys
    ble_hs_cfg.sm_our_key_dist   = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
    ble_hs_cfg.sm_their_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;

    ble_store_config_init();
}

/* Configure and start advertising */
static void ble_app_advertise()
{
    static uint8_t s_own_addr_type;

    // prefer RPA so the controller enables privacy/resolution
    ESP_ERROR_CHECK(ble_hs_id_infer_auto(/*prefer_rpa=*/1, &s_own_addr_type));
    struct ble_gap_adv_params advp;
    struct ble_hs_adv_fields fields = {0};

    /* Advertise flags (general discoverable, BR/EDR unsupported) */
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;

    /* Include service UUID so scanners can filter */
    fields.uuids128 = &SERVICE_UUID;
    fields.num_uuids128 = 1;
    fields.uuids128_is_complete = 1;

    /* Advertise complete device name */
    fields.name = (uint8_t*)DEVICE_NAME;
    fields.name_len = 8;          // shorter than full name
    fields.name_is_complete = 0;  // “shortened name” AD type

    int rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gap_adv_set_fields: rc=%d", rc);
        return;
    }

    /* Set advertising params (undirected, general discoverable) */
    memset(&advp, 0, sizeof(advp));
    advp.conn_mode = BLE_GAP_CONN_MODE_UND;  /* connectable undirected */
    advp.disc_mode = BLE_GAP_DISC_MODE_GEN;  /* general discoverable */
    advp.filter_policy = BLE_HCI_ADV_FILT_NONE;

    /* 0 = forever until explicitly stopped or connected */
    rc = ble_gap_adv_start(s_own_addr_type, NULL, BLE_HS_FOREVER,
                           &advp, ble_gap_event_cb, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gap_adv_start: rc=%d", rc);
    } else {
        ESP_LOGI(TAG, "Advertising as '%s'", DEVICE_NAME);
    }
}

/* Host reset callback (optional logging) */
static void on_sync(void)
{
    /* Ensure we have a proper address; prefer public if available */
    uint8_t addr_val[6] = {0};
    int rc = ble_hs_id_infer_auto(0, &addr_val[0]);
    assert(rc == 0);

    uint8_t own_addr_type;
    rc = ble_hs_id_infer_auto(0, &own_addr_type);
    assert(rc == 0);

    ble_addr_t addr = { .type = own_addr_type };
    ble_hs_id_copy_addr(own_addr_type, addr.val, NULL);
    ESP_LOGI(TAG, "Device Address: %02X:%02X:%02X:%02X:%02X:%02X",
             addr.val[5], addr.val[4], addr.val[3], addr.val[2], addr.val[1], addr.val[0]);


    ESP_ERROR_CHECK(nvs_open("storage", NVS_READWRITE, &storage_handle));
    wl_reload_from_store();
    ble_app_advertise();

}

/* NimBLE host task (runs the host thread) */
void host_task(void *param)
{
    ESP_LOGI(TAG, "NimBLE host task started");
    nimble_port_run();           // This function returns only when nimble_port_stop() is called.
    nimble_port_freertos_deinit();
}