//
// Created by Adam Gilbert on 10/11/25.
//
#include "esp_nimble_hci.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include <stdint.h>
#include "alarm.h"

/* ---------- App config ---------- */

#define DEVICE_NAME          "DOORBELL"
#define MAGIC_STRING         "OPEN_SESAME"


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
static void ble_app_advertise(void);
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
                    .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
                },
                { 0 } /* no more characteristics */
        }
    },
    { 0 } /* no more services */
};


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

/* GAP event handler: connection/disconnection, advertising restarts, MTU updates, etc. */
static int ble_gap_event_cb(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {
        case BLE_GAP_EVENT_CONNECT: {
            if (event->connect.status == 0) {
                ESP_LOGI(TAG, "Connected; conn_handle=%d", event->connect.conn_handle);
            } else {
                ESP_LOGI(TAG, "Connect failed; status=%d. Restarting adv.", event->connect.status);
                ble_app_advertise();
            }
            return 0;
        }

        case BLE_GAP_EVENT_DISCONNECT: {
            ESP_LOGI(TAG, "Disconnected; reason=%d", event->disconnect.reason);
            /* Resume advertising so the Android app can reconnect */
            ble_app_advertise();
            return 0;
        }

        case BLE_GAP_EVENT_ADV_COMPLETE: {
            ESP_LOGI(TAG, "Advertising complete; reason=%d. Restarting adv.", event->adv_complete.reason);
            ble_app_advertise();
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

/* Configure and start advertising */
static void ble_app_advertise(void)
{
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

    /* 0 = forever until explicitly stopped or connected */
    rc = ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER,
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

    ble_app_advertise();
}

/* NimBLE host task (runs the host thread) */
void host_task(void *param)
{
    ESP_LOGI(TAG, "NimBLE host task started");
    nimble_port_run();           // This function returns only when nimble_port_stop() is called.
    nimble_port_freertos_deinit();
}