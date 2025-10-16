/*
 * alarm.c - Play randomly selected WAV files when a GPIO pin is triggered
 * Written by AEG, RC-M
 * 2025
 */

#include "alarm.h"
#include "filesystem.c"
#include "bluetooth.c"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

typedef struct {
    decoded_wav_t* wav;
    bool destroy_on_finish;
} playback_cmd_t;

static decoded_wav_t s_decoded_wavs[2];


// ---------------- I2S helpers ----------------
static esp_err_t i2s_init(int sample_rate_hz, int channels)
{
    const static char* TAG = "i2s_init";
    i2s_config_t i2s_config = {
        .mode = I2S_MODE_MASTER | I2S_MODE_TX,
        .sample_rate = sample_rate_hz,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = (channels == 1) ? I2S_CHANNEL_FMT_ONLY_LEFT
                                          : I2S_CHANNEL_FMT_RIGHT_LEFT,
        .communication_format = I2S_COMM_FORMAT_I2S,
        .intr_alloc_flags = ESP_INTR_FLAG_IRAM,
        .dma_buf_count = 12,
        .dma_buf_len = 256,
        .use_apll = true,
        .tx_desc_auto_clear = true,
        .fixed_mclk = 0
    };

    i2s_pin_config_t pin_config = {
        .bck_io_num   = I2S_BCLK,
        .ws_io_num    = I2S_LRC,
        .data_out_num = I2S_DOUT,
        .data_in_num  = I2S_PIN_NO_CHANGE
    };

    ESP_RETURN_ON_ERROR(i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL),
                        TAG, "i2s_driver_install failed");
    ESP_RETURN_ON_ERROR(i2s_set_pin(I2S_NUM_0, &pin_config),
                        TAG, "i2s_set_pin failed");
    ESP_RETURN_ON_ERROR(i2s_set_clk(I2S_NUM_0, sample_rate_hz,
                                    I2S_BITS_PER_SAMPLE_16BIT,
                                    (channels == 1) ? I2S_CHANNEL_MONO
                                                    : I2S_CHANNEL_STEREO),
                        TAG, "i2s_set_clk failed");
    return ESP_OK;
}

static inline void i2s_update_rate_channels(int sample_rate_hz, int channels)
{
    i2s_set_clk(I2S_NUM_0, sample_rate_hz, I2S_BITS_PER_SAMPLE_16BIT,
                (channels == 1) ? I2S_CHANNEL_MONO : I2S_CHANNEL_STEREO);
}

// ---------------- SD card helpers ----------------
static esp_err_t sdcard_mount(void)
{
    const static char* TAG = "sdcard_mount";
    ESP_LOGI("CARD", "Mounting SD card (SDSPI @ SPI2_HOST)...");
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = SPI_MOSI,
        .miso_io_num = SPI_MISO,
        .sclk_io_num = SPI_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 0,
    };

    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus_cfg, SDSPI_DEFAULT_DMA));

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = SPI2_HOST;
    host.max_freq_khz = 2000;

    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = SD_CS;
    slot_config.host_id = host.slot;

    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };

    sdmmc_card_t *card = NULL;
    esp_err_t ret = esp_vfs_fat_sdspi_mount(MOUNT_POINT, &host, &slot_config,
                                            &mount_config, &card);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Mount failed: %s", esp_err_to_name(ret));
        spi_bus_free(SPI2_HOST);
        return ret;
    }

    sdmmc_card_print_info(stdout, card);
    return ESP_OK;
}

// static void sdcard_unmount(void)
// {
//     esp_vfs_fat_sdcard_unmount(MOUNT_POINT, NULL);
//     spi_bus_free(SPI2_HOST);
// }


int randrange(int min, int max){
    return min + esp_random() / (RAND_MAX / (max - min + 1) + 1);
}

char* name_from_rarity(file_rarity_t rarity) {
    switch (rarity) {
        case ANY: return "ANY"; break;
        case COMMON: return "COMMON"; break;
        case UNCOMMON: return "UNCOMMON"; break;
        case RARE: return "RARE"; break;
        case LEGENDARY: return "LEGENDARY"; break;
    }
    return NULL;
}



// ---------------- WAV playback (task context) ----------------
static void consumer_i2s_task(void* arg) {
    const static char* TAG = "consumer_i2s_task";
    playback_cmd_t* cmd = (playback_cmd_t*)arg;
    decoded_wav_t* wav_s = cmd->wav;
    uint8_t* chunk = heap_caps_malloc(I2S_WRITE_CHUNK, MALLOC_CAP_DMA);
    i2s_update_rate_channels(wav_s->s_buffer_sample_rate, 1);
    for (;;) {
        // Wait until at least one byte is available, then read up to chunk size
        size_t got = xStreamBufferReceive(wav_s->s_reader_buffer, chunk, I2S_WRITE_CHUNK, pdMS_TO_TICKS(1000));
        if (got < I2S_WRITE_CHUNK) {
            ESP_LOGW(TAG, "Received %d bytes, less than chunk size!!", got);
        }
        if (got == 0) {
            // If producer is done AND buffer is empty, we’re finished
            size_t bytes_avail = xStreamBufferBytesAvailable(wav_s->s_reader_buffer);
            ESP_LOGI(TAG, "%d bytes remaining", bytes_avail);
            if (bytes_avail == 0) break;
            continue;
        }
        size_t written = 0;
        while (written < got) {
            size_t just;
            esp_err_t a = (i2s_write(I2S_NUM_0, chunk + written, got - written, &just, portMAX_DELAY));
            if (a == ESP_OK) {
                written += just;
            }
        }
    }
    heap_caps_free(chunk);
    xEventGroupSetBits(s_done_group, BIT_CONSUMER_DONE);
    vTaskDelete(NULL);
}

static void producer_task(void* arg) {
    const static char* TAG = "producer_task";
    const playback_cmd_t* cmd = (playback_cmd_t*)arg;
    decoded_wav_t* wav_s = cmd->wav;
    uint8_t* tmp = heap_caps_malloc(SD_READ_CHUNK, MALLOC_CAP_DMA);
    size_t got = 0;
    for (;;) {
        if (wav_s->s_paused) continue;
        got = fread(tmp, 1, SD_READ_CHUNK, wav_s->s_file_handle);
        if (got == 0) break; // EOF or error
        wav_s->s_reader_offset += got;
        size_t pushed = 0;
        while (pushed < got) {
            // Blocks until there is space
            pushed += xStreamBufferSend(wav_s->s_reader_buffer, tmp + pushed, got - pushed, portMAX_DELAY);
        }
    }
    // Signal EOF by closing the stream (writer will detect no more incoming after drain)
    heap_caps_free(tmp);
    if (cmd->destroy_on_finish) {
        fclose(wav_s->s_file_handle);
        wav_s->s_file_handle = NULL;
    }
    else {
        // Reset the file to the beginning of PCM data for the next play instead
        fseek(wav_s->s_file_handle, 44, SEEK_SET);
        wav_s->s_reader_offset = 0;
    }
    xEventGroupSetBits(s_done_group, BIT_PRODUCER_DONE);
    ESP_LOGI(TAG, "Producer done reading file");
    vTaskDelete(NULL);
}

// ---------------- GPIO ISR + configuration ----------------

static const TickType_t s_debounce_ticks = pdMS_TO_TICKS(DEBOUNCE_MS);
static volatile TickType_t s_last_tick = 0;

static void IRAM_ATTR trigger_isr(void *arg)
{
    TickType_t now = xTaskGetTickCountFromISR();
    if ((now - s_last_tick) < s_debounce_ticks) return; // debounce
    s_last_tick = now;

    if (!s_is_playing) {
        player_cmd_t cmd = CMD_PLAY;
        BaseType_t hp_task_woken = pdFALSE;
        xQueueSendFromISR(s_cmd_q, &cmd, &hp_task_woken);
        if (hp_task_woken == pdTRUE) {
            portYIELD_FROM_ISR();
        }
    }
}

static void trigger_gpio_init(void)
{
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << TRIGGER_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en   = (TRIGGER_LEVEL == 0) ? GPIO_PULLUP_ENABLE   : GPIO_PULLUP_DISABLE,
        .pull_down_en = (TRIGGER_LEVEL == 1) ? GPIO_PULLDOWN_ENABLE : GPIO_PULLDOWN_DISABLE,
        .intr_type    = (TRIGGER_LEVEL == 0) ? GPIO_INTR_POSEDGE    : GPIO_INTR_POSEDGE
    };
    ESP_ERROR_CHECK(gpio_config(&io));
    ESP_ERROR_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_IRAM));
    ESP_ERROR_CHECK(gpio_isr_handler_add(TRIGGER_GPIO, trigger_isr, NULL));
}

// ---------------- Player task ----------------
static void player_task(void *arg)
{
    char* filename = NULL;
    const static char* TAG = "player_task";
    if (sdcard_mount() != ESP_OK) {
        ESP_LOGE(TAG, "SD mount failed, player task exiting.");
        vTaskDelete(NULL);
        return;
    }

    // Prepare first file
    filename = get_random_filename(ANY);
    prepare_wav_file(&s_decoded_wavs[0], filename);
    i2s_init(s_decoded_wavs[0].s_buffer_sample_rate, 1);
    free(filename);
    filename = NULL;

    // Prepare BLE file
    prepare_wav_file(&s_decoded_wavs[1], AUDIO_FOLDER "doorbell.wav");
    if (!s_done_group) s_done_group = xEventGroupCreate();

    while (1) {
        player_cmd_t cmd;
        if (xQueueReceive(s_cmd_q, &cmd, portMAX_DELAY) == pdTRUE) {
            if (cmd == CMD_PLAY || cmd == CMD_PLAY_DOORBELL) {
                playback_cmd_t pcmd;
                ESP_LOGI(TAG, "PLAY command received");
                if (!s_is_playing) {
                    decoded_wav_t* wav = cmd == CMD_PLAY ? &s_decoded_wavs[0] : &s_decoded_wavs[1];
                    pcmd.wav = wav;
                    pcmd.destroy_on_finish = cmd == CMD_PLAY;
                    xTaskCreatePinnedToCore(producer_task, "sd_reader", 4096, &pcmd, 5, NULL, 1);
                    xTaskCreatePinnedToCore(consumer_i2s_task, "i2s_writer", 4096, &pcmd, 6, NULL, 1);

                    // Wait until BOTH bits are set
                    (void)xEventGroupWaitBits(
                        s_done_group,
                        BIT_PRODUCER_DONE | BIT_CONSUMER_DONE,
                        pdTRUE,   // clear on exit
                        pdTRUE,   // wait for ALL bits
                        portMAX_DELAY
                    );
                    // If we queued more than one play, reset now
                    xQueueReset(s_cmd_q);
                    if (cmd == CMD_PLAY) {
                        filename = get_random_filename(ANY);
                        prepare_wav_file(&s_decoded_wavs[0], filename);
                        free(filename);
                        filename = NULL;
                    }
                    else{ xStreamBufferReset(wav->s_reader_buffer); }
                }
            }
        }
    }


}

static void IRAM_ATTR isr_boot(void *arg) {
    uint32_t v = 1;
    BaseType_t hpw = pdFALSE;
    if (!s_pair_window_open)
        xQueueSendFromISR(s_btnq, &v, &hpw);
    portYIELD_FROM_ISR();
}

static void button_task(void *arg) {
    uint32_t v;
    for (;;) {
        if (xQueueReceive(s_btnq, &v, portMAX_DELAY)) {
            // simple debounce (optional)
            vTaskDelay(pdMS_TO_TICKS(50));
            if (gpio_get_level(BOOT_BTN_GPIO) == 0) {
                open_pair_window();
            }
        }
    }
}

static void button_init(void) {
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << BOOT_BTN_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,   // BOOT has on-board pull-up
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE      // press -> low
    };
    gpio_config(&io);
    gpio_isr_handler_add(BOOT_BTN_GPIO, isr_boot, NULL);

    s_btnq = xQueueCreate(4, sizeof(uint32_t));
    xTaskCreatePinnedToCore(button_task, "btn", 8192, NULL, 5, NULL, tskNO_AFFINITY);

    // Timer for closing the window
    const esp_timer_create_args_t targs = { .callback = &close_pair_window, .name = "pairwin" };
    esp_timer_create(&targs, &s_pair_timer);
}

// ---------------- app_main ----------------
void app_main(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    s_cmd_q = xQueueCreate(4, sizeof(player_cmd_t));
    configASSERT(s_cmd_q != NULL);
    trigger_gpio_init();

    /* Initialize the controller + HCI and the NimBLE host */
    nimble_port_init();
    ble_security_init();
    button_init();

    /* Register GAP/GATT services provided by NimBLE (for Device Name, etc.) */
    ble_svc_gap_init();
    ble_svc_gatt_init();

    /* Set the device name that appears in scan results */
    ble_svc_gap_device_name_set(DEVICE_NAME);

    /* Add our own GATT services */
    int rc = ble_gatts_count_cfg(gatt_svcs);
    assert(rc == 0);
    rc = ble_gatts_add_svcs(gatt_svcs);
    assert(rc == 0);

    /* Start the host; once sync completes, start advertising */
    ble_hs_cfg.reset_cb = NULL;          /* optional */
    ble_hs_cfg.sync_cb  = on_sync;       /* called when the stack is ready */
    ble_hs_cfg.gatts_register_cb = NULL; /* optional debug */

    /* Launch host thread */
    nimble_port_freertos_init(host_task);


    xTaskCreatePinnedToCore(player_task, "player_task",
                            20000,
                            NULL, 5, NULL, 0);

}
