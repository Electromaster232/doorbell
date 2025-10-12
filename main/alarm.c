/*
 * alarm.c - Play randomly selected WAV files when a GPIO pin is triggered
 * Written by AEG, RC-M
 * 2025
 */

#include <bootloader_random.h>
#include <esp_heap_trace.h>
#include <esp_random.h>
#include <stdio.h>
#include <string.h>
#include <sys/dirent.h>
#include <sys/stat.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "driver/gpio.h"
#include "driver/i2s.h"
#include "driver/spi_master.h"
#include "driver/sdspi_host.h"
#include "sdmmc_cmd.h"
#include "esp_vfs_fat.h"
#include "esp_check.h"
#include "freertos/stream_buffer.h"


#include "alarm.h"
#include "filesystem.c"



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
    return min + rand() / (RAND_MAX / (max - min + 1) + 1);
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
    decoded_wav_t* wav_s = (decoded_wav_t*)arg;
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
    vStreamBufferDelete(wav_s->s_reader_buffer);
    wav_s->s_reader_buffer = NULL;
    vTaskDelete(NULL);
}

static void producer_task(void* arg) {
    const static char* TAG = "producer_task";
    decoded_wav_t* wav_s = (decoded_wav_t*)arg;
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
    fclose(wav_s->s_file_handle);
    wav_s->s_file_handle = NULL;
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
    filename = get_random_filename(ANY);
    prepare_wav_file(&s_decoded_wavs[0], filename);
    i2s_init(s_decoded_wavs[0].s_buffer_sample_rate, 1);
    free(filename);
    filename = NULL;
    if (!s_done_group) s_done_group = xEventGroupCreate();

    while (1) {
        player_cmd_t cmd;
        if (xQueueReceive(s_cmd_q, &cmd, portMAX_DELAY) == pdTRUE) {
            if (cmd == CMD_PLAY) {
                ESP_LOGI(TAG, "PLAY command received");
                if (!s_is_playing) {
                    xTaskCreatePinnedToCore(producer_task, "sd_reader", 4096, &s_decoded_wavs[0], 5, NULL, 1);
                    xTaskCreatePinnedToCore(consumer_i2s_task, "i2s_writer", 4096, &s_decoded_wavs[0], 6, NULL, 1);

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
                    filename = get_random_filename(ANY);
                    prepare_wav_file(&s_decoded_wavs[0], filename);
                    free(filename);
                    filename = NULL;
                }
            }
        }
    }


}

// ---------------- app_main ----------------
void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());

    s_cmd_q = xQueueCreate(4, sizeof(player_cmd_t));
    configASSERT(s_cmd_q != NULL);
    bootloader_random_enable();
    srand(esp_random());
    bootloader_random_disable();
    trigger_gpio_init();


    xTaskCreatePinnedToCore(player_task, "player_task",
                            20000,
                            NULL, 5, NULL, 0);

}
