/*
 * alarm.c - Play randomly selected WAV files when a GPIO pin is triggered
 * Written by AEG, RC-M
 * 2025
 */

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


// --- Pins ---
// SD over SPI (SDSPI on SPI2/HSPI)
#define SD_CS          5
#define SPI_MOSI      23
#define SPI_MISO      19
#define SPI_SCK       18

// I2S
#define I2S_DOUT      27
#define I2S_BCLK      26
#define I2S_LRC       25

// Trigger input
#define TRIGGER_GPIO   GPIO_NUM_4
#define TRIGGER_LEVEL  0             // 0: falling edge, 1: rising edge
#define DEBOUNCE_MS    50

// Filesystem
#define MOUNT_POINT   "/sdcard"
#define AUDIO_FOLDER MOUNT_POINT "/audio/"

// Audio system
#define AUDIO_BUFFER_MAX_SIZE (44100 * 16 * 30)           // buffer size for reading the wav file and sending to i2s. 44.1K 16-bit samples per second, times 30 seconds
#define RB_CAPACITY     (96 * 1024)
#define SD_READ_CHUNK   (8 * 1024)      // bytes per SD read  (multiple of 512)
#define I2S_WRITE_CHUNK (4 * 1024)      // bytes per I2S write

// Command queue
#define BIT_PRODUCER_DONE  (1 << 0)
#define BIT_CONSUMER_DONE  (1 << 1)

typedef enum {COMMON, UNCOMMON, RARE, LEGENDARY, ANY} file_rarity_t;

#define GET_ANY_RANDOM_FILENAME get_random_filename(file_rarity_t::ANY)

static EventGroupHandle_t s_done_group;
typedef enum { CMD_PLAY, CMD_STOP } player_cmd_t;
static QueueHandle_t s_cmd_q = NULL;
static volatile bool s_is_playing = false;


typedef struct {
    size_t s_buffer_size;
    size_t s_number_of_samples;
    int s_buffer_sample_rate;
    FILE* s_file_handle;
    size_t s_reader_offset;
    StreamBufferHandle_t s_reader_buffer;
    bool s_paused;
} decoded_wav_t;

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

static esp_err_t prepare_wav_file(decoded_wav_t* wav_s, const char *filename) {
    const static char* TAG = "prepare_wav_file";
    ESP_LOGI(TAG, "Preparing wav file %s", filename);
    // Setup the stream buffer
    if (wav_s->s_reader_buffer) vStreamBufferDelete(wav_s->s_reader_buffer);
    wav_s->s_reader_buffer = xStreamBufferCreate(RB_CAPACITY, 0);
    if (wav_s->s_reader_buffer == NULL) {
        ESP_LOGE(TAG, "Failed to create reader buffer");
        return ESP_FAIL;
    }
    // Put the file names together
    // char* full_file_name = malloc(strlen(filename) + strlen(MOUNT_POINT) + 2);
    // strcpy(full_file_name, MOUNT_POINT);
    // strcat(full_file_name, "/");
    // strcat(full_file_name, filename);
    FILE *fh = fopen(filename, "rb");
    // free(full_file_name);
    if (fh == NULL)
    {
        ESP_LOGE(TAG, "Failed to open file");
        return ESP_ERR_INVALID_ARG;
    }
    wav_s->s_file_handle = fh;
    // Seek to sample rate
    fseek(fh, 24, SEEK_SET);
    // Read sample rate
    fread(&(wav_s->s_buffer_sample_rate), 4, 1, fh);
    ESP_LOGI(TAG, "sample rate: %d", wav_s->s_buffer_sample_rate);
    // Seek to Subchunk2Size
    fseek(fh, 40, SEEK_SET);
    // Read subchunk (audio data) size
    size_t data_byte_size;
    fread(&data_byte_size, 4, 1, fh);
    // This is NOT the number of samples
    // This is the number of bytes of the entire data area!!
    // == NumSamples * NumChannels * BitsPerSample/8
    // Since we know NumChannels and BitsPerSample, it is possible to calculate NumSamples

    wav_s->s_number_of_samples = data_byte_size / 1 / (16/8);
    wav_s->s_buffer_size = data_byte_size;
    ESP_LOGI(TAG, "buffer size: %d", wav_s->s_buffer_size);

    // Seek to data
    fseek(fh, 44, SEEK_SET);

    // Begin prefilling the buffer
    {
        uint8_t* pre = heap_caps_malloc(SD_READ_CHUNK, MALLOC_CAP_DMA);
        wav_s->s_reader_offset = 0;
        while (wav_s->s_reader_offset < (RB_CAPACITY)) {
            size_t n = fread(pre, 1, SD_READ_CHUNK, fh);
            if (n == 0) break;
            size_t sent = 0;
            while (sent < n) {
                sent += xStreamBufferSend(wav_s->s_reader_buffer, pre + sent, n - sent, portMAX_DELAY);
            }
            wav_s->s_reader_offset += n;
        }
        vPortFree(pre);
    }

    ESP_LOGI(TAG, "WAV file loaded. Leaving the following struct behind:");
    ESP_LOGI(TAG, "Sample rate: %d", wav_s->s_buffer_sample_rate);
    ESP_LOGI(TAG, "Buffer size: %d", wav_s->s_buffer_size);
    ESP_LOGI(TAG, "Number of samples: %d", wav_s->s_number_of_samples);
    return ESP_OK;
}

int randrange(int min, int max){
    return min + rand() / (RAND_MAX / (max - min + 1) + 1);
}

static char* name_from_rarity(file_rarity_t rarity) {
    switch (rarity) {
        case ANY: return "ANY"; break;
        case COMMON: return "COMMON"; break;
        case UNCOMMON: return "UNCOMMON"; break;
        case RARE: return "RARE"; break;
        case LEGENDARY: return "LEGENDARY"; break;
    }
    return NULL;
}

static int count_files_in_directory(const char *path) {
    DIR *dir_ptr;
    struct dirent *direntp;
    int file_count = 0;

    // Open the directory
    if ((dir_ptr = opendir(path)) == NULL) {
        perror("Failed to open directory");
        return -1; // Indicate an error
    }

    // Read directory entries
    while ((direntp = readdir(dir_ptr)) != NULL) {
        // Skip "." and ".." entries
        if (strcmp(direntp->d_name, ".") == 0 || strcmp(direntp->d_name, "..") == 0) {
            continue;
        }

        // Check if the entry is a regular file
        // d_type is not universally supported, consider using stat() for robustness
        if (direntp->d_type == DT_REG) {
            file_count++;
        }
    }

    // Close the directory
    closedir(dir_ptr);
    return file_count;
}

int numPlaces (int n) {
    if (n < 0) return numPlaces ((n == INT_MIN) ? INT_MAX: -n);
    if (n < 10) return 1;
    return 1 + numPlaces (n / 10);
}




static char* get_random_filename(file_rarity_t rarity) {
    const static char* TAG = "get_random_filename";
    // First decide which folder type we want
    // If any, roll random between 0-3
    if (rarity == ANY) {
        rarity = randrange(0,3);
    }
    ESP_LOGI(TAG, "Rarity: %d", rarity);
    // Get foldername
    char* rarityName = name_from_rarity(rarity);
    char* foldername = malloc(strlen(rarityName) + strlen(AUDIO_FOLDER) + 2);
    strcpy(foldername, AUDIO_FOLDER);
    strcat(foldername, rarityName);


    uint8_t fileNum = count_files_in_directory(foldername);

    int selectedFileNum = randrange(0, fileNum);
    char* fileName = malloc(strlen(foldername) + numPlaces(selectedFileNum) + 5);
    strcpy(fileName, foldername);
    strcat(fileName, "/");
    strcat(fileName, (const char*) ('0' + selectedFileNum));
    strcat(fileName, ".wav");
    ESP_LOGI(TAG, "File name: %s", fileName);
    free(foldername);
    return fileName;
}


// ---------------- WAV playback (task context) ----------------
static void consumer_i2s_task(void* arg) {
    const static char* TAG = "consumer_i2s_task";
    decoded_wav_t* wav_s = (decoded_wav_t*)arg;
    uint8_t* chunk = heap_caps_malloc(I2S_WRITE_CHUNK, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    i2s_init(wav_s->s_buffer_sample_rate, 1);
    for (;;) {
        // Wait until at least one byte is available, then read up to chunk size
        size_t got = xStreamBufferReceive(wav_s->s_reader_buffer, chunk, I2S_WRITE_CHUNK, pdMS_TO_TICKS(2000));
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
    vPortFree(chunk);
    i2s_driver_uninstall(I2S_NUM_0);
    xEventGroupSetBits(s_done_group, BIT_CONSUMER_DONE);
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
    vPortFree(tmp);
    fclose(wav_s->s_file_handle);
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
    free(filename);
    filename = NULL;
    if (!s_done_group) s_done_group = xEventGroupCreate();

    for (;;) {
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

    trigger_gpio_init();



    xTaskCreatePinnedToCore(player_task, "player_task",
                            20000,
                            NULL, 5, NULL, 0);

}
