//
// Created by Adam Gilbert on 10/11/25.
//

#ifndef ALARM_ALARM_H
#define ALARM_ALARM_H

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


char* name_from_rarity(file_rarity_t rarity);
int randrange(int min, int max);

#endif //ALARM_ALARM_H