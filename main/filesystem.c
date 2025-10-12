//
// Created by Adam Gilbert on 10/11/25.
//


#include <esp_log.h>
#include <limits.h>
#include <sys/dirent.h>
#include "alarm.h"
#include "freertos/FreeRTOS.h"

int count_files_in_directory(const char *path) {
    const static char* TAG = "count_files_in_directory";
    DIR *dir_ptr;
    struct dirent *direntp;
    int file_count = 0;
    ESP_LOGI(TAG, "Path: %s", path);

    // Open the directory
    if ((dir_ptr = opendir(path)) == NULL) {
        ESP_LOGE(TAG, "Failed to open directory");
        return -1; // Indicate an error
    }

    // Read directory entries
    while ((direntp = readdir(dir_ptr)) != NULL) {
        // Skip "." and ".." entries
        if (strcmp(direntp->d_name, ".") == 0 || strcmp(direntp->d_name, "..") == 0 || strncmp(direntp->d_name, ".", 1) == 0) {
            continue;
        }

        //Check if the entry is a regular file
        if (direntp->d_type == DT_REG) {
            file_count++;
        }
    }

    // Close the directory
    closedir(dir_ptr);
    return file_count;
}

int numPlaces (int n) {
    int r = 1;
    if (n < 0) n = (n == INT_MIN) ? INT_MAX: -n;
    while (n > 9) {
        n /= 10;
        r++;
    }
    return r;
}



char* get_random_filename(file_rarity_t rarity) {
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

    int selectedFileNum = randrange(0, fileNum-1);
    unsigned int decPlacesInFileNum = numPlaces(selectedFileNum);
    ESP_LOGI(TAG, "Places in file: %d", decPlacesInFileNum);
    size_t fileNameLen = strlen(foldername) + decPlacesInFileNum + 6;
    ESP_LOGI(TAG, "File name len: %d", fileNameLen);
    char* fileName = malloc(fileNameLen);
    sprintf(fileName, "%s/%d.wav", foldername, selectedFileNum);
    ESP_LOGI(TAG, "File name: %s", fileName);
    free(foldername);
    return fileName;
}

esp_err_t prepare_wav_file(decoded_wav_t* wav_s, const char *filename) {
    const static char* TAG = "prepare_wav_file";
    ESP_LOGI(TAG, "Preparing wav file %s", filename);
    // Setup the stream buffer
    if (wav_s->s_reader_buffer != NULL) vStreamBufferDelete(wav_s->s_reader_buffer);
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

    uint8_t* pre = heap_caps_malloc(SD_READ_CHUNK, MALLOC_CAP_DMA);
    wav_s->s_reader_offset = 0;
    while (wav_s->s_reader_offset < (RB_CAPACITY)) {
        const size_t n = fread(pre, 1, SD_READ_CHUNK, fh);
        heap_caps_check_integrity_all(1);
        if (n == 0) break;
        size_t sent = 0;
        while (sent < n) {
            sent += xStreamBufferSend(wav_s->s_reader_buffer, pre + sent, n - sent, portMAX_DELAY);
        }
        wav_s->s_reader_offset += n;
    }
    heap_caps_free(pre);


    ESP_LOGI(TAG, "WAV file loaded. Leaving the following struct behind:");
    ESP_LOGI(TAG, "Sample rate: %d", wav_s->s_buffer_sample_rate);
    ESP_LOGI(TAG, "Buffer size: %d", wav_s->s_buffer_size);
    ESP_LOGI(TAG, "Number of samples: %d", wav_s->s_number_of_samples);
    return ESP_OK;
}
