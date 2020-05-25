#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_system.h"
// #include "esp_spi_flash.h"
#include "xtensa/core-macros.h"
// #include "esp_partition.h"
#include "speech_srcif.h"
#include "driver/i2s.h"
#include "esp_log.h"
#include "esp_spiffs.h"
// #include "app_main.h"
#include "fft.h"
QueueHandle_t sndQueue;
QueueHandle_t ledQueue;
static src_cfg_t srcif;

static void i2s_init(void)
{
    i2s_config_t i2s_config = {
        .mode = I2S_MODE_MASTER | I2S_MODE_RX,//the mode must be set according to DSP configuration
        .sample_rate = 48000,                           //must be the same as DSP configuration
        .channel_format = I2S_CHANNEL_FMT_ALL_LEFT,   //must be the same as DSP configuration
        .bits_per_sample = 32,                          //must be the same as DSP configuration
        .communication_format = I2S_COMM_FORMAT_I2S,
        .dma_buf_count = 3,
        .dma_buf_len = 300,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL2,
    };
    i2s_pin_config_t pin_config = {
        .bck_io_num = 14,  // IIS_SCLK
        .ws_io_num = 32,   // IIS_LCLK
        .data_out_num = -1,// IIS_DSIN
        .data_in_num = 33  // IIS_DOUT
    };
    i2s_driver_install(1, &i2s_config, 0, NULL);
    i2s_set_pin(1, &pin_config);
    i2s_zero_dma_buffer(1);
}

void recsrcTask(void *arg)
{
    i2s_init();

    src_cfg_t *cfg=(src_cfg_t*)arg;
    size_t samp_len = cfg->item_size*2*sizeof(int)/sizeof(int16_t);
    printf("item_size =%d,sample len =%d\n",cfg->item_size,samp_len);
    int *samp=malloc(samp_len);

    size_t read_len = 0;

    while(1) {
        // if (g_state != WAIT_FOR_WAKEUP)
        // {
        //     vTaskDelay(1000 / portTICK_PERIOD_MS);
        //     continue;
        // }

        i2s_read(1, samp, samp_len, &read_len, portMAX_DELAY);
        for (int x=0; x<cfg->item_size/4; x++) {
            int s1 = ((samp[x * 4] + samp[x * 4 + 1]) >> 13) & 0x0000FFFF;
            int s2 = ((samp[x * 4 + 2] + samp[x * 4 + 3]) << 3) & 0xFFFF0000;
            samp[x] = s1 | s2;
        }

        xQueueSend(*cfg->queue, samp, portMAX_DELAY);
    }

    vTaskDelete(NULL);
}
void nnTask(void *arg)
{
    // int audio_chunksize = model->get_samp_chunksize(model_data);
    int16_t *buffer=malloc(32*4*sizeof(int16_t));//32,1通道
    assert(buffer);
    int16_t wave[16];
    while(1) 
    {
        xQueueReceive(sndQueue, buffer, portMAX_DELAY);
        fft_config_t *fft_analysis = fft_init(32, FFT_REAL, FFT_FORWARD, NULL, NULL);

        // Fill array with some dummy data
        for (int k = 0 ; k < 16 ; k++)
        {
            fft_analysis->input[2*k] = buffer[k];
            fft_analysis->input[2*k+1]=0;
        }

        // Test accuracy
        fft_execute(fft_analysis);
        // for(int i=0;i<16;i++)
        // {
        //     printf("%1.0f ",fft_analysis->input[i]);
        // }
        // printf("#");
        for(int i=0;i<16;i++)
        {
            wave[i]=sqrt(fft_analysis->output[2*i]*fft_analysis->output[2*i]+fft_analysis->output[2*i+1]*fft_analysis->output[2*i+1]);
            wave[i]=10.0*wave[i]/6000;
            if(i==0)
            wave[i]-=4;
            if(wave[i]<0)
            {
                wave[i]=0;
            }
            if(wave[i]>10)
            {
                wave[i]=10;
            }
            // printf("%1.0f ",fft_analysis->output[i]);
        }
        xQueueSend(ledQueue, wave, portMAX_DELAY);
        // // printf("% 10.0f",fft_analysis->output[1]);
        // printf("          ");
        // printf("\r");
        // for(int i=0;i<wave[1];i++)
        // printf("=");
        // printf("\r");
        fft_destroy(fft_analysis);
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    free(buffer);
    vTaskDelete(NULL);
}

void speech_init()
{
    //Initialize sound source
    sndQueue=xQueueCreate(2, (32*4*sizeof(int16_t)));
    srcif.queue=&sndQueue;
    srcif.item_size=32*4*sizeof(int16_t);
    ledQueue=xQueueCreate(1, (16*sizeof(int16_t)));
    xTaskCreatePinnedToCore(&recsrcTask, "rec", 3*1024, (void*)&srcif, 5, NULL, 1);

    xTaskCreatePinnedToCore(&nnTask, "nn", 2*1024, NULL, 5, NULL, 1);
}
