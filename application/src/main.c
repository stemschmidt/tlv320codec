#include <zephyr/kernel.h>
#include <zephyr/drivers/i2s.h>
#include <zephyr/sys/printk.h>
#include <math.h>
#include <stdint.h>

#define SAMPLE_BIT_WIDTH 16U
#define NUMBER_OF_CHANNELS 2U
#define BLOCK_DURATION_MS 100U

#define SAMPLES_PER_BLOCK ((CONFIG_SAMPLE_FREQ * BLOCK_DURATION_MS) / 1000 * NUMBER_OF_CHANNELS)
#define BYTES_PER_SAMPLE (SAMPLE_BIT_WIDTH / 8U)
#define BLOCK_SIZE (SAMPLES_PER_BLOCK * BYTES_PER_SAMPLE)
#define BLOCK_COUNT 4U

#ifndef M_PI
#define M_PI ((float)3.14159265358979323846)
#endif

/* DMA-tauglicher Slab für Zero-Copy */
K_MEM_SLAB_DEFINE(mem_slab, BLOCK_SIZE, BLOCK_COUNT, 4);

/* I2S-Gerät (TX) */
#define I2S_CODEC_TX DT_ALIAS(i2s_codec_tx)
const struct device* i2s_dev;

/* Fixed-Point Q15 Oszillator */
static float amplitude = 30000.0; // max int16 für Audio
static float freq = 440.0f;  // Frequenz A4

/* Blockweise Sinus-Generierung */
static void generate_sine_block(int16_t *samples, size_t frames)
{
    float phase = 0.0;
    for (size_t i = 0; i < frames; i++) {
      phase += 2.0f * M_PI * freq / CONFIG_SAMPLE_FREQ;
      int16_t val16 = (int16_t)(amplitude * sinf(phase));

      /* Stereo interleaved */
      samples[2*i + 0] = val16;
      samples[2*i + 1] = val16;
    }
}

/* Audio-Streaming Loop mit Vorbefüllung */
static void audio_stream_loop(void)
{
    size_t frames = SAMPLES_PER_BLOCK / NUMBER_OF_CHANNELS;

    /* 1. Vorab-Blöcke füllen */
    for (int i = 0; i < BLOCK_COUNT; i++) {
        void *mem_block;
        if (k_mem_slab_alloc(&mem_slab, &mem_block, K_FOREVER) < 0) continue;
        int16_t *samples = (int16_t *)mem_block;
        generate_sine_block(samples, frames);

        if (i2s_write(i2s_dev, mem_block, BLOCK_SIZE) < 0) {
            printk("i2s_write failed during prefill\n");
        }
    }

    /* 2. I2S starten */
    if (i2s_trigger(i2s_dev, I2S_DIR_TX, I2S_TRIGGER_START) < 0) {
        printk("I2S trigger start failed\n");
        return;
    }

    /* 3. Endlosschleife: kontinuierlich neue Blöcke generieren */
    while (1) {
        void *mem_block;
        int16_t *samples;

        if (k_mem_slab_alloc(&mem_slab, &mem_block, K_FOREVER) < 0) continue;

        samples = (int16_t *)mem_block;
        generate_sine_block(samples, frames);

        if (i2s_write(i2s_dev, mem_block, BLOCK_SIZE) < 0) {
            printk("i2s_write failed\n");
        }
    }
}

/* Main-Funktion */
int main(void)
{
    printk("Zero-Copy I2S Fixed-Point Sine Test\n");

    i2s_dev = DEVICE_DT_GET(I2S_CODEC_TX);
    if (!device_is_ready(i2s_dev)) {
        printk("I2S device not ready\n");
        return -1;
    }

    struct i2s_config cfg = {0};
    cfg.word_size = SAMPLE_BIT_WIDTH;
    cfg.channels = NUMBER_OF_CHANNELS;
    cfg.format = I2S_FMT_DATA_FORMAT_I2S;
#ifdef CONFIG_USE_CODEC_CLOCK
    cfg.options = I2S_OPT_FRAME_CLK_MASTER | I2S_OPT_BIT_CLK_MASTER;
#else
    cfg.options = I2S_OPT_FRAME_CLK_SLAVE | I2S_OPT_BIT_CLK_SLAVE;
#endif
    cfg.frame_clk_freq = CONFIG_SAMPLE_FREQ;
    cfg.mem_slab = &mem_slab;
    cfg.block_size = BLOCK_SIZE;
    cfg.timeout = 2000U;

    if (i2s_configure(i2s_dev, I2S_DIR_TX, &cfg) < 0) {
        printk("I2S configure failed\n");
        return -1;
    }

    /* Starte Audio-Streaming */
    audio_stream_loop();

    return 0;
}