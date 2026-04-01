#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <math.h>

#include <zephyr/audio/codec.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2s.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#define SAMPLE_BIT_WIDTH   16
#define NUMBER_OF_CHANNELS 2U
#define SAMPLES_PER_BLOCK  128U
#define BLOCK_SIZE         (SAMPLES_PER_BLOCK * sizeof(int16_t) * NUMBER_OF_CHANNELS)
#define BLOCK_COUNT        4U
#define TWO_POW_32         4294967296.0
#ifndef M_PI
#define M_PI ((double)3.14159265358979323846)
#endif

#define TEST_FREQ_DOUBLE 440.0

K_MEM_SLAB_DEFINE(mem_slab, BLOCK_SIZE, BLOCK_COUNT, 4);

/* LED toggles for each block write. */
#define LED0_NODE DT_ALIAS(led0)
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
#define I2S_CODEC_TX DT_ALIAS(i2s_codec_tx)
const struct device *i2s_dev;
#define AUDIO_CODEC DT_ALIAS(audio_codec)
const struct device *codec_dev;

typedef struct {
	int16_t l, r;
} frame_t;

/* Sine generator state. */
typedef struct {
	uint32_t phase;     /* 0 .. 2^32-1  relates to 0 .. 2*pi. */
	uint32_t phase_inc; /* Phase increment per sample. */
	int16_t amplitude;  /* Ampitude of the sine. */
} sine_gen_t;

static sine_gen_t gen;

/* Set frequency, sampling rate, amplitude. */
void sine_gen_set(sine_gen_t *g, double freq_hz, double fs_hz, int16_t amplitude)
{
	double cycles_per_sample = freq_hz / fs_hz;
	cycles_per_sample -= floor(cycles_per_sample);
	g->phase_inc = (uint32_t)(cycles_per_sample * TWO_POW_32);
	g->amplitude = amplitude;
}

/* Fill buffer with n frames (same value for both channels). */
void sine_gen_fill(sine_gen_t *g, frame_t *buf, size_t n)
{
	for (size_t i = 0; i < n; i++) {
		double angle = (double)g->phase * (2.0 * M_PI / TWO_POW_32);
		int16_t s = (int16_t)lrint(sin(angle) * g->amplitude);
		buf[i].l = s;
		buf[i].r = s;
		g->phase += g->phase_inc;
	}
}

/* Example:
 *
 *   static frame_t buffer[256];
 *   static sine_gen_t gen;
 *
 *   sine_gen_set(&gen, 1000.0, 48000.0, 30000);
 *   sine_gen_fill(&gen, buffer, 256);
 *
 */

static void audio_stream_loop(void)
{
	frame_t *samples = NULL;

	/* 1. Pre-fill buffer. */
	for (int i = 0; i < BLOCK_COUNT; i++) {
		void *mem_block;
		if (k_mem_slab_alloc(&mem_slab, &mem_block, K_FOREVER) < 0) {
			continue;
		}

		samples = (frame_t *)mem_block;
		sine_gen_fill(&gen, samples, SAMPLES_PER_BLOCK);

		if (i2s_write(i2s_dev, mem_block, BLOCK_SIZE) < 0) {
			printk("i2s_write failed during prefill\n");
		}
	}

	/* 2. Start I2S. */
	if (i2s_trigger(i2s_dev, I2S_DIR_TX, I2S_TRIGGER_START) < 0) {
		printk("I2S trigger start failed\n");
		return;
	}

	/* 3. Continuously generate new blocks. */
	while (1) {
		void *mem_block;

		if (k_mem_slab_alloc(&mem_slab, &mem_block, K_FOREVER) < 0) {
			continue;
		}

		samples = (frame_t *)mem_block;
		sine_gen_fill(&gen, samples, SAMPLES_PER_BLOCK);

		gpio_pin_toggle_dt(&led);

		if (i2s_write(i2s_dev, mem_block, BLOCK_SIZE) < 0) {
			printk("i2s_write failed\n");
		}
	}
}

int main(void)
{
	printk("Sine Test\n");

	sine_gen_set(&gen, TEST_FREQ_DOUBLE, (double)CONFIG_SAMPLE_FREQ, 30000);

	i2s_dev = DEVICE_DT_GET(I2S_CODEC_TX);
	if (!device_is_ready(i2s_dev)) {
		printk("I2S device not ready\n");
		return -1;
	}

	codec_dev = DEVICE_DT_GET(AUDIO_CODEC);
	if (!device_is_ready(codec_dev)) {
		printk("Audio codec not ready\n");
		return -1;
	}

	if (!gpio_is_ready_dt(&led)) {
		return 0;
	}

	gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);

	struct i2s_config cfg = {0};
	cfg.word_size = SAMPLE_BIT_WIDTH;
	cfg.channels = NUMBER_OF_CHANNELS;
	cfg.format = I2S_FMT_DATA_FORMAT_I2S;
	cfg.options = I2S_OPT_FRAME_CLK_CONTROLLER | I2S_OPT_BIT_CLK_CONTROLLER;
	cfg.frame_clk_freq = CONFIG_SAMPLE_FREQ;
	cfg.mem_slab = &mem_slab;
	cfg.block_size = BLOCK_SIZE;
	cfg.timeout = 2000U;

	if (i2s_configure(i2s_dev, I2S_DIR_TX, &cfg) < 0) {
		printk("I2S configure failed\n");
		return -1;
	}

	struct audio_codec_cfg codec_cfg = {
		.dai_type = AUDIO_DAI_TYPE_I2S,
		.dai_route = AUDIO_ROUTE_PLAYBACK,
		.dai_cfg.i2s =
			{
				.word_size = SAMPLE_BIT_WIDTH,
				.channels = NUMBER_OF_CHANNELS,
				.format = I2S_FMT_DATA_FORMAT_I2S,
				.options = I2S_OPT_BIT_CLK_TARGET | I2S_OPT_FRAME_CLK_TARGET,
				.frame_clk_freq = CONFIG_SAMPLE_FREQ,
			},
	};

	int ret = audio_codec_configure(codec_dev, &codec_cfg);

	if (ret < 0) {
		printk("Codec configure failed: %d\n", ret);
		return -1;
	}

	audio_codec_start_output(codec_dev);

	audio_stream_loop();

	return 0;
}
