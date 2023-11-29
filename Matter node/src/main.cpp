/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include "app_task.h"
#include <zephyr/device.h>
#include <zephyr/audio/dmic.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>


LOG_MODULE_REGISTER(app, CONFIG_CHIP_APP_LOG_LEVEL);

/* DMIC configure */
#define MAX_SAMPLE_RATE  16000
#define SAMPLE_BIT_WIDTH 16
#define BYTES_PER_SAMPLE sizeof(int16_t)
#define READ_TIMEOUT     1000

/* Size of a block for 100 ms of audio data. */
#define BLOCK_SIZE(_sample_rate, _number_of_channels) (BYTES_PER_SAMPLE * (_sample_rate / 10) * _number_of_channels)
#define MAX_BLOCK_SIZE   BLOCK_SIZE(MAX_SAMPLE_RATE, 2)
#define BLOCK_COUNT      4

K_MEM_SLAB_DEFINE_STATIC(mem_slab, MAX_BLOCK_SIZE, BLOCK_COUNT, 4);

float audio[16000];

/* DMIC audio */
static int do_pdm_transfer(const struct device *dmic_dev, struct dmic_cfg *cfg, size_t block_count) {
	int ret;

	LOG_INF("Starting sampling:");

	/* Start the microphone. */
	ret = dmic_trigger(dmic_dev, DMIC_TRIGGER_START);
	if (ret) {
		LOG_INF("dmic_trigger failed: %d\n", ret);
		return ret;
	}
	
	for (int i = 0; i < 11; ++i) {
		void *buffer;
		uint32_t size;

		ret = dmic_read(dmic_dev, 0, &buffer, &size, READ_TIMEOUT);
		if (ret) {
			LOG_INF("dmic_read failed: %d\n", ret);
			break;
		}

		/* Discard first readout due to microphone needing to 
		 * stabilize before valid data can be read. */
		if (i != 0) {
			int16_t tempInt;
			float tempFloat;
			for(int j = 0; j < 1600; j++) {
				memcpy(&tempInt, buffer + 2*j, 2);
				tempFloat = (float)tempInt;
				audio[(i-1)*1600+j] = tempFloat;
			}
		}
		k_mem_slab_free(&mem_slab, &buffer);
	}

	/* Stop the microphone. */
	ret = dmic_trigger(dmic_dev, DMIC_TRIGGER_STOP);
	if (ret) {
		return ret;
	}
	return 0;
}

/* UART callback */
static void uart_cb(const struct device *dev, struct uart_event *evt, void *user_data)
{
	switch (evt->type) {
	case UART_TX_DONE: /* Transfer data done */
		LOG_INF("Transmit data successful\n");
	case UART_TX_ABORTED:
		LOG_INF("Transmit data failed\n");
	default:
		break;
	}
}

int main()
{
	int ret;

	/* Init Matter */
	CHIP_ERROR err = AppTask::Instance().StartApp();
	LOG_ERR("Exited with code %" CHIP_ERROR_FORMAT, err.Format());
	return err == CHIP_NO_ERROR ? EXIT_SUCCESS : EXIT_FAILURE;

	/* Init UART1 */
	const struct device *uart= DEVICE_DT_GET(DT_NODELABEL(uart1));
	if (!device_is_ready(uart)){
		LOG_ERR("UART device not ready\n");
		return 1;
	}

	ret = uart_callback_set(uart, uart_cb, NULL);
	if (ret) {
		printk("uart_callback_set error: %d\n", ret);
		return 1;
	}

	/* Init DMIC */
	const struct device *dmic_dev = DEVICE_DT_GET(DT_NODELABEL(dmic_dev));
	if (!device_is_ready(dmic_dev)) {
		LOG_ERR("DMIC device not ready\n");
        return EXIT_FAILURE;
	}

	/* DMIC configuration */
	struct pcm_stream_cfg stream = {
		.pcm_width = SAMPLE_BIT_WIDTH,
		.mem_slab  = &mem_slab,
	};
	
	struct dmic_cfg cfg = {
		.io = {
			.min_pdm_clk_freq = 1100000,
			.max_pdm_clk_freq = 3500000,
			.min_pdm_clk_dc   = 40,
			.max_pdm_clk_dc   = 60,
		},
		.streams = &stream,
		.channel = {
			.req_num_streams = 1,
		},
	};

	cfg.channel.req_num_chan = 1;
	cfg.channel.req_chan_map_lo = dmic_build_channel_map(0, 0, PDM_CHAN_LEFT);
	cfg.streams[0].pcm_rate = MAX_SAMPLE_RATE;
	cfg.streams[0].block_size = BLOCK_SIZE(cfg.streams[0].pcm_rate, cfg.channel.req_num_chan);

	ret = dmic_configure(dmic_dev, &cfg);
	if (ret) {
		LOG_ERR("dmic_configure failed: %d\n", ret);
        return EXIT_FAILURE;
	}

	/* main loop */
	while(1) {
		/* Record audio */
		ret = do_pdm_transfer(dmic_dev, &cfg, 2 * BLOCK_COUNT);
		if (ret) {
			LOG_ERR("do_pdm_transfer failed: %d\n", ret);
			return EXIT_FAILURE;
		}
		/* Transfer data via UART */
		for (int i = 0; i < 16000; i++) {
			char audio_buf[sizeof(float)];
			uint8_t tx_buf[sizeof(float)];
			sprintf(audio_buf, "%0.7f", audio[i]);
			memcpy(tx_buf, audio_buf, sizeof(float));
			ret = uart_tx(uart, tx_buf, sizeof(tx_buf), SYS_FOREVER_US);
			if (ret) {
				return EXIT_FAILURE;
			}
		}
		k_sleep(K_SECONDS(2));
	}
}
