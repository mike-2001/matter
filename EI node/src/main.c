/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/kernel.h>
#include <ei_wrapper.h>

#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/uart.h>

#define RECEIVE_BUFF_SIZE 16000
#define RECEIVE_TIMEOUT 100

static uint8_t rx_buf[sizeof(float)];
float audio_buf[16000];
int cnt = 0;

/* EI results ready */
static void result_ready_cb(int err)
{
	if(err) {
		printk("Result ready callback error: %d", err);
		return;
	}

	const char label[3];
	float value[3];
	size_t *inx;

	while (true) {
		err = ei_wrapper_get_next_classification_result(&label, &value, &inx);
		if (err) {
			printk("Unable to get next classification result: %d", err);
			break;
		}
		for (int i = 0; i < 3; i++) {
			printk("Predictions:\n");
			printk("%s: %f\n", label[i], value[i]);
			break;
		}
	}

	if (!err) {
		if (ei_wrapper_classifier_has_anomaly()){
			float anomaly;
			err = ei_wrapper_get_anomaly(&anomaly);
		}
	}

	bool cancelled;
	err = ei_wrapper_clear_data(&cancelled);
	if(err){
		printk("Unable to clear data: %i", err);
	}
}

/* UART callback */
static void uart_cb(const struct device *dev, struct uart_event *evt, void *user_data)
{
	int ret;
	switch (evt->type) {
	case UART_RX_RDY: 
		/* Copy data to a buffer */
		for (int i = 0; i < sizeof(float); i++) {
			audio_buf[cnt] = (float) rx_buf[i];
			cnt++;
		}
		if (cnt == RECEIVE_BUFF_SIZE) {
			/* Reset counter */
			cnt = 0;
			/* Do EI prediction */
			ret = ei_wrapper_add_data(&audio_buf, ei_wrapper_get_window_size());
			if (ret) {
				printk("Cannot provide input data (err: %d)\n", ret);
				printk("Increase CONFIG_EI_WRAPPER_DATA_BUF_SIZE\n");
			}
			ei_wrapper_start_prediction(0,0);
		}
		break;
	case UART_RX_DISABLED: /* Enable RX */
		uart_rx_enable(dev, rx_buf, sizeof rx_buf, RECEIVE_TIMEOUT);
		break;
	default:
		break;
	}
}

int main(void)
{
	int err;
	/* UART */
	const struct device *uart= DEVICE_DT_GET(DT_NODELABEL(uart1));
	if (!device_is_ready(uart)){
		printk("device_is_ready(uart) error: %d\n", err);
		return 1;
	}

	err = uart_callback_set(uart, uart_cb, NULL);
	if (err) {
		printk("uart_callback_set error: %d\n", err);
		return 1;
	}
	/* Start receiving data */
	err = uart_rx_enable(uart, rx_buf, sizeof rx_buf, RECEIVE_TIMEOUT);
	if (err) {
		printk("uart_rx_enable error: %d\n", err);
		return 1;
	}

	/* EI */
	err = ei_wrapper_init(result_ready_cb);
	if (err) {
		printk("ei_wrapper_init error: %d\n", err);
		return 0;
	};

	while (true) {
	}
	return 0;
}
