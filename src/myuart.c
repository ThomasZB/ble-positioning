#include "myuart.h"


#define UART_BUF_SIZE 20
#define UART_RX_TIMEOUT 50


const struct device *uart;

struct uart_data_t {
	void *fifo_reserved;
	uint8_t  data[UART_BUF_SIZE];
	uint16_t len;
};


static void uart_cb(const struct device *dev, struct uart_event *evt, void *user_data);


int uart_init(void)
{
	int err;
	static struct uart_data_t rx;

	uart = device_get_binding(DT_LABEL(DT_NODELABEL(uart0)));
	if (!uart) {
		return -ENXIO;
	}

	err = uart_callback_set(uart, uart_cb, NULL);
	if (err) {
		return err;
	}

	return uart_rx_enable(uart, rx.data, sizeof(rx.data),
			      UART_RX_TIMEOUT);
}

static void uart_cb(const struct device *dev, struct uart_event *evt, void *user_data)
{

}