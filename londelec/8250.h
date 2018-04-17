/*
 *  Driver for 8250/16550-type serial ports
 *
 *  Based on drivers/char/serial.c, by Linus Torvalds, Theodore Ts'o.
 *
 *  Copyright (C) 2001 Russell King.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/version.h>
#include <linux/serial_8250.h>
#include <linux/serial_reg.h>
#include <linux/dmaengine.h>


int adv_serial8250_init(void);
void adv_serial8250_exit(void);
int adv_serial8250_register_port(struct uart_8250_port *up);
void adv_serial8250_unregister_port(int line);
void adv_default_serial_dl_write(struct uart_8250_port *up, int value);
int adv_serial8250_do_startup(struct uart_port *port);
void adv_serial8250_do_shutdown(struct uart_port *port);


struct uart_8250_dma {
	int (*tx_dma)(struct uart_8250_port *p);
	int (*rx_dma)(struct uart_8250_port *p, unsigned int iir);

	/* Filter function */
	dma_filter_fn		fn;
	/* Parameter to the filter function */
	void			*rx_param;
	void			*tx_param;

	struct dma_slave_config	rxconf;
	struct dma_slave_config	txconf;

	struct dma_chan		*rxchan;
	struct dma_chan		*txchan;

	dma_addr_t		rx_addr;
	dma_addr_t		tx_addr;

	dma_cookie_t		rx_cookie;
	dma_cookie_t		tx_cookie;

	void			*rx_buf;

	size_t			rx_size;
	size_t			tx_size;

	unsigned char		tx_running;
	unsigned char		tx_err;
	unsigned char		rx_running;
};

struct old_serial_port {
	unsigned int uart;
	unsigned int baud_base;
	unsigned int port;
	unsigned int irq;
	upf_t        flags;
	unsigned char hub6;
	unsigned char io_type;
	unsigned char __iomem *iomem_base;
	unsigned short iomem_reg_shift;
	unsigned long irqflags;
};


#define TX_BUF_TOT_LEN 128
#define RX_BUF_TOT_LEN 128
//DMA registers
#define DMAADL (0x100 + 0x00)		//DMA Address Low
#define DMAADH (0x100 + 0x04)		//DMA Address High
#define DMATL  (0x100 + 0x08)		//DMA Transfer Length
#define DMASTA (0x100 + 0x0c)  		//DMA Status

//DMA Status
#define DMAREAD	(1UL <<  31)
#define DMAACT	(1UL << 0)
#define DMAERR	(1UL << 1)
#define DMADONE	(1UL << 2)

#ifndef CDTRDSR
#define CDTRDSR	  004000000000  /* DTR/DSR flow control */
#endif

#define UART_CAP_FIFO	(1 << 8)	/* UART has FIFO */
#define UART_CAP_EFR	(1 << 9)	/* UART has EFR */
#define UART_CAP_SLEEP	(1 << 10)	/* UART has IER sleep */
#define UART_CAP_AFE	(1 << 11)	/* MCR-based hw flow control */
#define UART_CAP_UUE	(1 << 12)	/* UART needs IER bit 6 set (Xscale) */
#define UART_CAP_RTOIE	(1 << 13)	/* UART needs IER bit 4 set (Xscale, Tegra) */
#define UART_CAP_HFIFO	(1 << 14)	/* UART has a "hidden" FIFO */
#define UART_CAP_RPM	(1 << 15)	/* Runtime PM is active while idle */

#define UART_BUG_QUOT	(1 << 0)	/* UART has buggy quot LSB */
#define UART_BUG_TXEN	(1 << 1)	/* UART has buggy TX IIR status */
#define UART_BUG_NOMSR	(1 << 2)	/* UART has buggy MSR status bits (Au1x00) */
#define UART_BUG_THRE	(1 << 3)	/* UART has buggy THRE reassertion */
#define UART_BUG_PARITY	(1 << 4)	/* UART mishandles parity if FIFO enabled */

#define HIGH_BITS_OFFSET ((sizeof(long)-sizeof(int))*8)

#ifdef CONFIG_SERIAL_8250_SHARE_IRQ
#define SERIAL8250_SHARE_IRQS 1
#else
#define SERIAL8250_SHARE_IRQS 0
#endif


/*
 * The special register set for XR17v25x UARTs.
 */

#define XR_17V25X_EXTENDED_FCTR		8
#define XR_17V25X_EXTENDED_EFR		9
#define XR_17V25X_TXFIFO_CNT		10
#define XR_17V25X_RXFIFO_CNT		11
#define XR_17V25X_EXTENDED_RXTRG	11

#define XR_17V25X_FCTR_RTS_8DELAY	0x03
#define XR_17V25X_FCTR_TRGD		192

/* 17V15X TX/RX memory mapped buffer offsets */

#define UART_17V25X_RX_OFFSET		0x100
#define UART_17V25X_TX_OFFSET 		0x100

/*
 * These are the EXTENDED definitions for the 17V25X's Interrupt
 * Enable Register
 */
#define	XR_17V25X_IER_RTSDTR	0x40
#define XR_17V25X_IER_CTSDSR	0x80


static inline int serial_in(struct uart_8250_port *up, int offset)
{
	return up->port.serial_in(&up->port, offset);
}

static inline void serial_out(struct uart_8250_port *up, int offset, int value)
{
	up->port.serial_out(&up->port, offset, value);
}

#if defined(__alpha__) && !defined(CONFIG_PCI)
/*
 * Digital did something really horribly wrong with the OUT1 and OUT2
 * lines on at least some ALPHA's.  The failure mode is that if either
 * is cleared, the machine locks up with endless interrupts.
 */
#define ALPHA_KLUDGE_MCR  (UART_MCR_OUT2 | UART_MCR_OUT1)
#elif defined(CONFIG_SBC8560)
/*
 * WindRiver did something similarly broken on their SBC8560 board. The
 * UART tristates its IRQ output while OUT2 is clear, but they pulled
 * the interrupt line _up_ instead of down, so if we register the IRQ
 * while the UART is in that state, we die in an IRQ storm. */
#define ALPHA_KLUDGE_MCR (UART_MCR_OUT2)
#else
#define ALPHA_KLUDGE_MCR 0
#endif

#define LE485FLAG_DMA	0x1000


static inline int serial_index(struct uart_port *port)
{
	return port->minor - 64;
}

#if 0
#define DEBUG_INTR(fmt...)	printk(fmt)
#else
#define DEBUG_INTR(fmt...)	do { } while (0)
#endif
