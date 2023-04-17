/*
 *  Universal/legacy driver for 8250/16550-type serial ports
 *
 *  Based on drivers/char/serial.c, by Linus Torvalds, Theodore Ts'o.
 *
 *  Copyright (C) 2001 Russell King.
 *
 *  Supports: ISA-compatible 8250/16550 ports
 *	      PNP 8250/16550 ports
 *	      early_serial_setup() ports
 *	      userspace-configurable "phantom" ports
 *	      "serial8250" platform devices
 *	      serial8250_register_8250_port() ports
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/ioport.h>
//#include <linux/init.h>
#include <linux/console.h>
#include <linux/sysrq.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/tty.h>
#include <linux/ratelimit.h>
#include <linux/tty_flip.h>
#include <linux/serial.h>
#include <linux/serial_8250.h>
#include <linux/nmi.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/pm_runtime.h>
#ifdef CONFIG_SPARC
#include <linux/sunserialcore.h>
#endif
#include <linux/dma-mapping.h>
#include <asm/io.h>
#include <asm/irq.h>

#include "8250.h"

/*
 * Configuration:
 *   share_irqs - whether we pass IRQF_SHARED to request_irq().  This option
 *                is unsafe when used on edge-triggered interrupts.
 */
static unsigned int share_irqs = SERIAL8250_SHARE_IRQS;

static unsigned int nr_uarts = CONFIG_SERIAL_8250_RUNTIME_UARTS;

static struct uart_driver serial8250_reg;

static unsigned int skip_txen_test; /* force skip of txen test at init time */

#define PASS_LIMIT	512

#include <asm/serial.h>
/*
 * SERIAL_PORT_DFNS tells us about built-in ports that have no
 * standard enumeration mechanism.   Platforms that can find all
 * serial ports via mechanisms like ACPI or PCI need not supply it.
 */
#ifndef SERIAL_PORT_DFNS
#define SERIAL_PORT_DFNS
#endif

static const struct old_serial_port old_serial_port[] = {
	SERIAL_PORT_DFNS /* defined in asm/serial.h */
};

#define UART_NR	CONFIG_SERIAL_8250_NR_UARTS

#ifdef CONFIG_SERIAL_8250_RSA

#define PORT_RSA_MAX 4
static unsigned long probe_rsa[PORT_RSA_MAX];
static unsigned int probe_rsa_count;
#endif /* CONFIG_SERIAL_8250_RSA  */


#define ACR_DTR_RS232 				0x00
#define ACR_DTR_ACTIVE_LOW_RS485        	0x10
#define ACR_DTR_ACTIVE_HIGH_RS485       	0x18
#define PORT_XR17V25X 15


static void serial_icr_write(struct uart_8250_port *up, int offset, int value);
struct uart_8250_port *serial8250_get_port(int line);


/* Uart divisor latch write */
void adv_default_serial_dl_write(struct uart_8250_port *up, int value)
{
	 unsigned int baud;
	unsigned int dlldlm, cpr, tcr;

		/* code from windows
		if (DesiredBaud == 50)
		{
			pBaudConfig->Divisor = 18432;
			pBaudConfig->SampleClock = 16;
			pBaudConfig->Prescaler = 8;
			if (DeviceData->ClockRate == 62500000)
			{
				pBaudConfig->Prescaler = pBaudConfig->Prescaler * 4;
			}
			return status;
		}
		*/
		if (!value || (value == 1)) {	// Values 0 and 1 are used by autoconfig_read_divisor_id() and size_fifo() in 8250_port.c
			serial_out(up, UART_DLL, value & 0xff);
			serial_out(up, UART_DLM, value >> 8 & 0xff);
			return;
		}

                baud = 921600 / value;
                //printk(KERN_INFO "baud=%x,", baud);
		if ( baud == 50)
		{
			tcr = 16;
		}
		else
		{
			tcr = 4;
		}
		if ( value >= 4)
		{
			cpr = 32;
		}
		else
		{
			cpr = 8;
		}
		dlldlm = (int)((10*(3906250*16)/(tcr*cpr*baud/8)+5)/10);
		if ( baud < 19200)
		{
			dlldlm = dlldlm - 1;
		}
		serial_out(up, UART_LCR, up->lcr | UART_LCR_DLAB);	/* set DLAB */
		serial_out(up, UART_DLL, dlldlm & 0xff);		/* LS of divisor */
		serial_out(up, UART_DLM, dlldlm >> 8);
		serial_icr_write(up, UART_TCR, (tcr & 0x0F)); /* TCR[3:0]=SC */
		serial_icr_write(up, UART_CPR, cpr); /* CPR[7:3]=M */
		//printk("LEDEBUG: %s() baud=%u dlldlm=%u tcr=0x%02x cpr=0x%02x\n", __func__, baud, dlldlm, tcr, cpr);
}

/*
 * For the 16C950
 */
static void serial_icr_write(struct uart_8250_port *up, int offset, int value)
{
	serial_out(up, UART_SCR, offset);
	serial_out(up, UART_ICR, value);
}
/*
static unsigned int serial_icr_read(struct uart_8250_port *up, int offset)
{
	unsigned int value;

	serial_icr_write(up, UART_ACR, up->acr | UART_ACR_ICRRD);
	serial_out(up, UART_SCR, offset);
	value = serial_in(up, UART_ICR);
	serial_icr_write(up, UART_ACR, up->acr);

	return value;
}
*/

static void serial8250_rpm_put_tx(struct uart_8250_port *p)
{
	unsigned char rpm_active;

	if (!(p->capabilities & UART_CAP_RPM))
		return;

	rpm_active = xchg(&p->rpm_tx_active, 0);
	if (!rpm_active)
		return;
	pm_runtime_mark_last_busy(p->port.dev);
	pm_runtime_put_autosuspend(p->port.dev);
}

static inline void __stop_tx(struct uart_8250_port *p)
{
	if (p->ier & UART_IER_THRI) {
		p->ier &= ~UART_IER_THRI;
		serial_out(p, UART_IER, p->ier);
		serial8250_rpm_put_tx(p);
	}
}

/*
 * serial8250_rx_chars: processes according to the passed in LSR
 * value, and returns the remaining LSR bits not handled
 * by this Rx routine.
 */
static int
adv_serial8250_rx_chars(struct uart_8250_port *up, unsigned int iir)
{
	struct uart_port *port = &up->port;
	unsigned int count,insertcount;
	int flags;
	unsigned char save_rfl;
	unsigned char lsr;

	lsr = serial_port_in(port, UART_LSR);
	//Get number of chars in the receiver FIFO from RFL
	serial_out(up, UART_SCR, UART_ACR);
	serial_out(up, UART_ICR, up->acr|0x80);
	save_rfl = serial_in(up, UART_RFL);
	serial_out(up, UART_SCR, UART_ACR);
	serial_out(up, UART_ICR, up->acr);
	count  = (up->port.fifosize < save_rfl)? up->port.fifosize: save_rfl;
	//ensure tty core has enough space
	/*
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 9, 0)
	count  = tty_buffer_request_room(&port->state->port, count);
#else
	count  = tty_buffer_request_room(tty, count);
#endif
	*/

	//printk(KERN_INFO "RFL=%d,count=%d,", save_rfl, count);
	//firstly clear DMA status register
	writel(DMADONE|DMAERR, up->port.membase + DMASTA);
	//copy data to DMA buffer
	writel(up->dma->rx_addr, up->port.membase + DMAADL);
	writel(0x0, up->port.membase + DMAADH);
	writel(DMAREAD|count, up->port.membase + DMATL);
	do{
		flags = readl(up->port.membase + DMASTA);
      } while (flags & DMAACT);
	insertcount = tty_insert_flip_string(&port->state->port, (unsigned char *) up->dma->rx_param, count);
	up->port.icount.rx += insertcount;
	if ((lsr & UART_LSR_OE) || insertcount < count)
	{
		port->icount.overrun++;
		/*
		* Overrun is special.  Since it's reported immediately,
	 	* it doesn't affect the current character.
	 	*/

		if (lsr & ~port->ignore_status_mask & UART_LSR_OE)
		{
			if (tty_insert_flip_char(&port->state->port, 0, TTY_OVERRUN) == 0)
				++port->icount.buf_overrun;
		}
	}


	spin_unlock(&port->lock);
	tty_flip_buffer_push(&port->state->port);
	spin_lock(&port->lock);
	return 0;
}
//EXPORT_SYMBOL_GPL(serial8250_rx_chars);


static int adv_serial8250_tx_chars(struct uart_8250_port *up)
{
	struct uart_port *port = &up->port;
	struct circ_buf *xmit = &port->state->xmit;
	int count;
	unsigned int flags;
    	unsigned char save_tfl;

	if (port->x_char) {
		serial_out(up, UART_TX, port->x_char);
		port->icount.tx++;
		port->x_char = 0;
		return 0;
	}
	if (uart_tx_stopped(port)) {
		port->ops->stop_tx(port);
		return 0;
	}
	if (uart_circ_empty(xmit)) {
		__stop_tx(up);
		return 0;
	}

		//Get number of chars in the receiver FIFO from RFL
		serial_out(up, UART_SCR, UART_ACR);
		serial_out(up, UART_ICR, up->acr|0x80);
		save_tfl = serial_in(up, UART_TFL);
		serial_out(up, UART_SCR, UART_ACR);
		serial_out(up, UART_ICR, up->acr);
		//printk(KERN_INFO "save_tfl=%d,", save_tfl);
		//printk(KERN_INFO "TTL=%d,", serial_icr_read(up,UART_TTL));
		//count  = (up->port.fifosize < (128 - save_tfl))?up->port.fifosize:(128 - save_tfl);
		//count = MIN(count, up->port.fifosize >> 1);
		//count = MIN(count,uart_circ_chars_pending(xmit));
		count = (uart_circ_chars_pending(xmit) < (up->port.fifosize - save_tfl) )? uart_circ_chars_pending(xmit): (up->port.fifosize - save_tfl);

		if ((xmit->tail + count) > (UART_XMIT_SIZE - 1))
		{
			count = UART_XMIT_SIZE - xmit->tail;
		}
		//printk(KERN_INFO "count=%d\n", count);

		//firstly clear DMA status register
		writel(DMADONE|DMAERR, up->port.membase + DMASTA);
		//copy data to DMA buffer
		memcpy(up->dma->tx_param, &xmit->buf[xmit->tail], count);
		writel(up->dma->tx_addr, up->port.membase + DMAADL);
		writel(0x0, up->port.membase + DMAADH);
		writel(count, up->port.membase + DMATL);
		do{
			flags = readl(up->port.membase + DMASTA);
	    	} while (flags & DMAACT);

		xmit->tail = (xmit->tail + count) & (UART_XMIT_SIZE - 1);
		up->port.icount.tx += count;


	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(port);

	DEBUG_INTR("THRE...");

	/*
	 * With RPM enabled, we have to wait until the FIFO is empty before the
	 * HW can go idle. So we get here once again with empty FIFO and disable
	 * the interrupt and RPM in __stop_tx()
	 */
	if (uart_circ_empty(xmit) && !(up->capabilities & UART_CAP_RPM))
		__stop_tx(up);
	return 0;
}

/*
 * Londelec startup function to allocate DMA buffers and
 * set UART_ACR which enables DTR line that drives RS485 direction.
 */
int adv_serial8250_do_startup(struct uart_port *port)
{
	struct uart_8250_port *up = up_to_u8250p(port);

	int ret = serial8250_do_startup(port);

	if (port->rs485.flags & LE485FLAG_DMA) {	// Initialize DMA
		up->dma = kzalloc(sizeof(*up->dma), GFP_KERNEL);
		up->dma->rx_dma = adv_serial8250_rx_chars;
		up->dma->tx_dma = adv_serial8250_tx_chars;

		up->dma->tx_param = dma_alloc_coherent(up->port.dev, TX_BUF_TOT_LEN,
					   &up->dma->tx_addr, GFP_KERNEL);
		up->dma->rx_param = dma_alloc_coherent(up->port.dev, RX_BUF_TOT_LEN,
					   &up->dma->rx_addr, GFP_KERNEL);

		if (up->dma->tx_param == NULL || up->dma->rx_param == NULL) {
			if (up->dma->tx_param)
				dma_free_coherent(up->port.dev, TX_BUF_TOT_LEN,
					    up->dma->tx_param, up->dma->tx_addr);
			if (up->dma->rx_param)
				dma_free_coherent(up->port.dev, RX_BUF_TOT_LEN,
					    up->dma->rx_param, up->dma->rx_addr);
			return -ENOMEM;
		}
		//printk("LEDEBUG: %s() DMA init iobase=0x%x\n", __func__, (unsigned) port->iobase);
	}

	if (port->rs485.flags & SER_RS485_ENABLED) {	// Need to set UART_ACR after serial8250_do_startup()
		if (port->rs485.flags & SER_RS485_RTS_ON_SEND)
			up->acr = ACR_DTR_ACTIVE_HIGH_RS485;
		else
			up->acr = ACR_DTR_ACTIVE_LOW_RS485;
		//printk("LEDEBUG: %s() iobase=0x%x up->acr=0x%x\n", __func__, (unsigned) port->iobase, up->acr);
		serial_icr_write(up, UART_ACR, up->acr);
	}
	return ret;
}

/*
 * Londelec shutdown function to free DMA buffers before calling serial8250_do_shutdown()
 */
void adv_serial8250_do_shutdown(struct uart_port *port)
{
	struct uart_8250_port *up = up_to_u8250p(port);

	if (port->rs485.flags & LE485FLAG_DMA) {	// Free DMA
		dma_free_coherent(up->port.dev, RX_BUF_TOT_LEN,
				up->dma->rx_param, up->dma->rx_addr);
		dma_free_coherent(up->port.dev, TX_BUF_TOT_LEN,
				up->dma->tx_param, up->dma->tx_addr);
		kfree(up->dma);
		up->dma = NULL;
		//printk("LEDEBUG: %s() DMA free \n", __func__);
	}
	serial8250_do_shutdown(port);
}


static struct uart_8250_port serial8250_ports[UART_NR];


static void (*serial8250_isa_config)(int port, struct uart_port *up,
	u32 *capabilities);



/*
static void __init serial8250_isa_init_ports(void)
{
	struct uart_8250_port *up;
	static int first = 1;
	int i, irqflag = 0;

	if (!first)
		return;
	first = 0;

	if (nr_uarts > UART_NR)
		nr_uarts = UART_NR;

	for (i = 0; i < nr_uarts; i++) {
		struct uart_8250_port *up = &serial8250_ports[i];
		struct uart_port *port = &up->port;

		port->line = i;
		serial8250_init_port(up);
		if (!base_ops)
			base_ops = port->ops;
		port->ops = &univ8250_port_ops;

		timer_setup(&up->timer, serial8250_timeout, 0);

		up->ops = &univ8250_driver_ops;

		if (IS_ENABLED(CONFIG_ALPHA_JENSEN) ||
		    (IS_ENABLED(CONFIG_ALPHA_GENERIC) && alpha_jensen()))
			port->set_mctrl = alpha_jensen_set_mctrl;

		serial8250_set_defaults(up);
	}

	* chain base port ops to support Remote Supervisor Adapter *
	univ8250_port_ops = *base_ops;
	univ8250_rsa_support(&univ8250_port_ops);

	if (share_irqs)
		irqflag = IRQF_SHARED;

	for (i = 0, up = serial8250_ports;
	     i < ARRAY_SIZE(old_serial_port) && i < nr_uarts;
	     i++, up++) {
		struct uart_port *port = &up->port;

		port->iobase   = old_serial_port[i].port;
		port->irq      = irq_canonicalize(old_serial_port[i].irq);
		port->irqflags = 0;
		port->uartclk  = old_serial_port[i].baud_base * 16;
		port->flags    = old_serial_port[i].flags;
		port->hub6     = 0;
		port->membase  = old_serial_port[i].iomem_base;
		port->iotype   = old_serial_port[i].io_type;
		port->regshift = old_serial_port[i].iomem_reg_shift;

		port->irqflags |= irqflag;
		if (serial8250_isa_config != NULL)
			serial8250_isa_config(i, &up->port, &up->capabilities);
	}
}
*/

/*
 * Londelec modified function based on serial8250_isa_init_ports()
 * Instead of copying code of univ8250_port_ops serial8250_timeout univ8250_driver_ops
 * from 8250_core.c we just assume the first port (ttyS0) will always be 8250 on x86 platforms
 * and we copy required functions from it. This is cheeky, but allows to avoid copying code
 * and relying on standard 8250 implementation.
 */
static void serial8250_register_ports(struct uart_driver *drv)
{
	int i;
	struct uart_8250_port *firstp;
	void *serial8250_timeout;

	firstp = serial8250_get_port(0);	// First initialized 8250 port (e.g. ttyS0) to copy functions from

	if (nr_uarts > UART_NR)
		nr_uarts = UART_NR;

	for (i = 0; i < nr_uarts; i++) {
		struct uart_8250_port *up = &serial8250_ports[i];
		up->port.line = i;
		serial8250_init_port(up);
		up->port.ops = firstp->port.ops;				// Copy standard function

		serial8250_timeout = firstp->timer.function;			// Copy timer function
		timer_setup(&up->timer, serial8250_timeout, 0);

		up->ops = firstp->ops;							// Copy IRQ functions

		//adv_uart_add_one_port(drv, &up->port);
	}
}


#define SERIAL8250_CONSOLE	NULL


static struct uart_driver serial8250_reg = {
	.owner			= THIS_MODULE,
	.driver_name		= "adv950",
	.dev_name		= "ttyAP",
	.major			= ADV_TTY_MAJOR,
	.minor			= 64,
	.cons			= SERIAL8250_CONSOLE,
};

/*
 * This "device" covers _all_ ISA 8250-compatible serial devices listed
 * in the table in include/asm/serial.h
 */
//static struct platform_device *serial8250_isa_devs;

/*
 * serial8250_register_8250_port and serial8250_unregister_port allows for
 * 16x50 serial ports to be configured at run-time, to support PCMCIA
 * modems and PCI multiport cards.
 */
static DEFINE_MUTEX(serial_mutex);

static struct uart_8250_port *serial8250_find_match_or_unused(struct uart_port *port)
{
	int i;

	/*
	 * First, find a port entry which matches.
	 */
	for (i = 0; i < nr_uarts; i++)
		if (uart_match_port(&serial8250_ports[i].port, port))
			return &serial8250_ports[i];

	/* try line number first if still available */
	i = port->line;
	if (i < nr_uarts && serial8250_ports[i].port.type == PORT_UNKNOWN &&
			serial8250_ports[i].port.iobase == 0)
		return &serial8250_ports[i];
	/*
	 * We didn't find a matching entry, so look for the first
	 * free entry.  We look for one which hasn't been previously
	 * used (indicated by zero iobase).
	 */
	for (i = 0; i < nr_uarts; i++)
		if (serial8250_ports[i].port.type == PORT_UNKNOWN &&
		    serial8250_ports[i].port.iobase == 0)
			return &serial8250_ports[i];

	/*
	 * That also failed.  Last resort is to find any entry which
	 * doesn't have a real port associated with it.
	 */
	for (i = 0; i < nr_uarts; i++)
		if (serial8250_ports[i].port.type == PORT_UNKNOWN)
			return &serial8250_ports[i];

	return NULL;
}

/**
 *	serial8250_register_8250_port - register a serial port
 *	@up: serial port template
 *
 *	Configure the serial port specified by the request. If the
 *	port exists and is in use, it is hung up and unregistered
 *	first.
 *
 *	The port is then probed and if necessary the IRQ is autodetected
 *	If this fails an error is returned.
 *
 *	On success the port is ready to use and the line number is returned.
 */
int adv_serial8250_register_port(struct uart_8250_port *up)
{
	struct uart_8250_port *uart;
	int ret = -ENOSPC;

	if (up->port.uartclk == 0)
		return -EINVAL;

	mutex_lock(&serial_mutex);

	uart = serial8250_find_match_or_unused(&up->port);
	if (uart && uart->port.type != PORT_8250_CIR) {
		if (uart->port.dev)
			uart_remove_one_port(&serial8250_reg, &uart->port);

		uart->port.iobase       = up->port.iobase;
		uart->port.membase      = up->port.membase;
		uart->port.irq          = up->port.irq;
		uart->port.irqflags     = up->port.irqflags;
		uart->port.uartclk      = up->port.uartclk;
		uart->port.fifosize     = up->port.fifosize;
		uart->port.regshift     = up->port.regshift;
		uart->port.iotype       = up->port.iotype;
		uart->port.flags        = up->port.flags | UPF_BOOT_AUTOCONF;
		uart->bugs		= up->bugs;
		uart->port.mapbase      = up->port.mapbase;
		uart->port.mapsize      = up->port.mapsize;
		uart->port.private_data = up->port.private_data;
		uart->tx_loadsz		= up->tx_loadsz;
		uart->capabilities	= up->capabilities;
		uart->port.throttle	= up->port.throttle;
		uart->port.unthrottle	= up->port.unthrottle;
		uart->port.rs485_config	= up->port.rs485_config;
		uart->port.rs485_supported = up->port.rs485_supported;
		uart->port.rs485	= up->port.rs485;
		uart->lsr_save_mask	= up->lsr_save_mask;
		uart->dma		= up->dma;
    	//uart->port.unused[0]    = up->port.unused[0];
    	//uart->port.unused[1]    = up->port.unused[1];


		/* Take tx_loadsz from fifosize if it wasn't set separately */
		if (uart->port.fifosize && !uart->tx_loadsz)
			uart->tx_loadsz = uart->port.fifosize;

		if (up->port.dev)
			uart->port.dev = up->port.dev;

		if (skip_txen_test)
			uart->port.flags |= UPQ_NO_TXEN_TEST;

		if (up->port.flags & UPF_FIXED_TYPE)
			uart->port.type = up->port.type;

		serial8250_set_defaults(uart);

		/* Possibly override default I/O functions.  */
		if (up->port.serial_in)
			uart->port.serial_in = up->port.serial_in;
		if (up->port.serial_out)
			uart->port.serial_out = up->port.serial_out;
		if (up->port.handle_irq)
			uart->port.handle_irq = up->port.handle_irq;
		/*  Possibly override set_termios call */
		if (up->port.set_termios)
			uart->port.set_termios = up->port.set_termios;
		if (up->port.set_ldisc)
			uart->port.set_ldisc = up->port.set_ldisc;
		if (up->port.get_mctrl)
			uart->port.get_mctrl = up->port.get_mctrl;
		if (up->port.set_mctrl)
			uart->port.set_mctrl = up->port.set_mctrl;
		if (up->port.get_divisor)
			uart->port.get_divisor = up->port.get_divisor;
		if (up->port.set_divisor)
			uart->port.set_divisor = up->port.set_divisor;
		if (up->port.startup)
			uart->port.startup = up->port.startup;
		if (up->port.shutdown)
			uart->port.shutdown = up->port.shutdown;
		if (up->port.pm)
			uart->port.pm = up->port.pm;
		if (up->port.handle_break)
			uart->port.handle_break = up->port.handle_break;
		if (up->dl_read)
			uart->dl_read = up->dl_read;
		if (up->dl_write)
			uart->dl_write = up->dl_write;
		if (up->mcr)			// DMA requires forcing UART_MCR_CLKSEL bit
			uart->mcr |= up->mcr;

		if (uart->port.type != PORT_8250_CIR) {
			if (serial8250_isa_config != NULL)
				serial8250_isa_config(0, &uart->port,
						&uart->capabilities);

			ret = uart_add_one_port(&serial8250_reg,
						&uart->port);
			if (ret == 0)
				ret = uart->port.line;
		} else {
			dev_info(uart->port.dev,
				"skipping CIR port at 0x%lx / 0x%llx, IRQ %d\n",
				uart->port.iobase,
				(unsigned long long)uart->port.mapbase,
				uart->port.irq);

			ret = 0;
		}
	}
	mutex_unlock(&serial_mutex);

	return ret;
}
//EXPORT_SYMBOL(serial8250_register_8250_port);

/**
 *	serial8250_unregister_port - remove a 16x50 serial port at runtime
 *	@line: serial line number
 *
 *	Remove one serial port.  This may not be called from interrupt
 *	context.  We hand the port back to the our control.
 */
void adv_serial8250_unregister_port(int line)
{
	struct uart_8250_port *uart = &serial8250_ports[line];

	mutex_lock(&serial_mutex);
	uart_remove_one_port(&serial8250_reg, &uart->port);
	/*if (serial8250_isa_devs) {
		uart->port.flags &= ~UPF_BOOT_AUTOCONF;
		if (skip_txen_test)
			uart->port.flags |= UPF_NO_TXEN_TEST;
		uart->port.type = PORT_UNKNOWN;
		uart->port.dev = &serial8250_isa_devs->dev;
		uart->capabilities = 0;
		uart_add_one_port(&serial8250_reg, &uart->port);
	} else {*/
		uart->port.dev = NULL;
	//}
	mutex_unlock(&serial_mutex);
}
//EXPORT_SYMBOL(serial8250_unregister_port);

//static int __init serial8250_init(void)
int adv_serial8250_init(void)
{
	int ret;

	if (nr_uarts == 0)
		return -ENODEV;

	//serial8250_isa_init_ports();

	printk(KERN_INFO "Serial: Advantech PCI/PCIE ICOM driver, "
		"Max support %d ports, IRQ sharing %sabled\n", nr_uarts,
		share_irqs ? "en" : "dis");

	serial8250_reg.nr = UART_NR;
	ret = uart_register_driver(&serial8250_reg);

	if (ret)
		goto out;

	serial8250_register_ports(&serial8250_reg);

out:
	return ret;
}

//static void __exit serial8250_exit(void)
void adv_serial8250_exit(void)
{
	//struct platform_device *isa_dev = serial8250_isa_devs;

	/*
	 * This tells serial8250_unregister_port() not to re-register
	 * the ports (thereby making serial8250_isa_driver permanently
	 * in use.)
	 */
	//serial8250_isa_devs = NULL;

	//platform_driver_unregister(&serial8250_isa_driver);
	//platform_device_unregister(isa_dev);

	//serial8250_pnp_exit();

#ifdef CONFIG_SPARC
	sunserial_unregister_minors(&serial8250_reg, UART_NR);
#else
	uart_unregister_driver(&serial8250_reg);
#endif
}

//module_init(serial8250_init);
//module_exit(serial8250_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Advantech IAG PCI-954/16C950 serial driver");

module_param(share_irqs, uint, 0644);
MODULE_PARM_DESC(share_irqs, "Share IRQs with other non-8250/16x50 devices"
	" (unsafe)");

module_param(nr_uarts, uint, 0644);
MODULE_PARM_DESC(nr_uarts, "Maximum number of UARTs supported. (1-" __MODULE_STRING(CONFIG_SERIAL_8250_NR_UARTS) ")");

module_param(skip_txen_test, uint, 0644);
MODULE_PARM_DESC(skip_txen_test, "Skip checking for the TXEN bug at init time");

#ifdef CONFIG_SERIAL_8250_RSA
module_param_array(probe_rsa, ulong, &probe_rsa_count, 0444);
MODULE_PARM_DESC(probe_rsa, "Probe I/O ports for RSA");
#endif
MODULE_ALIAS_CHARDEV_MAJOR(TTY_MAJOR);
