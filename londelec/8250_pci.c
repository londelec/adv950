//******************************************************************************
//
// Copyright (c) 2017 Advantech Industrial Automation Group.
//
// Oxford PCI-954/952/16C950 with Advantech RS232/422/485 capacities
//
// This program is free software; you can redistribute it and/or modify it
// under the terms of the GNU General Public License as published by the Free
// Software Foundation; either version 2 of the License, or (at your option)
// any later version.
//
// This program is distributed in the hope that it will be useful, but WITHOUT
// ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
// FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
// more details.
//
// You should have received a copy of the GNU General Public License along with
// this program; if not, write to the Free Software Foundation, Inc., 59
// Temple Place - Suite 330, Boston, MA  02111-1307, USA.
//
//
//
//******************************************************************************

//***********************************************************************
// File:      8250_pci.c
// Author:    Jianfeng Dai
//
//***********************************************************************
#include <linux/module.h>
#include <linux/init.h>
#include <linux/pci.h>
#include <linux/string.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/tty.h>
#include <linux/serial_core.h>
#include <linux/8250_pci.h>
#include <linux/bitops.h>

#include <asm/byteorder.h>
#include <asm/io.h>

#include "8250.h"

#undef SERIAL_DEBUG_PCI
/*
 * Advantech IAG PCI-954/16C950 cards
 *
 */
#define ADVANTECH_16C950_VER                    "6.0.6"
#define ADVANTECH_16C950_DATE                   "16/04/2023"
#define PCI_VENDOR_ID_ADVANTECH                 0x13fe
#define PCI_DEVICE_ID_ADVANTECH_PCI1600         0x1600 /* Internal */
#define PCI_DEVICE_ID_ADVANTECH_PCI1601         0x1601 /* Internal */
#define PCI_DEVICE_ID_ADVANTECH_PCI1602         0x1602 /* Internal */
#define PCI_DEVICE_ID_ADVANTECH_PCI1603         0x1603 /* Internal */
#define PCI_DEVICE_ID_ADVANTECH_PCI1604         0x1604 /* Internal */
#define PCI_DEVICE_ID_ADVANTECH_PCI16ff         0x16ff /* External */
#define PCI_DEVICE_ID_ADVANTECH_PCI1600_1601    0x1601
#define PCI_DEVICE_ID_ADVANTECH_PCI1600_1602    0x1602
#define PCI_DEVICE_ID_ADVANTECH_PCI1600_1610    0x1610
#define PCI_DEVICE_ID_ADVANTECH_PCI1600_1611    0x1611
#define PCI_DEVICE_ID_ADVANTECH_PCI1600_1612    0x1612 /* Also for UNO-2059 */
#define PCI_DEVICE_ID_ADVANTECH_PCI1600_1620    0x1620
#define PCI_DEVICE_ID_ADVANTECH_PCI1600_1622    0x1622
#define PCI_DEVICE_ID_ADVANTECH_UNO2050         0x2050
#define PCI_DEVICE_ID_ADVANTECH_UNOB2201        0x2201 //2668
#define PCI_DEVICE_ID_ADVANTECH_UNOBF201        0xf201 //2668
#define PCI_DEVICE_ID_ADVANTECH_UNOBX201_2201   0x2201 //2668
#define PCI_DEVICE_ID_ADVANTECH_MIC3620         0x3620
#define PCI_DEVICE_ID_ADVANTECH_MIC3612         0X3612
#define PCI_DEVICE_ID_ADVANTECH_MIC3611         0X3611
#define PCI_DEVICE_ID_ADVANTECH_UNO2176         0x2176
#define PCI_DEVICE_ID_ADVANTECH_UNO2X76_2176    0x2176
#define PCI_DEVICE_ID_ADVANTECH_PCIE952		0xA202
#define PCI_DEVICE_ID_ADVANTECH_PCIE954		0xA304
#define PCI_DEVICE_ID_ADVANTECH_PCIE958		0xA408
#define PCI_DEVICE_ID_ADVANTECH_PCM3614P        0x3614 //PCM-3614P
#define PCI_DEVICE_ID_ADVANTECH_PCM3641P        0x3641 //PCM-3641P
#define PCI_DEVICE_ID_ADVANTECH_PCM3618P        0x3618 //PCM-3618P
#define PCI_DEVICE_ID_ADVANTECH_PCMF618P        0xF618 //PCM-3618P
#define PCI_DEVICE_ID_ADVANTECH_PCM3681P        0x3681 //PCM-3681P
#define PCI_DEVICE_ID_ADVANTECH_PCMF681P        0xF681 //PCM-3681P
#define PCI_SUB_VENDOR_ID_ADVANTECH_PCM3614P    0x3614 //PCM-3614P
#define PCI_SUB_VENDOR_ID_ADVANTECH_PCM3618P    0x3618 //PCM-3618P
#define PCI_SUB_VENDOR_ID_ADVANTECH_PCM3641P    0x3641 //PCM-3641P
#define PCI_SUB_VENDOR_ID_ADVANTECH_PCM3681P    0x3681 //PCM-3681P
#define PCI_DEVICE_ID_ADVANTECH_UNO1150         0x3610 //UNO-1150
#define PCI_DEVICE_ID_ADVANTECH_MIC3621         0x3621 //PCM-3614P
#define PCI_SUB_VENDOR_ID_ADVANTECH_MIC3621     0x3621 //PCM-3614P
#define PCI_DEVICE_ID_ADVANTECH_A001            0xA001
#define PCI_DEVICE_ID_ADVANTECH_A002            0xA002
#define PCI_DEVICE_ID_ADVANTECH_A004            0xA004
#define PCI_DEVICE_ID_ADVANTECH_A101            0xA101
#define PCI_DEVICE_ID_ADVANTECH_A102            0xA102
#define PCI_DEVICE_ID_ADVANTECH_A104            0xA104
#define PCI_DEVICE_ID_ADVANTECH_F001            0xF001
#define PCI_DEVICE_ID_ADVANTECH_F002            0xF002
#define PCI_DEVICE_ID_ADVANTECH_F004            0xF004
#define PCI_DEVICE_ID_ADVANTECH_F101            0xF101
#define PCI_DEVICE_ID_ADVANTECH_F102            0xF102
#define PCI_DEVICE_ID_ADVANTECH_F104            0xF104

#define PCI_DEVICE_ID_ADVANTECH_A821            0xA821
#define PCI_DEVICE_ID_ADVANTECH_A822            0xA822
#define PCI_DEVICE_ID_ADVANTECH_A823            0xA823
#define PCI_DEVICE_ID_ADVANTECH_A824            0xA824
#define PCI_DEVICE_ID_ADVANTECH_A828            0xA828
#define PCI_DEVICE_ID_ADVANTECH_A831            0xA831
#define PCI_DEVICE_ID_ADVANTECH_A832            0xA832
#define PCI_DEVICE_ID_ADVANTECH_A833            0xA833
#define PCI_DEVICE_ID_ADVANTECH_A834            0xA834
#define PCI_DEVICE_ID_ADVANTECH_A838            0xA838

#define PCI_DEVICE_ID_ADVANTECH_A516            0xA516
#define PCI_DEVICE_ID_ADVANTECH_F500            0xF500







#define UART_TYPE_AUTO				0
#define UART_TYPE_RS232				1
#define UART_TYPE_RS485				2
static char * product_line[] = {"GENERAL","PCI","PCM","ADAM","APAX","BAS","UNO","TPC","EAMB"};
/*
 * init function returns:
 *  > 0 - number of ports
 *  = 0 - use board->num_ports
 *  < 0 - error
 */
struct pci_serial_quirk {
	u32	vendor;
	u32	device;
	u32	subvendor;
	u32	subdevice;
	int	(*probe)(struct pci_dev *dev);
	int	(*init)(struct pci_dev *dev);
	int	(*setup)(struct serial_private *,
			 const struct pciserial_board *,
			 struct uart_8250_port *, int);
	void	(*exit)(struct pci_dev *dev);
};

#define PCI_NUM_BAR_RESOURCES	6

struct serial_private {
	struct pci_dev		*dev;
	unsigned int		nr;
	void __iomem		*remapped_bar[PCI_NUM_BAR_RESOURCES];
	struct pci_serial_quirk	*quirk;
	const struct pciserial_board *board;
	int			line[0];
};

static void moan_device(const char *str, struct pci_dev *dev)
{
	dev_err(&dev->dev,
	       "%s: %s\n"
	       "Please send the output of lspci -vv, this\n"
	       "message (0x%04x,0x%04x,0x%04x,0x%04x), the\n"
	       "manufacturer and name of serial board or\n"
	       "modem board to <linux-serial@vger.kernel.org>.\n",
	       pci_name(dev), str, dev->vendor, dev->device,
	       dev->subsystem_vendor, dev->subsystem_device);
}

/* Dummy config function required for 6.0.6-londelec kernel */
static int adv_rs485_config(struct uart_port *port, struct ktermios *termios,
			       struct serial_rs485 *rs485)
{
	//printk("LEDEBUG: %s() rs485flags=0x%x, supported=0x%x\n", __func__,  port->rs485.flags, port->rs485_supported.flags);
	return 0;
}

/* Supported flags required for 6.0.6-londelec kernel */
static const struct serial_rs485 adv_rs485_supported = {
	.flags = SER_RS485_ENABLED | SER_RS485_RTS_ON_SEND | SER_RS485_RTS_AFTER_SEND | LE485FLAG_DMA,
};

static int
setup_port(struct serial_private *priv, struct uart_8250_port *port,
	   int bar, int offset, int regshift)
{
	struct pci_dev *dev = priv->dev;

	if (bar >= PCI_NUM_BAR_RESOURCES)
		return -EINVAL;

	if (pci_resource_flags(dev, bar) & IORESOURCE_MEM) {
		if (!priv->remapped_bar[bar])
			priv->remapped_bar[bar] = pci_ioremap_bar(dev, bar);
		if (!priv->remapped_bar[bar])
			return -ENOMEM;

		port->port.iotype = UPIO_MEM;
		port->port.iobase = 0;
		port->port.mapbase = pci_resource_start(dev, bar) + offset;
		port->port.membase = priv->remapped_bar[bar] + offset;
		port->port.regshift = regshift;
	} else {
		port->port.iotype = UPIO_PORT;
		port->port.iobase = pci_resource_start(dev, bar) + offset;
		port->port.mapbase = 0;
		port->port.membase = NULL;
		port->port.regshift = 0;
	}
	return 0;
}


static int pci_default_setup(struct serial_private *priv,
		  const struct pciserial_board *board,
		  struct uart_8250_port *port, int idx)
{
	unsigned int bar, offset = board->first_offset, maxnr;

	bar = FL_GET_BASE(board->flags);
	if (board->flags & FL_BASE_BARS)
		bar += idx;
	else
		offset += idx * board->uart_offset;

	maxnr = (pci_resource_len(priv->dev, bar) - board->first_offset) >>
		(board->reg_shift + 3);

	if (board->flags & FL_REGION_SZ_CAP && idx >= maxnr)
		return 1;

	return setup_port(priv, port, bar, offset, board->reg_shift);
}
/*
 * Advantech IAG PCI-954/16C950 cards
 */
//static int
//pci_advantech_setup(struct pci_dev *dev, struct pci_board *board,
//		  struct serial_struct *req, int idx)
// FIXME port->port.unused[1] = 0x20 needs to be sorted out for Exar chip to work, it is not working since the driver was upgarded for 4.4.0-116-londelec kernel
static int
pci_advantech_setup (struct serial_private *priv,
		  const struct pciserial_board *board,
		  struct uart_8250_port *port, int idx)
{
	u32 bar, port485, offset485;
	u32 len;
	void __iomem * remap;
	u8 config485, activeType, configFunc, configType;
	u16  config485_958;
	struct pci_dev *cfgdev = NULL;
	int base_idx=0;
	int rc;
	u8 isPCIe95x = 0;

	struct pci_dev *dev = NULL;

	port->port.rs485.flags = 0;
	configFunc = 1; // Default configuration BAR is function 1
	offset485 = 0x60; // Default offset to get RS232/422/485 configuration
	bar = PCI_BASE_ADDRESS_0; // Default BAR is PCI_BASE_ADDRESS_0
	activeType = SER_RS485_RTS_ON_SEND; // Default RS485 is active high
	configType = UART_TYPE_AUTO; // Default UART type is auto detection
	dev = priv->dev;
	switch(dev->subsystem_vendor)
	{
	case PCI_DEVICE_ID_ADVANTECH_PCI1600_1601:
		printk("PCI-1601");
		break;
	case PCI_DEVICE_ID_ADVANTECH_PCI1600_1602:
		printk("PCI-1602");
		break;
	case PCI_DEVICE_ID_ADVANTECH_PCI1600_1610:
		printk("PCI-1610");
		break;
	case PCI_DEVICE_ID_ADVANTECH_PCI1600_1611:
		printk("PCI-1611");
		configType = UART_TYPE_RS485;
		break;
	case PCI_DEVICE_ID_ADVANTECH_PCI1600_1612:	/* Also for UNO-2059 */
		printk("PCI-1612 / UNO-2059");
		break;
	case PCI_DEVICE_ID_ADVANTECH_PCI1600_1620:
		printk("PCI-1620");
		break;
	case PCI_DEVICE_ID_ADVANTECH_PCI1600_1622:
		printk("PCI-1622CU");
		break;
	case PCI_DEVICE_ID_ADVANTECH_UNO2050:
		printk("UNO-2050");
		offset485 = 0x18;
		break;
	case PCI_DEVICE_ID_ADVANTECH_UNOBX201_2201:
		printk("UNOB-2201CB");
		activeType = SER_RS485_RTS_AFTER_SEND;
		break;
	case PCI_DEVICE_ID_ADVANTECH_UNO2X76_2176:
		printk( "UNO-2176" );
		activeType = SER_RS485_RTS_AFTER_SEND;
		break;
	case PCI_DEVICE_ID_ADVANTECH_MIC3612:
		printk("MIC-3612");
		configFunc = 0;
		bar = PCI_BASE_ADDRESS_2;
		break;
	case PCI_DEVICE_ID_ADVANTECH_MIC3621:
		printk("MIC-3621");
		configFunc = 0;
		bar = PCI_BASE_ADDRESS_2;
		break;
	case PCI_DEVICE_ID_ADVANTECH_MIC3620:
		printk("MIC-3620");
		configType = UART_TYPE_RS232;
		break;
	case PCI_DEVICE_ID_ADVANTECH_MIC3611:
		printk("MIC-3611");
		configFunc = 0;
		bar = PCI_BASE_ADDRESS_2;
		configType = UART_TYPE_RS485;
		break;
	case PCI_SUB_VENDOR_ID_ADVANTECH_PCM3614P:
		printk( "PCM-3614P" );
		activeType = SER_RS485_RTS_AFTER_SEND;
		break;
	case PCI_SUB_VENDOR_ID_ADVANTECH_PCM3618P:
		printk( "PCM-3618P" );
		activeType = SER_RS485_RTS_AFTER_SEND;
		break;
	case PCI_SUB_VENDOR_ID_ADVANTECH_PCM3641P:
		printk( "PCM-3641P" );
		configType = UART_TYPE_RS232;
		break;
	case PCI_SUB_VENDOR_ID_ADVANTECH_PCM3681P:
		printk( "PCM-3681P" );
		configType = UART_TYPE_RS232;
		break;
	case PCI_DEVICE_ID_ADVANTECH_UNO1150:
		printk( "UNO-1150" );
		configFunc = 0;
		base_idx = 2;
		bar = PCI_BASE_ADDRESS_2;
		offset485 = 0x10;
		activeType = SER_RS485_RTS_AFTER_SEND;
		break;
	case PCI_VENDOR_ID_ADVANTECH:
		switch(dev->device)
		{
		case PCI_DEVICE_ID_ADVANTECH_PCIE952:
			printk( "PCIE952");
			port->dl_write = adv_default_serial_dl_write;//this bit means to use new way to calculate baudrate
			port->mcr |= UART_MCR_CLKSEL;	// This comes from Advantech's 8250.c serial8250_set_mctrl()
			port->port.rs485.flags |= LE485FLAG_DMA; //have DMA
			isPCIe95x = 1; //is PCIe952/4/8
			configFunc = 0;

			base_idx = 13;//
			bar = PCI_BASE_ADDRESS_0;//ok
			offset485 = 0x100;//
			activeType = SER_RS485_RTS_AFTER_SEND;
			break;
		case PCI_DEVICE_ID_ADVANTECH_PCIE954:
			printk( "PCIE954");
			port->dl_write = adv_default_serial_dl_write;
			port->mcr |= UART_MCR_CLKSEL;
			port->port.rs485.flags |= LE485FLAG_DMA;
			isPCIe95x = 1;
			configFunc = 0;

			base_idx = 13;//
			bar = PCI_BASE_ADDRESS_0;//ok
			offset485 = 0x100;//
			activeType = SER_RS485_RTS_AFTER_SEND;

			//port->port.startup = serial8250_do_startup;
			break;
		case PCI_DEVICE_ID_ADVANTECH_PCIE958:
			printk( "PCIE958-DMA");
			port->dl_write = adv_default_serial_dl_write;
			port->mcr |= UART_MCR_CLKSEL;
			port->port.rs485.flags |= LE485FLAG_DMA;
			isPCIe95x = 1;
			configFunc = 0;

			base_idx = 13;//
			bar = PCI_BASE_ADDRESS_0;//ok
			offset485 = 0x100;//
			activeType = SER_RS485_RTS_AFTER_SEND;
			break;
		case PCI_DEVICE_ID_ADVANTECH_A516:
			printk( "PCIE958-DMA");
			port->dl_write = adv_default_serial_dl_write;
			port->mcr |= UART_MCR_CLKSEL;
			port->port.rs485.flags |= LE485FLAG_DMA;
			isPCIe95x = 1;
			configFunc = 0;

			base_idx = 13;//
			bar = PCI_BASE_ADDRESS_0;//ok
			offset485 = 0x100;//
			activeType = SER_RS485_RTS_AFTER_SEND;
			break;
		case PCI_DEVICE_ID_ADVANTECH_PCI1601:
			printk("PCI-1601A/B/AU/BU");
			configType = UART_TYPE_RS485;
			break;
		case PCI_DEVICE_ID_ADVANTECH_PCI1602:
			printk("PCI-1602A/B/AU/BU/UP");
			configType = UART_TYPE_RS485;
			break;
		case PCI_DEVICE_ID_ADVANTECH_PCI1603:
			printk("PCI-1603");
			configType = UART_TYPE_RS232;
			break;
		case PCI_DEVICE_ID_ADVANTECH_PCI1604:
			printk("PCI-1604UP");
			configType = UART_TYPE_RS232;
			break;

		}
		break;
	}
	if (dev->device == PCI_DEVICE_ID_ADVANTECH_A001
	|| dev->device == PCI_DEVICE_ID_ADVANTECH_A002
	|| dev->device == PCI_DEVICE_ID_ADVANTECH_A004
	|| dev->device == PCI_DEVICE_ID_ADVANTECH_A101
	|| dev->device == PCI_DEVICE_ID_ADVANTECH_A102
	|| dev->device == PCI_DEVICE_ID_ADVANTECH_A104
	|| dev->device == PCI_DEVICE_ID_ADVANTECH_F001
	|| dev->device == PCI_DEVICE_ID_ADVANTECH_F002
	|| dev->device == PCI_DEVICE_ID_ADVANTECH_F004
	|| dev->device == PCI_DEVICE_ID_ADVANTECH_F101
	|| dev->device == PCI_DEVICE_ID_ADVANTECH_F102
	|| dev->device == PCI_DEVICE_ID_ADVANTECH_F104)
	{
		activeType = SER_RS485_RTS_AFTER_SEND;
		if (dev->subsystem_vendor != PCI_VENDOR_ID_ADVANTECH)
		{
			printk("%s-%04x",product_line[dev->subsystem_vendor],dev->subsystem_device);
		}
		else
		{
			printk("Advantech General COM Port Device");
		}

	}
	if (dev->device == PCI_DEVICE_ID_ADVANTECH_A821
	|| dev->device == PCI_DEVICE_ID_ADVANTECH_A822
	|| dev->device == PCI_DEVICE_ID_ADVANTECH_A823
	|| dev->device == PCI_DEVICE_ID_ADVANTECH_A824
	|| dev->device == PCI_DEVICE_ID_ADVANTECH_A828
	|| dev->device == PCI_DEVICE_ID_ADVANTECH_A831
	|| dev->device == PCI_DEVICE_ID_ADVANTECH_A832
	|| dev->device == PCI_DEVICE_ID_ADVANTECH_A833
	|| dev->device == PCI_DEVICE_ID_ADVANTECH_A834
	|| dev->device == PCI_DEVICE_ID_ADVANTECH_A838)
	{
		activeType = SER_RS485_RTS_AFTER_SEND;
		// FIXME  port->port.unused[1] |= 0x20; //Is XR chip
		printk("Advantech General COM Port Device");

		//UART_TYPE_AUTO: detect 232 or 422/485
		pci_read_config_dword(dev, PCI_BASE_ADDRESS_0, &port485);
		len =  pci_resource_len(dev, 0);
		remap = ioremap(port485, len);
		config485 = readw(remap + 0x90);
		//printk("port485 = 0x%x, len = 0x%x, config485=0x%x############\n",port485,len,config485);
		port->port.rs485.flags |= (config485 & (0x01 << idx)) ? (activeType | SER_RS485_ENABLED) : 0;

		iounmap(remap);

		goto done;
	}
	if(configType == UART_TYPE_RS232)
	{
		//port->port.rs485.flags = 0;
	}
	else if(configType == UART_TYPE_RS485)
	{
		port->port.rs485.flags |= activeType | SER_RS485_ENABLED;
	}
	else // UART_TYPE_AUTO
	{
		// find RS232/422/485 configuration BAR
		do {

			cfgdev = pci_get_device(PCI_VENDOR_ID_ADVANTECH,
				         PCI_ANY_ID, cfgdev);


			if ((dev->bus->number == cfgdev->bus->number) &&
		    	    (PCI_SLOT(dev->devfn) == PCI_SLOT(cfgdev->devfn)) &&
			    (PCI_FUNC(cfgdev->devfn) == configFunc))
			{
				pci_read_config_dword(cfgdev, bar, &port485);
				if((port485 & PCI_BASE_ADDRESS_SPACE) ==
					PCI_BASE_ADDRESS_SPACE_IO)
				{
					rc = pci_enable_device(cfgdev);
					if (rc)
						return rc;
					port485 &= PCI_BASE_ADDRESS_IO_MASK;
					break;
				}
				break;
			}
		} while(cfgdev != NULL);

		// if cannot get RS232/422/485 configuration port
		if(!cfgdev)
		{
			printk("%x: cannot get RS232/422/485 configuration!\n",
				dev->subsystem_vendor);
			return -ENODEV;
		}
		if (isPCIe95x) {
			len =  pci_resource_len(cfgdev, ((bar-0x10)/0x04));
			//printk("port485=0x%x,len=0x%x\n",port485, len);
			if (pci_resource_flags(cfgdev, ((bar-0x10)/0x04)) & IORESOURCE_MEM) {
				remap = ioremap(port485, len);
				if (idx < 8)
				{
				   config485_958 = readw(remap + offset485 + idx*0x10);
				}
				else if (idx >= 8)
				{
				   //A516
				   config485_958 = readw(remap + offset485 + 0x100 + (idx - 8)*0x10);
				}

				//printk(KERN_INFO "configure register = %x\n", config485_958);
				port->port.rs485.flags |= (config485_958 & (0x01 << base_idx)) ?
					 (activeType | SER_RS485_ENABLED) : 0;
				goto done;
			}
			else{
				config485_958 = inw(port485 + offset485 + idx*0x10);
				port->port.rs485.flags |= (config485_958 & (0x01 << base_idx)) ?
					 (activeType | SER_RS485_ENABLED) : 0;
				goto done;
			}
		}
		// read RS232/422/485 configuration value
		config485 = inb(port485 + offset485);
		if(PCI_FUNC(dev->devfn) == 1) base_idx=4;
		port->port.rs485.flags |= (config485 & (0x01 << (base_idx+idx))) ?
					 (activeType | SER_RS485_ENABLED): 0;
	}
done:
	port->port.startup = adv_serial8250_do_startup;
	port->port.shutdown = adv_serial8250_do_shutdown;
	port->port.rs485_config = adv_rs485_config;
	port->port.rs485_supported = adv_rs485_supported;

	printk(", function %d, port %d, %s",
		PCI_FUNC(dev->devfn), idx,
		(port->port.rs485.flags & SER_RS485_ENABLED) ?
		"RS422/485" : "RS232");
	if(port->port.rs485.flags & SER_RS485_ENABLED)
		printk(", %s\n", (activeType & SER_RS485_RTS_ON_SEND) ?
			"Active High" : "Active Low");
	else
		printk("\n");
	return pci_default_setup(priv, board, port, idx);
}

/* This should be in linux/pci_ids.h */
#define PCI_VENDOR_ID_SBSMODULARIO	0x124B
#define PCI_SUBVENDOR_ID_SBSMODULARIO	0x124B
#define PCI_DEVICE_ID_OCTPRO		0x0001
#define PCI_SUBDEVICE_ID_OCTPRO232	0x0108
#define PCI_SUBDEVICE_ID_OCTPRO422	0x0208
#define PCI_SUBDEVICE_ID_POCTAL232	0x0308
#define PCI_SUBDEVICE_ID_POCTAL422	0x0408
#define PCI_VENDOR_ID_ADVANTECH		0x13fe
#define PCI_DEVICE_ID_ADVANTECH_PCI3620	0x3620

/* Unknown vendors/cards - this should not be in linux/pci_ids.h */
#define PCI_SUBDEVICE_ID_UNKNOWN_0x1584	0x1584

/*
 * Master list of serial port init/setup/exit quirks.
 * This does not describe the general nature of the port.
 * (ie, baud base, number and location of ports, etc)
 *
 * This list is ordered alphabetically by vendor then device.
 * Specific entries must come before more generic entries.
 */
static struct pci_serial_quirk pci_serial_quirks[] __refdata = {
	/*
	 * Advantech IAG PCI-954/16C950 cards
	 */
	{
		.vendor		= PCI_VENDOR_ID_ADVANTECH,
		.device		= PCI_DEVICE_ID_ADVANTECH_PCI1600,
		.subvendor	= PCI_DEVICE_ID_ADVANTECH_PCI1600_1601,
		.subdevice	= PCI_ANY_ID,
		.setup		= pci_advantech_setup,
	},
	{
		.vendor		= PCI_VENDOR_ID_ADVANTECH,
		.device		= PCI_DEVICE_ID_ADVANTECH_PCI1600,
		.subvendor	= PCI_DEVICE_ID_ADVANTECH_PCI1600_1602,
		.subdevice	= PCI_ANY_ID,
		.setup		= pci_advantech_setup,
	},
	{
		.vendor		= PCI_VENDOR_ID_ADVANTECH,
		.device		= PCI_DEVICE_ID_ADVANTECH_PCI1600,
		.subvendor	= PCI_DEVICE_ID_ADVANTECH_PCI1600_1610,
		.subdevice	= PCI_ANY_ID,
		.setup		= pci_advantech_setup,
	},
	{
		.vendor		= PCI_VENDOR_ID_ADVANTECH,
		.device		= PCI_DEVICE_ID_ADVANTECH_PCI1600,
		.subvendor	= PCI_DEVICE_ID_ADVANTECH_PCI1600_1611,
		.subdevice	= PCI_ANY_ID,
		.setup		= pci_advantech_setup,
	},
	{
		.vendor		= PCI_VENDOR_ID_ADVANTECH,
		.device		= PCI_DEVICE_ID_ADVANTECH_PCI1600,
		.subvendor	= PCI_DEVICE_ID_ADVANTECH_PCI1600_1612,
		.subdevice	= PCI_ANY_ID,
		.setup		= pci_advantech_setup,
	},
	{
		.vendor		= PCI_VENDOR_ID_ADVANTECH,
		.device		= PCI_DEVICE_ID_ADVANTECH_PCI1600,
		.subvendor	= PCI_DEVICE_ID_ADVANTECH_PCI1600_1620,
		.subdevice	= PCI_ANY_ID,
		.setup		= pci_advantech_setup,
	},
	{
		.vendor		= PCI_VENDOR_ID_ADVANTECH,
		.device		= PCI_DEVICE_ID_ADVANTECH_PCI16ff,
		.subvendor	= PCI_DEVICE_ID_ADVANTECH_PCI1600_1620,
		.subdevice	= PCI_ANY_ID,
		.setup		= pci_advantech_setup,
	},
	{
		.vendor		= PCI_VENDOR_ID_ADVANTECH,
		.device		= PCI_DEVICE_ID_ADVANTECH_PCI1600,
		.subvendor	= PCI_DEVICE_ID_ADVANTECH_PCI1600_1622,
		.subdevice	= PCI_ANY_ID,
		.setup		= pci_advantech_setup,
	},
	{
		.vendor		= PCI_VENDOR_ID_ADVANTECH,
		.device		= PCI_DEVICE_ID_ADVANTECH_PCI16ff,
		.subvendor	= PCI_DEVICE_ID_ADVANTECH_PCI1600_1622,
		.subdevice	= PCI_ANY_ID,
		.setup		= pci_advantech_setup,
	},
	{
		.vendor		= PCI_VENDOR_ID_ADVANTECH,
		.device		= PCI_DEVICE_ID_ADVANTECH_UNO2050,
		.subvendor	= PCI_DEVICE_ID_ADVANTECH_UNO2050,
		.subdevice	= PCI_ANY_ID,
		.setup		= pci_advantech_setup,
	},
	{
		.vendor		= PCI_VENDOR_ID_ADVANTECH,
		.device		= PCI_DEVICE_ID_ADVANTECH_UNOB2201,
		.subvendor	= PCI_DEVICE_ID_ADVANTECH_UNOBX201_2201,
		.subdevice	= PCI_ANY_ID,
		.setup		= pci_advantech_setup,
	},
	{
		.vendor		= PCI_VENDOR_ID_ADVANTECH,
		.device		= PCI_DEVICE_ID_ADVANTECH_UNOBF201,
		.subvendor	= PCI_DEVICE_ID_ADVANTECH_UNOBX201_2201,
		.subdevice	= PCI_ANY_ID,
		.setup		= pci_advantech_setup,
	},
	{
		.vendor		= PCI_VENDOR_ID_ADVANTECH,
		.device		= PCI_DEVICE_ID_ADVANTECH_MIC3612,
		.subvendor	= PCI_DEVICE_ID_ADVANTECH_MIC3612,
		.subdevice	= PCI_ANY_ID,
		.setup		= pci_advantech_setup,
	},
	{
		.vendor		= PCI_VENDOR_ID_ADVANTECH,
		.device		= PCI_DEVICE_ID_ADVANTECH_MIC3611,
		.subvendor	= PCI_DEVICE_ID_ADVANTECH_MIC3611,
		.subdevice	= PCI_ANY_ID,
		.setup		= pci_advantech_setup,
	},
	{
		.vendor		= PCI_VENDOR_ID_ADVANTECH,
		.device		= PCI_DEVICE_ID_ADVANTECH_MIC3620,
		.subvendor	= PCI_DEVICE_ID_ADVANTECH_MIC3620,
		.subdevice	= PCI_ANY_ID,
		.setup		= pci_advantech_setup,
	},
	{
		.vendor		= PCI_VENDOR_ID_ADVANTECH,
		.device		= PCI_DEVICE_ID_ADVANTECH_PCI1601,
		.subvendor	= PCI_VENDOR_ID_ADVANTECH,
		.subdevice	= PCI_ANY_ID,
		.setup		= pci_advantech_setup,
	},
	{
		.vendor		= PCI_VENDOR_ID_ADVANTECH,
		.device		= PCI_DEVICE_ID_ADVANTECH_PCI1602,
		.subvendor	= PCI_VENDOR_ID_ADVANTECH,
		.subdevice	= PCI_ANY_ID,
		.setup		= pci_advantech_setup,
	},
	{
		.vendor		= PCI_VENDOR_ID_ADVANTECH,
		.device		= PCI_DEVICE_ID_ADVANTECH_PCI1603,
		.subvendor	= PCI_VENDOR_ID_ADVANTECH,
		.subdevice	= PCI_ANY_ID,
		.setup		= pci_advantech_setup,
	},
	{
		.vendor		= PCI_VENDOR_ID_ADVANTECH,
		.device		= PCI_DEVICE_ID_ADVANTECH_PCI1604,
		.subvendor	= PCI_VENDOR_ID_ADVANTECH,
		.subdevice	= PCI_ANY_ID,
		.setup		= pci_advantech_setup,
	},
	{
		.vendor		= PCI_VENDOR_ID_ADVANTECH,
		.device		= PCI_DEVICE_ID_ADVANTECH_UNO2176,
		.subvendor	= PCI_DEVICE_ID_ADVANTECH_UNO2X76_2176,
		.subdevice	= PCI_ANY_ID,
		.setup		= pci_advantech_setup,
	},
	{
		.vendor		= PCI_VENDOR_ID_ADVANTECH,
		.device		= PCI_DEVICE_ID_ADVANTECH_PCIE952,
		.subvendor	= PCI_VENDOR_ID_ADVANTECH,
		.subdevice	= PCI_DEVICE_ID_ADVANTECH_PCIE952,
		.setup		= pci_advantech_setup,
	},
	{
		.vendor		= PCI_VENDOR_ID_ADVANTECH,
		.device		= PCI_DEVICE_ID_ADVANTECH_PCIE954,
		.subvendor	= PCI_VENDOR_ID_ADVANTECH,
		.subdevice	= PCI_DEVICE_ID_ADVANTECH_PCIE954,
		.setup		= pci_advantech_setup,
	},
	{
		.vendor		= PCI_VENDOR_ID_ADVANTECH,
		.device		= PCI_DEVICE_ID_ADVANTECH_PCIE958,
		.subvendor	= PCI_VENDOR_ID_ADVANTECH,
		.subdevice	= PCI_DEVICE_ID_ADVANTECH_PCIE958,
		.setup		= pci_advantech_setup,
	},
	{
		.vendor		= PCI_VENDOR_ID_ADVANTECH,
		.device		= PCI_DEVICE_ID_ADVANTECH_A516,
		.subvendor	= PCI_VENDOR_ID_ADVANTECH,
		.subdevice	= PCI_DEVICE_ID_ADVANTECH_A516,
		.setup		= pci_advantech_setup,
	},
	{
		.vendor		= PCI_VENDOR_ID_ADVANTECH,
		.device		= PCI_DEVICE_ID_ADVANTECH_PCM3614P,
		.subvendor	= PCI_SUB_VENDOR_ID_ADVANTECH_PCM3614P,
		.subdevice	= PCI_ANY_ID,
		.setup		= pci_advantech_setup,
	},
	{
		.vendor		= PCI_VENDOR_ID_ADVANTECH,
		.device		= PCI_DEVICE_ID_ADVANTECH_PCM3618P,
		.subvendor	= PCI_SUB_VENDOR_ID_ADVANTECH_PCM3618P,
		.subdevice	= PCI_ANY_ID,
		.setup		= pci_advantech_setup,
	},
	{
		.vendor		= PCI_VENDOR_ID_ADVANTECH,
		.device		= PCI_DEVICE_ID_ADVANTECH_PCMF618P,
		.subvendor	= PCI_SUB_VENDOR_ID_ADVANTECH_PCM3618P,
		.subdevice	= PCI_ANY_ID,
		.setup		= pci_advantech_setup,
	},
	{
		.vendor		= PCI_VENDOR_ID_ADVANTECH,
		.device		= PCI_DEVICE_ID_ADVANTECH_PCM3641P,
		.subvendor	= PCI_SUB_VENDOR_ID_ADVANTECH_PCM3641P,
		.subdevice	= PCI_ANY_ID,
		.setup		= pci_advantech_setup,
	},
	{
		.vendor		= PCI_VENDOR_ID_ADVANTECH,
		.device		= PCI_DEVICE_ID_ADVANTECH_PCM3681P,
		.subvendor	= PCI_SUB_VENDOR_ID_ADVANTECH_PCM3681P,
		.subdevice	= PCI_ANY_ID,
		.setup		= pci_advantech_setup,
	},
	{
		.vendor		= PCI_VENDOR_ID_ADVANTECH,
		.device		= PCI_DEVICE_ID_ADVANTECH_PCMF681P,
		.subvendor	= PCI_SUB_VENDOR_ID_ADVANTECH_PCM3681P,
		.subdevice	= PCI_ANY_ID,
		.setup		= pci_advantech_setup,
	},
	{
		.vendor		= PCI_VENDOR_ID_ADVANTECH,
		.device		= PCI_DEVICE_ID_ADVANTECH_UNO1150,
		.subvendor	= PCI_DEVICE_ID_ADVANTECH_UNO1150,
		.subdevice	= PCI_ANY_ID,
		.setup		= pci_advantech_setup,
	},
	{
		.vendor		= PCI_VENDOR_ID_ADVANTECH,
		.device		= PCI_DEVICE_ID_ADVANTECH_MIC3621,
		.subvendor	= PCI_SUB_VENDOR_ID_ADVANTECH_MIC3621,
		.subdevice	= PCI_ANY_ID,
		.setup		= pci_advantech_setup,
	},
	{
		.vendor		= PCI_VENDOR_ID_ADVANTECH,
		.device		= PCI_DEVICE_ID_ADVANTECH_A001,
		.subvendor	= PCI_ANY_ID,
		.subdevice	= PCI_ANY_ID,
		.setup		= pci_advantech_setup,
	},
	{
		.vendor		= PCI_VENDOR_ID_ADVANTECH,
		.device		= PCI_DEVICE_ID_ADVANTECH_A002,
		.subvendor	= PCI_ANY_ID,
		.subdevice	= PCI_ANY_ID,
		.setup		= pci_advantech_setup,
	},
	{
		.vendor		= PCI_VENDOR_ID_ADVANTECH,
		.device		= PCI_DEVICE_ID_ADVANTECH_A004,
		.subvendor	= PCI_ANY_ID,
		.subdevice	= PCI_ANY_ID,
		.setup		= pci_advantech_setup,
	},
	{
		.vendor		= PCI_VENDOR_ID_ADVANTECH,
		.device		= PCI_DEVICE_ID_ADVANTECH_A101,
		.subvendor	= PCI_ANY_ID,
		.subdevice	= PCI_ANY_ID,
		.setup		= pci_advantech_setup,
	},
	{
		.vendor		= PCI_VENDOR_ID_ADVANTECH,
		.device		= PCI_DEVICE_ID_ADVANTECH_A102,
		.subvendor	= PCI_ANY_ID,
		.subdevice	= PCI_ANY_ID,
		.setup		= pci_advantech_setup,
	},
	{
		.vendor		= PCI_VENDOR_ID_ADVANTECH,
		.device		= PCI_DEVICE_ID_ADVANTECH_A104,
		.subvendor	= PCI_ANY_ID,
		.subdevice	= PCI_ANY_ID,
		.setup		= pci_advantech_setup,
	},
	{
		.vendor		= PCI_VENDOR_ID_ADVANTECH,
		.device		= PCI_DEVICE_ID_ADVANTECH_F001,
		.subvendor	= PCI_ANY_ID,
		.subdevice	= PCI_ANY_ID,
		.setup		= pci_advantech_setup,
	},
	{
		.vendor		= PCI_VENDOR_ID_ADVANTECH,
		.device		= PCI_DEVICE_ID_ADVANTECH_F002,
		.subvendor	= PCI_ANY_ID,
		.subdevice	= PCI_ANY_ID,
		.setup		= pci_advantech_setup,
	},
	{
		.vendor		= PCI_VENDOR_ID_ADVANTECH,
		.device		= PCI_DEVICE_ID_ADVANTECH_F004,
		.subvendor	= PCI_ANY_ID,
		.subdevice	= PCI_ANY_ID,
		.setup		= pci_advantech_setup,
	},
	{
		.vendor		= PCI_VENDOR_ID_ADVANTECH,
		.device		= PCI_DEVICE_ID_ADVANTECH_F101,
		.subvendor	= PCI_ANY_ID,
		.subdevice	= PCI_ANY_ID,
		.setup		= pci_advantech_setup,
	},
	{
		.vendor		= PCI_VENDOR_ID_ADVANTECH,
		.device		= PCI_DEVICE_ID_ADVANTECH_F102,
		.subvendor	= PCI_ANY_ID,
		.subdevice	= PCI_ANY_ID,
		.setup		= pci_advantech_setup,
	},
	{
		.vendor		= PCI_VENDOR_ID_ADVANTECH,
		.device		= PCI_DEVICE_ID_ADVANTECH_F104,
		.subvendor	= PCI_ANY_ID,
		.subdevice	= PCI_ANY_ID,
		.setup		= pci_advantech_setup,
	},
	{
		.vendor		= PCI_VENDOR_ID_ADVANTECH,
		.device		= PCI_DEVICE_ID_ADVANTECH_A821,
		.subvendor	= PCI_ANY_ID,
		.subdevice	= PCI_ANY_ID,
		.setup		= pci_advantech_setup,
	},
	{
		.vendor		= PCI_VENDOR_ID_ADVANTECH,
		.device		= PCI_DEVICE_ID_ADVANTECH_A822,
		.subvendor	= PCI_ANY_ID,
		.subdevice	= PCI_ANY_ID,
		.setup		= pci_advantech_setup,
	},
	{
		.vendor		= PCI_VENDOR_ID_ADVANTECH,
		.device		= PCI_DEVICE_ID_ADVANTECH_A823,
		.subvendor	= PCI_ANY_ID,
		.subdevice	= PCI_ANY_ID,
		.setup		= pci_advantech_setup,
	},
	{
		.vendor		= PCI_VENDOR_ID_ADVANTECH,
		.device		= PCI_DEVICE_ID_ADVANTECH_A824,
		.subvendor	= PCI_ANY_ID,
		.subdevice	= PCI_ANY_ID,
		.setup		= pci_advantech_setup,
	},
	{
		.vendor		= PCI_VENDOR_ID_ADVANTECH,
		.device		= PCI_DEVICE_ID_ADVANTECH_A828,
		.subvendor	= PCI_ANY_ID,
		.subdevice	= PCI_ANY_ID,
		.setup		= pci_advantech_setup,
	},
	{
		.vendor		= PCI_VENDOR_ID_ADVANTECH,
		.device		= PCI_DEVICE_ID_ADVANTECH_A831,
		.subvendor	= PCI_ANY_ID,
		.subdevice	= PCI_ANY_ID,
		.setup		= pci_advantech_setup,
	},
	{
		.vendor		= PCI_VENDOR_ID_ADVANTECH,
		.device		= PCI_DEVICE_ID_ADVANTECH_A832,
		.subvendor	= PCI_ANY_ID,
		.subdevice	= PCI_ANY_ID,
		.setup		= pci_advantech_setup,
	},
	{
		.vendor		= PCI_VENDOR_ID_ADVANTECH,
		.device		= PCI_DEVICE_ID_ADVANTECH_A833,
		.subvendor	= PCI_ANY_ID,
		.subdevice	= PCI_ANY_ID,
		.setup		= pci_advantech_setup,
	},
	{
		.vendor		= PCI_VENDOR_ID_ADVANTECH,
		.device		= PCI_DEVICE_ID_ADVANTECH_A834,
		.subvendor	= PCI_ANY_ID,
		.subdevice	= PCI_ANY_ID,
		.setup		= pci_advantech_setup,
	},
	{
		.vendor		= PCI_VENDOR_ID_ADVANTECH,
		.device		= PCI_DEVICE_ID_ADVANTECH_A838,
		.subvendor	= PCI_ANY_ID,
		.subdevice	= PCI_ANY_ID,
		.setup		= pci_advantech_setup,
	},
};


static inline int quirk_id_matches(u32 quirk_id, u32 dev_id)
{
	return quirk_id == PCI_ANY_ID || quirk_id == dev_id;
}

static struct pci_serial_quirk *find_quirk(struct pci_dev *dev)
{
	struct pci_serial_quirk *quirk;

	for (quirk = pci_serial_quirks; ; quirk++)
		if (quirk_id_matches(quirk->vendor, dev->vendor) &&
		    quirk_id_matches(quirk->device, dev->device) &&
		    quirk_id_matches(quirk->subvendor, dev->subsystem_vendor) &&
		    quirk_id_matches(quirk->subdevice, dev->subsystem_device))
			break;
	return quirk;
}

static inline int get_pci_irq(struct pci_dev *dev,
				const struct pciserial_board *board)
{
	if (board->flags & FL_NOIRQ)
		return 0;
	else
		return dev->irq;
}

/*
 * This is the configuration table for all of the PCI serial boards
 * which we support.  It is directly indexed by the pci_board_num_t enum
 * value, which is encoded in the pci_device_id PCI probe table's
 * driver_data member.
 *
 * The makeup of these names are:
 *  pbn_bn{_bt}_n_baud{_offsetinhex}
 *
 *  bn		= PCI BAR number
 *  bt		= Index using PCI BARs
 *  n		= number of serial ports
 *  baud	= baud rate
 *  offsetinhex	= offset for each sequential port (in hex)
 *
 * This table is sorted by (in order): bn, bt, baud, offsetindex, n.
 *
 * Please note: in theory if n = 1, _bt infix should make no difference.
 * ie, pbn_b0_1_115200 is the same as pbn_b0_bt_1_115200
 */
enum pci_board_num_t {
	pbn_default = 0,

	pbn_b0_1_115200,
	pbn_b0_2_115200,
	pbn_b0_4_115200,
	pbn_b0_5_115200,

	pbn_b0_1_921600,
	pbn_b0_2_921600,
	pbn_b0_4_921600,

	pbn_b0_2_d_921600,
	pbn_b0_4_d_921600,
	pbn_b0_8_d_921600,
	pbn_b0_16_d_921600,

	pbn_b0_bt_1_115200,
	pbn_b0_bt_2_115200,
	pbn_b0_bt_8_115200,

	pbn_b0_bt_1_460800,
	pbn_b0_bt_2_460800,
	pbn_b0_bt_4_460800,

	pbn_b0_bt_1_921600,
	pbn_b0_bt_2_921600,
	pbn_b0_bt_4_921600,
	pbn_b0_bt_8_921600,

	pbn_b1_1_115200,
	pbn_b1_2_115200,
	pbn_b1_4_115200,
	pbn_b1_8_115200,

	pbn_b1_1_921600,
	pbn_b1_2_921600,
	pbn_b1_4_921600,
	pbn_b1_8_921600,

	pbn_b1_bt_2_921600,

	pbn_b1_2_1382400,
	pbn_b1_4_1382400,
	pbn_b1_8_1382400,

	pbn_b2_1_115200,
	pbn_b2_8_115200,

	pbn_b2_1_460800,
	pbn_b2_4_460800,
	pbn_b2_8_460800,
	pbn_b2_16_460800,

	pbn_b2_1_921600,
	pbn_b2_4_921600,
	pbn_b2_8_921600,

	pbn_b2_bt_1_115200,
	pbn_b2_bt_2_115200,
	pbn_b2_bt_4_115200,

	pbn_b2_bt_2_921600,
	pbn_b2_bt_4_921600,

	pbn_b3_4_115200,
	pbn_b3_8_115200,

	pbn_b0_1_xr_921600,
	pbn_b0_2_xr_921600,
	pbn_b0_3_xr_921600,
	pbn_b0_4_xr_921600,
	pbn_b0_8_xr_921600,

};

/*
 * uart_offset - the space between channels
 * reg_shift   - describes how the UART registers are mapped
 *               to PCI memory by the card.
 * For example IER register on SBS, Inc. PMC-OctPro is located at
 * offset 0x10 from the UART base, while UART_IER is defined as 1
 * in include/linux/serial_reg.h,
 * see first lines of serial_in() and serial_out() in 8250.c
*/

static struct pciserial_board pci_boards[] = {
	[pbn_default] = {
		.flags		= FL_BASE0,
		.num_ports	= 1,
		.base_baud	= 115200,
		.uart_offset	= 8,
	},
	[pbn_b0_1_115200] = {
		.flags		= FL_BASE0,
		.num_ports	= 1,
		.base_baud	= 115200,
		.uart_offset	= 8,
	},
	[pbn_b0_2_115200] = {
		.flags		= FL_BASE0,
		.num_ports	= 2,
		.base_baud	= 115200,
		.uart_offset	= 8,
	},
	[pbn_b0_4_115200] = {
		.flags		= FL_BASE0,
		.num_ports	= 4,
		.base_baud	= 115200,
		.uart_offset	= 8,
	},
	[pbn_b0_5_115200] = {
		.flags		= FL_BASE0,
		.num_ports	= 5,
		.base_baud	= 115200,
		.uart_offset	= 8,
	},

	[pbn_b0_1_921600] = {
		.flags		= FL_BASE0,
		.num_ports	= 1,
		.base_baud	= 921600,
		.uart_offset	= 8,
	},
	[pbn_b0_2_921600] = {
		.flags		= FL_BASE0,
		.num_ports	= 2,
		.base_baud	= 921600,
		.uart_offset	= 8,
	},
	[pbn_b0_4_921600] = {
		.flags		= FL_BASE0,
		.num_ports	= 4,
		.base_baud	= 921600,
		.uart_offset	= 8,
	},

	[pbn_b0_2_d_921600] = {
		.flags		= FL_BASE0,
		.num_ports	= 2,
		.base_baud	= 921600,
		.uart_offset	= 0x200,
		.first_offset   = 0x1000,
	},
	[pbn_b0_4_d_921600] = {
		.flags		= FL_BASE0,
		.num_ports	= 4,
		.base_baud	= 921600,
		.uart_offset	= 0x200,
		.first_offset   = 0x1000,
	},

	[pbn_b0_8_d_921600] = {
		.flags		= FL_BASE0,
		.num_ports	= 8,
		.base_baud	= 921600,
		.uart_offset	= 0x200,
		.first_offset   = 0x1000,
	},
	[pbn_b0_16_d_921600] = {
		.flags		= FL_BASE0,
		.num_ports	= 16,
		.base_baud	= 921600,
		.uart_offset	= 0x200,
		.first_offset   = 0x1000,
	},
	[pbn_b0_bt_1_115200] = {
		.flags		= FL_BASE0|FL_BASE_BARS,
		.num_ports	= 1,
		.base_baud	= 115200,
		.uart_offset	= 8,
	},
	[pbn_b0_bt_2_115200] = {
		.flags		= FL_BASE0|FL_BASE_BARS,
		.num_ports	= 2,
		.base_baud	= 115200,
		.uart_offset	= 8,
	},
	[pbn_b0_bt_8_115200] = {
		.flags		= FL_BASE0|FL_BASE_BARS,
		.num_ports	= 8,
		.base_baud	= 115200,
		.uart_offset	= 8,
	},

	[pbn_b0_bt_1_460800] = {
		.flags		= FL_BASE0|FL_BASE_BARS,
		.num_ports	= 1,
		.base_baud	= 460800,
		.uart_offset	= 8,
	},
	[pbn_b0_bt_2_460800] = {
		.flags		= FL_BASE0|FL_BASE_BARS,
		.num_ports	= 2,
		.base_baud	= 460800,
		.uart_offset	= 8,
	},
	[pbn_b0_bt_4_460800] = {
		.flags		= FL_BASE0|FL_BASE_BARS,
		.num_ports	= 4,
		.base_baud	= 460800,
		.uart_offset	= 8,
	},

	[pbn_b0_bt_1_921600] = {
		.flags		= FL_BASE0|FL_BASE_BARS,
		.num_ports	= 1,
		.base_baud	= 921600,
		.uart_offset	= 8,
	},
	[pbn_b0_bt_2_921600] = {
		.flags		= FL_BASE0|FL_BASE_BARS,
		.num_ports	= 2,
		.base_baud	= 921600,
		.uart_offset	= 8,
	},
	[pbn_b0_bt_4_921600] = {
		.flags		= FL_BASE0|FL_BASE_BARS,
		.num_ports	= 4,
		.base_baud	= 921600,
		.uart_offset	= 8,
	},
	[pbn_b0_bt_8_921600] = {
		.flags		= FL_BASE0|FL_BASE_BARS,
		.num_ports	= 8,
		.base_baud	= 921600,
		.uart_offset	= 8,
	},

	[pbn_b1_1_115200] = {
		.flags		= FL_BASE1,
		.num_ports	= 1,
		.base_baud	= 115200,
		.uart_offset	= 8,
	},
	[pbn_b1_2_115200] = {
		.flags		= FL_BASE1,
		.num_ports	= 2,
		.base_baud	= 115200,
		.uart_offset	= 8,
	},
	[pbn_b1_4_115200] = {
		.flags		= FL_BASE1,
		.num_ports	= 4,
		.base_baud	= 115200,
		.uart_offset	= 8,
	},
	[pbn_b1_8_115200] = {
		.flags		= FL_BASE1,
		.num_ports	= 8,
		.base_baud	= 115200,
		.uart_offset	= 8,
	},

	[pbn_b1_1_921600] = {
		.flags		= FL_BASE1,
		.num_ports	= 1,
		.base_baud	= 921600,
		.uart_offset	= 8,
	},
	[pbn_b1_2_921600] = {
		.flags		= FL_BASE1,
		.num_ports	= 2,
		.base_baud	= 921600,
		.uart_offset	= 8,
	},
	[pbn_b1_4_921600] = {
		.flags		= FL_BASE1,
		.num_ports	= 4,
		.base_baud	= 921600,
		.uart_offset	= 8,
	},
	[pbn_b1_8_921600] = {
		.flags		= FL_BASE1,
		.num_ports	= 8,
		.base_baud	= 921600,
		.uart_offset	= 8,
	},

	[pbn_b1_bt_2_921600] = {
		.flags		= FL_BASE1|FL_BASE_BARS,
		.num_ports	= 2,
		.base_baud	= 921600,
		.uart_offset	= 8,
	},

	[pbn_b1_2_1382400] = {
		.flags		= FL_BASE1,
		.num_ports	= 2,
		.base_baud	= 1382400,
		.uart_offset	= 8,
	},
	[pbn_b1_4_1382400] = {
		.flags		= FL_BASE1,
		.num_ports	= 4,
		.base_baud	= 1382400,
		.uart_offset	= 8,
	},
	[pbn_b1_8_1382400] = {
		.flags		= FL_BASE1,
		.num_ports	= 8,
		.base_baud	= 1382400,
		.uart_offset	= 8,
	},

	[pbn_b2_1_115200] = {
		.flags		= FL_BASE2,
		.num_ports	= 1,
		.base_baud	= 115200,
		.uart_offset	= 8,
	},
	[pbn_b2_8_115200] = {
		.flags		= FL_BASE2,
		.num_ports	= 8,
		.base_baud	= 115200,
		.uart_offset	= 8,
	},

	[pbn_b2_1_460800] = {
		.flags		= FL_BASE2,
		.num_ports	= 1,
		.base_baud	= 460800,
		.uart_offset	= 8,
	},
	[pbn_b2_4_460800] = {
		.flags		= FL_BASE2,
		.num_ports	= 4,
		.base_baud	= 460800,
		.uart_offset	= 8,
	},
	[pbn_b2_8_460800] = {
		.flags		= FL_BASE2,
		.num_ports	= 8,
		.base_baud	= 460800,
		.uart_offset	= 8,
	},
	[pbn_b2_16_460800] = {
		.flags		= FL_BASE2,
		.num_ports	= 16,
		.base_baud	= 460800,
		.uart_offset	= 8,
	 },

	[pbn_b2_1_921600] = {
		.flags		= FL_BASE2,
		.num_ports	= 1,
		.base_baud	= 921600,
		.uart_offset	= 8,
	},
	[pbn_b2_4_921600] = {
		.flags		= FL_BASE2,
		.num_ports	= 4,
		.base_baud	= 921600,
		.uart_offset	= 8,
	},
	[pbn_b2_8_921600] = {
		.flags		= FL_BASE2,
		.num_ports	= 8,
		.base_baud	= 921600,
		.uart_offset	= 8,
	},

	[pbn_b2_bt_1_115200] = {
		.flags		= FL_BASE2|FL_BASE_BARS,
		.num_ports	= 1,
		.base_baud	= 115200,
		.uart_offset	= 8,
	},
	[pbn_b2_bt_2_115200] = {
		.flags		= FL_BASE2|FL_BASE_BARS,
		.num_ports	= 2,
		.base_baud	= 115200,
		.uart_offset	= 8,
	},
	[pbn_b2_bt_4_115200] = {
		.flags		= FL_BASE2|FL_BASE_BARS,
		.num_ports	= 4,
		.base_baud	= 115200,
		.uart_offset	= 8,
	},

	[pbn_b2_bt_2_921600] = {
		.flags		= FL_BASE2|FL_BASE_BARS,
		.num_ports	= 2,
		.base_baud	= 921600,
		.uart_offset	= 8,
	},
	[pbn_b2_bt_4_921600] = {
		.flags		= FL_BASE2|FL_BASE_BARS,
		.num_ports	= 4,
		.base_baud	= 921600,
		.uart_offset	= 8,
	},

	[pbn_b3_4_115200] = {
		.flags		= FL_BASE3,
		.num_ports	= 4,
		.base_baud	= 115200,
		.uart_offset	= 8,
	},
	[pbn_b3_8_115200] = {
		.flags		= FL_BASE3,
		.num_ports	= 8,
		.base_baud	= 115200,
		.uart_offset	= 8,
	},


	[pbn_b0_1_xr_921600] = {
		.flags		= FL_BASE0,
		.num_ports	= 1,
		.base_baud	= 921600,
		.uart_offset	= 0x200,
		.reg_shift	= 0,
		.first_offset	= 0,
	},

	[pbn_b0_2_xr_921600] = {
		.flags		= FL_BASE0,
		.num_ports	= 2,
		.base_baud	= 921600,
		.uart_offset	= 0x200,
		.reg_shift	= 0,
		.first_offset	= 0,
	},

	[pbn_b0_3_xr_921600] = {
		.flags		= FL_BASE0,
		.num_ports	= 3,
		.base_baud	= 921600,
		.uart_offset	= 0x200,
		.reg_shift	= 0,
		.first_offset	= 0,
	},

	[pbn_b0_4_xr_921600] = {
		.flags		= FL_BASE0,
		.num_ports	= 4,
		.base_baud	= 921600,
		.uart_offset	= 0x200,
		.reg_shift	= 0,
		.first_offset	= 0,
	},

	[pbn_b0_8_xr_921600] = {
		.flags		= FL_BASE0,
		.num_ports	= 8,
		.base_baud	= 921600,
		.uart_offset	= 0x200,
		.reg_shift	= 0,
		.first_offset	= 0,
	},


};

static const struct pci_device_id blacklist[] = {
	/* softmodems */
	{ PCI_VDEVICE(AL, 0x5457), }, /* ALi Corporation M5457 AC'97 Modem */
	{ PCI_VDEVICE(MOTOROLA, 0x3052), }, /* Motorola Si3052-based modem */
	{ PCI_DEVICE(0x1543, 0x3052), }, /* Si3052-based modem, default IDs */

	/* multi-io cards handled by parport_serial */
	{ PCI_DEVICE(0x4348, 0x7053), }, /* WCH CH353 2S1P */
	{ PCI_DEVICE(0x4348, 0x5053), }, /* WCH CH353 1S1P */
	{ PCI_DEVICE(0x1c00, 0x3250), }, /* WCH CH382 2S1P */
	{ PCI_DEVICE(0x1c00, 0x3470), }, /* WCH CH384 4S */

	/* Intel platforms with MID UART */
	{ PCI_VDEVICE(INTEL, 0x081b), },
	{ PCI_VDEVICE(INTEL, 0x081c), },
	{ PCI_VDEVICE(INTEL, 0x081d), },
	{ PCI_VDEVICE(INTEL, 0x1191), },
	{ PCI_VDEVICE(INTEL, 0x19d8), },
};

/*
 * Given a complete unknown PCI device, try to use some heuristics to
 * guess what the configuration might be, based on the pitiful PCI
 * serial specs.  Returns 0 on success, 1 on failure.
 */
static int
serial_pci_guess_board(struct pci_dev *dev, struct pciserial_board *board)
{
	const struct pci_device_id *bldev;
	int num_iomem, num_port, first_port = -1, i;

	/*
	 * If it is not a communications device or the programming
	 * interface is greater than 6, give up.
	 *
	 * (Should we try to make guesses for multiport serial devices
	 * later?)
	 */
	if ((((dev->class >> 8) != PCI_CLASS_COMMUNICATION_SERIAL) &&
	     ((dev->class >> 8) != PCI_CLASS_COMMUNICATION_MODEM)) ||
	    (dev->class & 0xff) > 6)
		return -ENODEV;

	/*
	 * Do not access blacklisted devices that are known not to
	 * feature serial ports or are handled by other modules.
	 */
	for (bldev = blacklist;
	     bldev < blacklist + ARRAY_SIZE(blacklist);
	     bldev++) {
		if (dev->vendor == bldev->vendor &&
		    dev->device == bldev->device)
			return -ENODEV;
	}

	num_iomem = num_port = 0;
	for (i = 0; i < PCI_NUM_BAR_RESOURCES; i++) {
		if (pci_resource_flags(dev, i) & IORESOURCE_IO) {
			num_port++;
			if (first_port == -1)
				first_port = i;
		}
		if (pci_resource_flags(dev, i) & IORESOURCE_MEM)
			num_iomem++;
	}

	/*
	 * If there is 1 or 0 iomem regions, and exactly one port,
	 * use it.  We guess the number of ports based on the IO
	 * region size.
	 */
	if (num_iomem <= 1 && num_port == 1) {
		board->flags = first_port;
		board->num_ports = pci_resource_len(dev, first_port) / 8;
		return 0;
	}

	/*
	 * Now guess if we've got a board which indexes by BARs.
	 * Each IO BAR should be 8 bytes, and they should follow
	 * consecutively.
	 */
	first_port = -1;
	num_port = 0;
	for (i = 0; i < PCI_NUM_BAR_RESOURCES; i++) {
		if (pci_resource_flags(dev, i) & IORESOURCE_IO &&
		    pci_resource_len(dev, i) == 8 &&
		    (first_port == -1 || (first_port + num_port) == i)) {
			num_port++;
			if (first_port == -1)
				first_port = i;
		}
	}

	if (num_port > 1) {
		board->flags = first_port | FL_BASE_BARS;
		board->num_ports = num_port;
		return 0;
	}

	return -ENODEV;
}

static inline int
serial_pci_matches(const struct pciserial_board *board,
		   const struct pciserial_board *guessed)
{
	return
	    board->num_ports == guessed->num_ports &&
	    board->base_baud == guessed->base_baud &&
	    board->uart_offset == guessed->uart_offset &&
	    board->reg_shift == guessed->reg_shift &&
	    board->first_offset == guessed->first_offset;
}

struct serial_private *
adv_pciserial_init_ports(struct pci_dev *dev, const struct pciserial_board *board)
{
	struct uart_8250_port uart;
	struct serial_private *priv;
	struct pci_serial_quirk *quirk;
	int rc, nr_ports, i;

	nr_ports = board->num_ports;

	/*
	 * Find an init and setup quirks.
	 */
	quirk = find_quirk(dev);

	/*
	 * Run the new-style initialization function.
	 * The initialization function returns:
	 *  <0  - error
	 *   0  - use board->num_ports
	 *  >0  - number of ports
	 */
	if (quirk->init) {
		rc = quirk->init(dev);
		if (rc < 0) {
			priv = ERR_PTR(rc);
			goto err_out;
		}
		if (rc)
			nr_ports = rc;
	}

	priv = kzalloc(sizeof(struct serial_private) +
		       sizeof(unsigned int) * nr_ports,
		       GFP_KERNEL);
	if (!priv) {
		priv = ERR_PTR(-ENOMEM);
		goto err_deinit;
	}

	priv->dev = dev;
	priv->quirk = quirk;

	memset(&uart, 0, sizeof(uart));
	uart.port.flags = UPF_SKIP_TEST | UPF_BOOT_AUTOCONF | UPF_SHARE_IRQ;
	uart.port.uartclk = board->base_baud * 16;
	uart.port.irq = get_pci_irq(dev, board);
	uart.port.dev = &dev->dev;

	for (i = 0; i < nr_ports; i++) {
		if (quirk->setup(priv, board, &uart, i))
			break;

		dev_dbg(&dev->dev, "Setup PCI port: port %lx, irq %d, type %d\n",
			uart.port.iobase, uart.port.irq, uart.port.iotype);

		priv->line[i] = adv_serial8250_register_port(&uart);	// Custom function required to get ttyAPx name
		//priv->line[i] = serial8250_register_8250_port(&uart);
		if (priv->line[i] < 0) {
			dev_err(&dev->dev,
				"Couldn't register serial port %lx, irq %d, type %d, error %d\n",
				uart.port.iobase, uart.port.irq,
				uart.port.iotype, priv->line[i]);
			break;
		}
	}
	priv->nr = i;
	priv->board = board;
	return priv;

err_deinit:
	if (quirk->exit)
		quirk->exit(dev);
err_out:
	return priv;
}

void adv_pciserial_detach_ports(struct serial_private *priv)
{
	struct pci_serial_quirk *quirk;
	int i;

	for (i = 0; i < priv->nr; i++)
		adv_serial8250_unregister_port(priv->line[i]);

	for (i = 0; i < PCI_NUM_BAR_RESOURCES; i++) {
		if (priv->remapped_bar[i])
			iounmap(priv->remapped_bar[i]);
		priv->remapped_bar[i] = NULL;
	}

	/*
	 * Find the exit quirks.
	 */
	quirk = find_quirk(priv->dev);
	if (quirk->exit)
		quirk->exit(priv->dev);
}

void adv_pciserial_remove_ports(struct serial_private *priv)
{
	adv_pciserial_detach_ports(priv);
	kfree(priv);
}
//EXPORT_SYMBOL_GPL(pciserial_remove_ports);

/*
 * Probe one serial board.  Unfortunately, there is no rhyme nor reason
 * to the arrangement of serial ports on a PCI card.
 */
static int
pciserial_init_one(struct pci_dev *dev, const struct pci_device_id *ent)
{
	struct pci_serial_quirk *quirk;
	struct serial_private *priv;
	const struct pciserial_board *board;
	struct pciserial_board tmp;
	int rc;

	quirk = find_quirk(dev);
	if (quirk->probe) {
		rc = quirk->probe(dev);
		if (rc)
			return rc;
	}

	if (ent->driver_data >= ARRAY_SIZE(pci_boards)) {
		dev_err(&dev->dev, "invalid driver_data: %ld\n",
			ent->driver_data);
		return -EINVAL;
	}

	board = &pci_boards[ent->driver_data];

	rc = pci_enable_device(dev);
	pci_save_state(dev);
	if (rc)
		return rc;

	if (ent->driver_data == pbn_default) {
		/*
		 * Use a copy of the pci_board entry for this;
		 * avoid changing entries in the table.
		 */
		memcpy(&tmp, board, sizeof(struct pciserial_board));
		board = &tmp;

		/*
		 * We matched one of our class entries.  Try to
		 * determine the parameters of this board.
		 */
		rc = serial_pci_guess_board(dev, &tmp);
		if (rc)
			goto disable;
	} else {
		/*
		 * We matched an explicit entry.  If we are able to
		 * detect this boards settings with our heuristic,
		 * then we no longer need this entry.
		 */
		memcpy(&tmp, &pci_boards[pbn_default],
		       sizeof(struct pciserial_board));
		rc = serial_pci_guess_board(dev, &tmp);
		if (rc == 0 && serial_pci_matches(board, &tmp))
			moan_device("Redundant entry in serial pci_table.",
				    dev);
	}

	priv = adv_pciserial_init_ports(dev, board);
	if (!IS_ERR(priv)) {
		pci_set_drvdata(dev, priv);
		return 0;
	}

	rc = PTR_ERR(priv);

 disable:
	pci_disable_device(dev);
	return rc;
}

static void pciserial_remove_one(struct pci_dev *dev)
{
	struct serial_private *priv = pci_get_drvdata(dev);

	adv_pciserial_remove_ports(priv);

	pci_disable_device(dev);
}

#ifdef CONFIG_PM_SLEEP
static int pciserial_suspend_one(struct device *dev)
{
	struct pci_dev *pdev = to_pci_dev(dev);
	struct serial_private *priv = pci_get_drvdata(pdev);

	if (priv)
		pciserial_suspend_ports(priv);

	return 0;
}

static int pciserial_resume_one(struct device *dev)
{
	struct pci_dev *pdev = to_pci_dev(dev);
	struct serial_private *priv = pci_get_drvdata(pdev);
	int err;

	if (priv) {
		/*
		 * The device may have been disabled.  Re-enable it.
		 */
		err = pci_enable_device(pdev);
		/* FIXME: We cannot simply error out here */
		if (err)
			dev_err(dev, "Unable to re-enable ports, trying to continue.\n");
		pciserial_resume_ports(priv);
	}
	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(pciserial_pm_ops, pciserial_suspend_one,
			 pciserial_resume_one);

static struct pci_device_id serial_pci_tbl[] = {
	/*
	 * Advantech IAG PCI-954/16C950 cards
	 */
	{	PCI_VENDOR_ID_ADVANTECH, PCI_DEVICE_ID_ADVANTECH_PCI1600,
		PCI_DEVICE_ID_ADVANTECH_PCI1600_1601, PCI_ANY_ID, 0, 0,
		pbn_b0_2_921600 },
	{	PCI_VENDOR_ID_ADVANTECH, PCI_DEVICE_ID_ADVANTECH_PCI1600,
		PCI_DEVICE_ID_ADVANTECH_PCI1600_1602, PCI_ANY_ID, 0, 0,
		pbn_b0_2_921600 },
	{	PCI_VENDOR_ID_ADVANTECH, PCI_DEVICE_ID_ADVANTECH_PCI1600,
		PCI_DEVICE_ID_ADVANTECH_PCI1600_1610, PCI_ANY_ID, 0, 0,
		pbn_b0_4_921600 },
	{	PCI_VENDOR_ID_ADVANTECH, PCI_DEVICE_ID_ADVANTECH_PCI1600,
		PCI_DEVICE_ID_ADVANTECH_PCI1600_1611, PCI_ANY_ID, 0, 0,
		pbn_b0_4_921600 },
	{	PCI_VENDOR_ID_ADVANTECH, PCI_DEVICE_ID_ADVANTECH_PCI1600,
		PCI_DEVICE_ID_ADVANTECH_PCI1600_1612, PCI_ANY_ID, 0, 0,
		pbn_b0_4_921600 },
	{	PCI_VENDOR_ID_ADVANTECH, PCI_DEVICE_ID_ADVANTECH_PCI1600,
		PCI_DEVICE_ID_ADVANTECH_PCI1600_1620, PCI_ANY_ID, 0, 0,
		pbn_b0_4_921600 },
	{	PCI_VENDOR_ID_ADVANTECH, PCI_DEVICE_ID_ADVANTECH_PCI16ff,
		PCI_DEVICE_ID_ADVANTECH_PCI1600_1620, PCI_ANY_ID, 0, 0,
		pbn_b0_4_921600 },
	{	PCI_VENDOR_ID_ADVANTECH, PCI_DEVICE_ID_ADVANTECH_PCI1600,
		PCI_DEVICE_ID_ADVANTECH_PCI1600_1622, PCI_ANY_ID, 0, 0,
		pbn_b0_4_921600 },
	{	PCI_VENDOR_ID_ADVANTECH, PCI_DEVICE_ID_ADVANTECH_PCI16ff,
		PCI_DEVICE_ID_ADVANTECH_PCI1600_1622, PCI_ANY_ID, 0, 0,
		pbn_b0_4_921600 },
	{	PCI_VENDOR_ID_ADVANTECH, PCI_DEVICE_ID_ADVANTECH_UNO2050,
		PCI_DEVICE_ID_ADVANTECH_UNO2050, PCI_ANY_ID, 0, 0,
		pbn_b0_2_921600 },
	{	PCI_VENDOR_ID_ADVANTECH, PCI_DEVICE_ID_ADVANTECH_UNOB2201,
		PCI_DEVICE_ID_ADVANTECH_UNOBX201_2201, PCI_ANY_ID, 0, 0,
		pbn_b0_4_921600 },
	{	PCI_VENDOR_ID_ADVANTECH, PCI_DEVICE_ID_ADVANTECH_UNOBF201,
		PCI_DEVICE_ID_ADVANTECH_UNOBX201_2201, PCI_ANY_ID, 0, 0,
		pbn_b0_4_921600 },
	{	PCI_VENDOR_ID_ADVANTECH, PCI_DEVICE_ID_ADVANTECH_MIC3612,
		PCI_DEVICE_ID_ADVANTECH_MIC3612, PCI_ANY_ID, 0, 0,
		pbn_b2_4_921600 },
	{	PCI_VENDOR_ID_ADVANTECH, PCI_DEVICE_ID_ADVANTECH_MIC3620,
		PCI_DEVICE_ID_ADVANTECH_MIC3620, PCI_ANY_ID, 0, 0,
		pbn_b2_8_921600 },
	{	PCI_VENDOR_ID_ADVANTECH, PCI_DEVICE_ID_ADVANTECH_MIC3611,
		PCI_DEVICE_ID_ADVANTECH_MIC3611, PCI_ANY_ID, 0, 0,
		pbn_b2_4_921600 },
	{	PCI_VENDOR_ID_ADVANTECH, PCI_DEVICE_ID_ADVANTECH_PCI1601,
		PCI_VENDOR_ID_ADVANTECH, PCI_ANY_ID, 0, 0,
		pbn_b0_bt_2_921600 },
	{	PCI_VENDOR_ID_ADVANTECH, PCI_DEVICE_ID_ADVANTECH_PCI1602,
		PCI_VENDOR_ID_ADVANTECH, PCI_ANY_ID, 0, 0,
		pbn_b0_bt_2_921600 },
	{	PCI_VENDOR_ID_ADVANTECH, PCI_DEVICE_ID_ADVANTECH_PCI1603,
		PCI_VENDOR_ID_ADVANTECH, PCI_ANY_ID, 0, 0,
		pbn_b0_bt_2_921600 },
	{	PCI_VENDOR_ID_ADVANTECH, PCI_DEVICE_ID_ADVANTECH_PCI1604,
		PCI_VENDOR_ID_ADVANTECH, PCI_ANY_ID, 0, 0,
		pbn_b0_bt_2_921600 },
	{	PCI_VENDOR_ID_ADVANTECH, PCI_DEVICE_ID_ADVANTECH_UNO2176,
		PCI_DEVICE_ID_ADVANTECH_UNO2X76_2176, PCI_ANY_ID, 0, 0,
		pbn_b0_4_921600 },
	{	PCI_VENDOR_ID_ADVANTECH, PCI_DEVICE_ID_ADVANTECH_PCIE952,
		PCI_ANY_ID, PCI_ANY_ID, 0, 0,
		pbn_b0_2_d_921600 },
	{	PCI_VENDOR_ID_ADVANTECH, PCI_DEVICE_ID_ADVANTECH_PCIE954,
		PCI_ANY_ID, PCI_ANY_ID, 0, 0,
		pbn_b0_4_d_921600 },
	{	PCI_VENDOR_ID_ADVANTECH, PCI_DEVICE_ID_ADVANTECH_PCIE958,
		PCI_ANY_ID, PCI_ANY_ID, 0, 0,
		pbn_b0_8_d_921600 },
	{	PCI_VENDOR_ID_ADVANTECH, PCI_DEVICE_ID_ADVANTECH_A516,
		PCI_ANY_ID, PCI_ANY_ID, 0, 0,
		pbn_b0_16_d_921600 },
	{	PCI_VENDOR_ID_ADVANTECH, PCI_DEVICE_ID_ADVANTECH_PCM3614P,
		PCI_SUB_VENDOR_ID_ADVANTECH_PCM3614P, PCI_ANY_ID, 0, 0,
		pbn_b0_4_921600 },
	{	PCI_VENDOR_ID_ADVANTECH, PCI_DEVICE_ID_ADVANTECH_PCM3618P,
		PCI_SUB_VENDOR_ID_ADVANTECH_PCM3618P, PCI_ANY_ID, 0, 0,
		pbn_b0_4_921600 },
	{	PCI_VENDOR_ID_ADVANTECH, PCI_DEVICE_ID_ADVANTECH_PCMF618P,
		PCI_SUB_VENDOR_ID_ADVANTECH_PCM3618P, PCI_ANY_ID, 0, 0,
		pbn_b0_4_921600 },
	{	PCI_VENDOR_ID_ADVANTECH, PCI_DEVICE_ID_ADVANTECH_PCM3641P,
		PCI_SUB_VENDOR_ID_ADVANTECH_PCM3641P, PCI_ANY_ID, 0, 0,
		pbn_b0_4_921600 },
	{	PCI_VENDOR_ID_ADVANTECH, PCI_DEVICE_ID_ADVANTECH_PCM3681P,
		PCI_SUB_VENDOR_ID_ADVANTECH_PCM3681P, PCI_ANY_ID, 0, 0,
		pbn_b0_4_921600},
	{	PCI_VENDOR_ID_ADVANTECH, PCI_DEVICE_ID_ADVANTECH_PCMF681P,
		PCI_SUB_VENDOR_ID_ADVANTECH_PCM3681P, PCI_ANY_ID, 0, 0,
		pbn_b0_4_921600 },
	{	PCI_VENDOR_ID_ADVANTECH, PCI_DEVICE_ID_ADVANTECH_UNO1150,
		PCI_DEVICE_ID_ADVANTECH_UNO1150, PCI_ANY_ID, 0, 0,
		pbn_b0_bt_2_921600 },
	{	PCI_VENDOR_ID_ADVANTECH, PCI_DEVICE_ID_ADVANTECH_MIC3621,
		PCI_SUB_VENDOR_ID_ADVANTECH_MIC3621, PCI_ANY_ID, 0, 0,
		pbn_b2_8_921600 },
	{	PCI_VENDOR_ID_ADVANTECH, PCI_DEVICE_ID_ADVANTECH_A001,
		PCI_ANY_ID, PCI_ANY_ID, 0, 0,
		pbn_b0_1_921600 },
	{	PCI_VENDOR_ID_ADVANTECH, PCI_DEVICE_ID_ADVANTECH_A002,
		PCI_ANY_ID, PCI_ANY_ID, 0, 0,
		pbn_b0_2_921600 },
	{	PCI_VENDOR_ID_ADVANTECH, PCI_DEVICE_ID_ADVANTECH_A004,
		PCI_ANY_ID, PCI_ANY_ID, 0, 0,
		pbn_b0_4_921600 },
	{	PCI_VENDOR_ID_ADVANTECH, PCI_DEVICE_ID_ADVANTECH_A101,
		PCI_ANY_ID, PCI_ANY_ID, 0, 0,
		pbn_b0_bt_1_921600 },
	{	PCI_VENDOR_ID_ADVANTECH, PCI_DEVICE_ID_ADVANTECH_A102,
		PCI_ANY_ID, PCI_ANY_ID, 0, 0,
		pbn_b0_bt_2_921600 },
	{	PCI_VENDOR_ID_ADVANTECH, PCI_DEVICE_ID_ADVANTECH_A104,
		PCI_ANY_ID, PCI_ANY_ID, 0, 0,
		pbn_b0_bt_4_921600 },
	{	PCI_VENDOR_ID_ADVANTECH, PCI_DEVICE_ID_ADVANTECH_F001,
		PCI_ANY_ID, PCI_ANY_ID, 0, 0,
		pbn_b0_1_921600 },
	{	PCI_VENDOR_ID_ADVANTECH, PCI_DEVICE_ID_ADVANTECH_F002,
		PCI_ANY_ID, PCI_ANY_ID, 0, 0,
		pbn_b0_2_921600 },
	{	PCI_VENDOR_ID_ADVANTECH, PCI_DEVICE_ID_ADVANTECH_F004,
		PCI_ANY_ID, PCI_ANY_ID, 0, 0,
		pbn_b0_4_921600 },
	{	PCI_VENDOR_ID_ADVANTECH, PCI_DEVICE_ID_ADVANTECH_F101,
		PCI_ANY_ID, PCI_ANY_ID, 0, 0,
		pbn_b0_bt_1_921600 },
	{	PCI_VENDOR_ID_ADVANTECH, PCI_DEVICE_ID_ADVANTECH_F102,
		PCI_ANY_ID, PCI_ANY_ID, 0, 0,
		pbn_b0_bt_2_921600 },
	{	PCI_VENDOR_ID_ADVANTECH, PCI_DEVICE_ID_ADVANTECH_F104,
		PCI_ANY_ID, PCI_ANY_ID, 0, 0,
		pbn_b0_bt_4_921600 },
	{	PCI_VENDOR_ID_ADVANTECH, PCI_DEVICE_ID_ADVANTECH_A821,
		PCI_ANY_ID, PCI_ANY_ID, 0, 0,
		pbn_b0_1_xr_921600 },
	{	PCI_VENDOR_ID_ADVANTECH, PCI_DEVICE_ID_ADVANTECH_A822,
		PCI_ANY_ID, PCI_ANY_ID, 0, 0,
		pbn_b0_2_xr_921600 },
	{	PCI_VENDOR_ID_ADVANTECH, PCI_DEVICE_ID_ADVANTECH_A823,
		PCI_ANY_ID, PCI_ANY_ID, 0, 0,
		pbn_b0_3_xr_921600  },
	{	PCI_VENDOR_ID_ADVANTECH, PCI_DEVICE_ID_ADVANTECH_A824,
		PCI_ANY_ID, PCI_ANY_ID, 0, 0,
		pbn_b0_4_xr_921600 },
	{	PCI_VENDOR_ID_ADVANTECH, PCI_DEVICE_ID_ADVANTECH_A828,
		PCI_ANY_ID, PCI_ANY_ID, 0, 0,
		pbn_b0_8_xr_921600 },
	{	PCI_VENDOR_ID_ADVANTECH, PCI_DEVICE_ID_ADVANTECH_A831,
		PCI_ANY_ID, PCI_ANY_ID, 0, 0,
		pbn_b0_1_xr_921600 },
	{	PCI_VENDOR_ID_ADVANTECH, PCI_DEVICE_ID_ADVANTECH_A832,
		PCI_ANY_ID, PCI_ANY_ID, 0, 0,
		pbn_b0_2_xr_921600 },
	{	PCI_VENDOR_ID_ADVANTECH, PCI_DEVICE_ID_ADVANTECH_A833,
		PCI_ANY_ID, PCI_ANY_ID, 0, 0,
		pbn_b0_3_xr_921600 },
	{	PCI_VENDOR_ID_ADVANTECH, PCI_DEVICE_ID_ADVANTECH_A834,
		PCI_ANY_ID, PCI_ANY_ID, 0, 0,
		pbn_b0_4_xr_921600 },
	{	PCI_VENDOR_ID_ADVANTECH, PCI_DEVICE_ID_ADVANTECH_A838,
		PCI_ANY_ID, PCI_ANY_ID, 0, 0,
		pbn_b0_8_xr_921600 },
	{ 0, }
};

static pci_ers_result_t serial8250_io_error_detected(struct pci_dev *dev,
						pci_channel_state_t state)
{
	struct serial_private *priv = pci_get_drvdata(dev);

	if (state == pci_channel_io_perm_failure)
		return PCI_ERS_RESULT_DISCONNECT;

	if (priv)
		adv_pciserial_detach_ports(priv);

	pci_disable_device(dev);

	return PCI_ERS_RESULT_NEED_RESET;
}

static pci_ers_result_t serial8250_io_slot_reset(struct pci_dev *dev)
{
	int rc;

	rc = pci_enable_device(dev);

	if (rc)
		return PCI_ERS_RESULT_DISCONNECT;

	pci_restore_state(dev);
	pci_save_state(dev);

	return PCI_ERS_RESULT_RECOVERED;
}

static void serial8250_io_resume(struct pci_dev *dev)
{
	struct serial_private *priv = pci_get_drvdata(dev);
	struct serial_private *new;

	if (!priv)
		return;

	new = pciserial_init_ports(dev, priv->board);
	if (!IS_ERR(new)) {
		pci_set_drvdata(dev, new);
		kfree(priv);
	}
}

static const struct pci_error_handlers serial8250_err_handler = {
	.error_detected = serial8250_io_error_detected,
	.slot_reset = serial8250_io_slot_reset,
	.resume = serial8250_io_resume,
};

static struct pci_driver serial_pci_driver = {
	.name		= "advserial",
	.probe		= pciserial_init_one,
	.remove		= pciserial_remove_one,
	.driver         = {
		.pm     = &pciserial_pm_ops,
	},
/*#ifdef CONFIG_PM
	.suspend	= pciserial_suspend_one,
	.resume		= pciserial_resume_one,
#endif*/
	.id_table	= serial_pci_tbl,
	.err_handler	= &serial8250_err_handler,
};

//module_pci_driver(serial_pci_driver);


static int __init adv_serial8250_pci_init(void)
{
	printk("\n");
	printk("==========================================================="
	       "====\n");
	printk("Advantech PCI-954/952/16C950 Device Drivers. V%s [%s]\n",
		ADVANTECH_16C950_VER, ADVANTECH_16C950_DATE);
 	printk("Supports: RS232/422/485 auto detection and setting\n");
 	printk("Devices:  UNO:  UNO2050 [COM3/COM4]\n");
 	printk("                UNO2059 [COM1~COM4]\n");
 	printk("                UNOB-2201CB [COM1~COM8]\n");
 	printk("                UNOB-2176 [COM1~COM4]\n");
	printk("                UNO-1150 [COM2/COM3]\n");
	printk("                UNO-2679 [COM3~COM6]\n");
	printk("                UNO-4672 [COM3~COM10]\n");
 	printk("          ICOM: PCI-1601, PCI-1602\n"
	       "                PCI-1603, PCI-1604\n"
	       "                PCI-1610, PCI-1611\n"
	       "                PCI-1612\n"
	       "                PCI-1620, PCI-1622\n");
 	printk("          MIC:  MIC-3611, MIC-3612\n");
 	printk("                MIC-3620, MIC-3621\n");
 	printk("          PCM:  PCM-3614P/I, PCM-3641P/I\n");
 	printk("                PCM-3618P/I, PCM-3681P/I\n");
	printk("      General:  A001, A002, A004\n");
	printk("                A101, A102, A104\n");
	printk("                F001, F002, F004\n");
	printk("                F101, F102, F104\n");
	printk("                A202, A304, A408\n");
	printk("Advantech Industrial Automation Group.\n");
	printk("==========================================================="
	       "====\n");
	if(adv_serial8250_init() >= 0)
		return pci_register_driver(&serial_pci_driver);
	return -ENODEV;
}

static void __exit adv_serial8250_pci_exit(void)
{
	pci_unregister_driver(&serial_pci_driver);
	adv_serial8250_exit();
}
EXPORT_SYMBOL_GPL(adv_pciserial_init_ports);

module_init(adv_serial8250_pci_init);
module_exit(adv_serial8250_pci_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Advantech IAG PCI-954/16C950 serial probe module");
MODULE_DEVICE_TABLE(pci, serial_pci_tbl);
