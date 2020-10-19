// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for NAL Research Corporation USB Serial converter.
 * Tested using NAL A3LA-XG.
 *
 * Copyright (C) 2020 Bertold Van den Bergh (vandenbergh@bertold.org)
 *
 */

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/module.h>
#include <linux/usb.h>
#include <linux/usb/serial.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>

static const struct usb_device_id id_table[] = {
	{ USB_DEVICE(0x2017, 0x0001) },
	{ }
};

MODULE_DEVICE_TABLE(usb, id_table);

struct nal_serial_private {
	struct usb_device       *dev;
	spinlock_t               lock;
	struct workqueue_struct *work_queue;
	struct mutex             cmd_mutex;
	unsigned char            cmd_buf[2];
	wait_queue_head_t        control_event;
	unsigned char            control_get;
	unsigned char            control_put;
	struct work_struct       control_work;
};

static int nal_prepare_write_buffer(struct usb_serial_port *port,
					void *buf, size_t count);
static void nal_process_read_urb(struct urb *urb);
static int nal_tiocmget(struct tty_struct *tty);
static int nal_send_control(struct nal_serial_private *priv,
			unsigned int set, unsigned int clear);
static int nal_tiocmset(struct tty_struct *tty,
		unsigned int set, unsigned int clear);
static void nal_dtr_rts(struct usb_serial_port *port, int on);
static void nal_control_work(struct work_struct *work);
static int nal_port_probe(struct usb_serial_port *serial);
static int nal_port_remove(struct usb_serial_port *serial);

static struct usb_serial_driver nal_device = {
	.driver = {
		.owner =	THIS_MODULE,
		.name =		"nal",
	},
	.id_table		= id_table,
	.num_ports		= 1,
	.tiocmget		= nal_tiocmget,
	.tiocmset		= nal_tiocmset,
	.port_probe		= nal_port_probe,
	.port_remove		= nal_port_remove,
	.dtr_rts		= nal_dtr_rts,
        .process_read_urb       = nal_process_read_urb,
        .prepare_write_buffer   = nal_prepare_write_buffer
};

static struct usb_serial_driver * const serial_drivers[] = {
	&nal_device, NULL
};

#define CONTROL_DSR (1)
#define CONTROL_CD  (2)
#define CONTROL_RI  (4)
#define CONTROL_CTS (8)
#define CONTROL_DTR (16)
#define CONTROL_RTS (32)

static int nal_prepare_write_buffer(struct usb_serial_port *port,
					void *buf, size_t count)
{
	unsigned char *header = (unsigned char*)buf;
	header[0] = 5;

	return kfifo_out_locked(&port->write_fifo, buf + 1,
		min_t(size_t, count, 63), &port->lock) + 1;
}

static void nal_process_read_urb(struct urb *urb)
{
	struct usb_serial_port *port = urb->context;
	struct nal_serial_private *priv = usb_get_serial_port_data(port);
	const unsigned char *buf = (unsigned char*)urb->transfer_buffer;

	if (urb->actual_length < 1)
		return;

	if (buf[0] == 5){
		tty_insert_flip_string(&port->port, buf + 1,
			urb->actual_length - 1);
		tty_flip_buffer_push(&port->port);
	} else if (buf[0] == 1){ 
		schedule_work(&priv->control_work);
	} else if (buf[0] == 0 && urb->actual_length >= 2) {
		spin_lock(&priv->lock);
		priv->control_get = 0x80 | buf[1];
		spin_unlock(&priv->lock);

		wake_up(&priv->control_event);
	}
}

static int nal_tiocmget(struct tty_struct *tty)
{
	struct usb_serial_port *port = tty->driver_data;
	struct nal_serial_private *priv = usb_get_serial_port_data(port);
        int retVal, control = 0;

	spin_lock(&priv->lock);
	priv->control_get = 0;
	spin_unlock(&priv->lock);

	mutex_lock(&priv->cmd_mutex);

	priv->cmd_buf[0] = 0x01;
	priv->cmd_buf[1] = 0xFF;

	retVal = usb_bulk_msg(priv->dev, usb_sndbulkpipe(priv->dev, 1),
		priv->cmd_buf, 2, NULL, HZ);

	mutex_unlock(&priv->cmd_mutex);

	if (retVal)
		return retVal;

	while(!control){
		retVal = wait_event_interruptible_timeout(priv->control_event,
				priv->control_get > 0, HZ);
		if (retVal == 0) 
			retVal = -ETIMEDOUT;
		if (retVal < 0)
			return retVal;

		spin_lock(&priv->lock);
		control = priv->control_get;
		spin_unlock(&priv->lock);
	}

	retVal = ((control & CONTROL_DSR) ? TIOCM_DSR : 0) |
		 ((control & CONTROL_CD)  ? TIOCM_CD  : 0) |
		 ((control & CONTROL_RI)  ? TIOCM_RI  : 0) |
		 ((control & CONTROL_CTS) ? TIOCM_CTS : 0);

	spin_lock(&priv->lock);
	control = priv->control_put;
	spin_unlock(&priv->lock);

	retVal |= ((control & CONTROL_DTR) ? TIOCM_DTR : 0) |
		  ((control & CONTROL_RTS) ? TIOCM_RTS : 0);

	return retVal;
}

static int nal_send_control(struct nal_serial_private *priv,
			unsigned int set, unsigned int clear){
	int retVal, control;

	spin_lock(&priv->lock);
	if (set & TIOCM_RTS) {
		priv->control_put |= CONTROL_RTS;
	}
	if (set & TIOCM_DTR) {
		priv->control_put |= CONTROL_DTR;
	}
	if (clear & TIOCM_RTS) {
		priv->control_put &= ~CONTROL_RTS;
	}
	if (clear & TIOCM_DTR) {
		priv->control_put &= ~CONTROL_DTR;
	}

	control = priv->control_put;
	spin_unlock(&priv->lock);

	mutex_lock(&priv->cmd_mutex);

	priv->cmd_buf[0] = 0x00;
	priv->cmd_buf[1] = 0x0d | control;

	retVal = usb_bulk_msg(priv->dev, usb_sndbulkpipe(priv->dev, 1),
		priv->cmd_buf, 2, NULL, HZ);

	mutex_unlock(&priv->cmd_mutex);

	return retVal;
}

static int nal_tiocmset(struct tty_struct *tty,
		unsigned int set, unsigned int clear)
{
	struct usb_serial_port *port = tty->driver_data;
	struct nal_serial_private *priv = usb_get_serial_port_data(port);

	return nal_send_control(priv, set, clear);
}

static void nal_dtr_rts(struct usb_serial_port *port, int enable)
{
	struct nal_serial_private *priv = usb_get_serial_port_data(port);

	if (enable)
		nal_send_control(priv, TIOCM_RTS | TIOCM_DTR, 0);
	else
		nal_send_control(priv, 0, TIOCM_RTS | TIOCM_DTR);
}

static void nal_control_work(struct work_struct *work)
{
	struct nal_serial_private *priv = 
		container_of(work, struct nal_serial_private, control_work);

	nal_send_control(priv, 0, 0);
}

static int nal_port_probe(struct usb_serial_port *serial)
{
	struct nal_serial_private *priv;
	int retVal;

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->dev = serial->serial->dev;
	spin_lock_init(&priv->lock);
	init_waitqueue_head(&priv->control_event);
	mutex_init(&priv->cmd_mutex);

	priv->work_queue = alloc_workqueue("nal_wq", 0, 0);
	if (!priv->work_queue) {
		retVal = -ENOMEM;
		goto fail_queue;
	}

	INIT_WORK(&priv->control_work, nal_control_work);

	usb_set_serial_port_data(serial, priv);
	
	retVal = nal_send_control(priv, TIOCM_RTS | TIOCM_DTR, 0);
	if (retVal < 0)
		goto fail_probe;

	return 0;

fail_probe:
	cancel_work_sync(&priv->control_work);
	destroy_workqueue(priv->work_queue);
fail_queue:
	kfree(priv);
	return retVal;
}

static int nal_port_remove(struct usb_serial_port *serial)
{
	struct nal_serial_private *priv = usb_get_serial_port_data(serial);

	cancel_work_sync(&priv->control_work);
	destroy_workqueue(priv->work_queue);
	kfree(priv);

	return 0;
}

module_usb_serial_driver(serial_drivers, id_table);

#define AUTHOR "Bertold Van den Bergh <vandenbergh@bertold.org>"
#define DESC   "Driver for NAL Research Corporation USB serial interface"
MODULE_DESCRIPTION(DESC);
MODULE_LICENSE("GPL v2");
