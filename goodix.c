/*
 *  driver for Goodix Touchscreens
 *
 *  Copyright (c) 2014 Red Hat Inc.
 *
 *  This code is based on gt9xx.c authored by andrew@goodix.com:
 *
 *  2010 - 2012 Goodix Technology.
 */

/*
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; version 2 of the License.
 */

#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <asm/unaligned.h>

#include <asm/io.h>
#include <linux/gpio.h>
#include <mach/irqs.h>
#include <mach/system.h>
#include <mach/hardware.h>
#include <plat/sys_config.h>

struct goodix_ts_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	int abs_x_max;
	int abs_y_max;
	unsigned int max_touch_num;
	unsigned int int_trigger_type;
};

#define GOODIX_CHANGE_X2Y		1

#define GOODIX_MAX_HEIGHT		4096
#define GOODIX_MAX_WIDTH		4096
#define GOODIX_INT_TRIGGER		1
#define GOODIX_CONTACT_SIZE		8
#define GOODIX_MAX_CONTACTS		10

#define GOODIX_CONFIG_MAX_LENGTH	240

/* Register defines */
#define GOODIX_READ_COOR_ADDR		0x814E
#define GOODIX_REG_CONFIG_DATA		0x8047
#define GOODIX_REG_VERSION		0x8140

#define RESOLUTION_LOC		1
#define TRIGGER_LOC		6

/* Base defines for gpio irq */
#define PIO_BASE_ADDRESS	(0x01c20800)
#define PIO_RANGE_SIZE		(0x400)

#define PIO_INT_STAT_OFFSET	(0x214)
#define PIO_INT_CTRL_OFFSET	(0x210)

typedef enum {
     PIO_INT_CFG0_OFFSET = 0x200,
     PIO_INT_CFG1_OFFSET = 0x204,
     PIO_INT_CFG2_OFFSET = 0x208,
     PIO_INT_CFG3_OFFSET = 0x20c,
} int_cfg_offset;

typedef enum {
	POSITIVE_EDGE = 0x0,
	NEGATIVE_EDGE = 0x1,
	HIGH_LEVEL = 0x2,
	LOW_LEVEL = 0x3,
	DOUBLE_EDGE = 0x4
} ext_int_mode;

#define CTP_IRQ_NO		(gpio_int_info[0].port_num)
#define CTP_IRQ_MODE		(NEGATIVE_EDGE)

static void * __iomem gpio_addr = NULL;
static int gpio_int_hdle = 0;
static user_gpio_set_t gpio_int_info[1];
static int int_cfg_addr[] = { PIO_INT_CFG0_OFFSET, \
			      PIO_INT_CFG1_OFFSET, \
			      PIO_INT_CFG2_OFFSET, \
			      PIO_INT_CFG3_OFFSET };

/* Addresses to scan */
static __u32 twi_id;
static const unsigned short normal_i2c[2] = {0x5d, I2C_CLIENT_END};

/**
 * goodix_i2c_read - read data from a register of the i2c slave device.
 *
 * @client: i2c device.
 * @reg: the register to read from.
 * @buf: raw write data buffer.
 * @len: length of the buffer to write
 */
static int goodix_i2c_read(struct i2c_client *client,
				u16 reg, u8 *buf, int len)
{
	struct i2c_msg msgs[2];
	u16 wbuf = cpu_to_be16(reg);
	int ret;

	msgs[0].flags = 0;
	msgs[0].addr  = client->addr;
	msgs[0].len   = 2;
	msgs[0].buf   = (u8 *) &wbuf;

	msgs[1].flags = I2C_M_RD;
	msgs[1].addr  = client->addr;
	msgs[1].len   = len;
	msgs[1].buf   = buf;

	ret = i2c_transfer(client->adapter, msgs, 2);
	return ret < 0 ? ret : (ret != ARRAY_SIZE(msgs) ? -EIO : 0);
}

static int goodix_ts_read_input_report(struct goodix_ts_data *ts, u8 *data)
{
	int touch_num;
	int error;

	error = goodix_i2c_read(ts->client, GOODIX_READ_COOR_ADDR, data,
				GOODIX_CONTACT_SIZE + 1);
	if (error) {
		dev_err(&ts->client->dev, "I2C transfer error: %d\n", error);
		return error;
	}

	touch_num = data[0] & 0x0f;
	if (touch_num > GOODIX_MAX_CONTACTS)
		return -EPROTO;

	if (touch_num > 1) {
		data += 1 + GOODIX_CONTACT_SIZE;
		error = goodix_i2c_read(ts->client,
					GOODIX_READ_COOR_ADDR +
						1 + GOODIX_CONTACT_SIZE,
					data,
					GOODIX_CONTACT_SIZE * (touch_num - 1));
		if (error)
			return error;
	}

	return touch_num;
}

static void goodix_ts_report_touch(struct goodix_ts_data *ts, u8 *coor_data)
{
	int id = coor_data[0] & 0x0F;
	int input_x = get_unaligned_le16(&coor_data[1]);
	int input_y = get_unaligned_le16(&coor_data[3]);
	int input_w = get_unaligned_le16(&coor_data[5]);

	#if GOODIX_CHANGE_X2Y
		swap(input_x, input_y);
	#endif

	input_mt_slot(ts->input_dev, id);
	input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, true);
	input_report_abs(ts->input_dev, ABS_MT_POSITION_X, input_x);
	input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, input_y);
	input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, input_w);
	input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, input_w);
}

/**
 * goodix_process_events - Process incoming events
 *
 * @ts: our goodix_ts_data pointer
 *
 * Called when the IRQ is triggered. Read the current device state, and push
 * the input events to the user space.
 */
static void goodix_process_events(struct goodix_ts_data *ts)
{
	u8  point_data[1 + GOODIX_CONTACT_SIZE * GOODIX_MAX_CONTACTS];
	int touch_num;
	int i;

	touch_num = goodix_ts_read_input_report(ts, point_data);
	if (touch_num < 0)
		return;

	for (i = 0; i < touch_num; i++)
		goodix_ts_report_touch(ts,
				&point_data[1 + GOODIX_CONTACT_SIZE * i]);

	/* input_mt_sync_frame(ts->input_dev); */
	input_sync(ts->input_dev);
}

/**
 * goodix_ts_irq_handler - The IRQ handler
 *
 * @irq: interrupt number.
 * @dev_id: private data pointer.
 */
static irqreturn_t goodix_ts_irq_handler(int irq, void *dev_id)
{
	static const u8 end_cmd[] = {
		GOODIX_READ_COOR_ADDR >> 8,
		GOODIX_READ_COOR_ADDR & 0xff,
		0
	};
	struct goodix_ts_data *ts = dev_id;

	goodix_process_events(ts);

	/* Clear the IRQ_EINT21 interrupt pending */
	reg_val = readl(gpio_addr + PIO_INT_STAT_OFFSET);

	if (reg_val&(1<<(CTP_IRQ_NO)))
		writel(reg_val&(1<<(CTP_IRQ_NO)), \
			gpio_addr + PIO_INT_STAT_OFFSET);
	else
		return IRQ_NONE;

	if (i2c_master_send(ts->client, end_cmd, sizeof(end_cmd)) < 0)
		dev_err(&ts->client->dev, "I2C write end_cmd error\n");

	return IRQ_HANDLED;
}

/**
 * goodix_clear_penirq - Clear int pending
 *
 */
static void goodix_clear_penirq(void)
{
	int reg_val;
	reg_val = readl(gpio_addr + PIO_INT_STAT_OFFSET);
	if (reg_val = (reg_val&(1<<(CTP_IRQ_NO))))
		writel(reg_val, gpio_addr + PIO_INT_STAT_OFFSET);
	return;
}

/**
 * goodix_set_irq_mode - Configure irq to int port.
 *
 * @ts: our goodix_ts_data pointer
 * @major_key: section key
 * @subkey: section subkey
 * @int_mode: int mode
 *
 * Must be called during probe
 */
static int goodix_set_irq_mode(struct goodix_ts_data *ts, \
				char *major_key, char *subkey, \
				ext_int_mode int_mode)
{
	__u32 reg_num = 0;
	__u32 reg_addr = 0;
	__u32 reg_val = 0;

	dev_info(&ts->client->dev, "Config gpio to int mode.");

	if (gpio_int_hdle)
		gpio_release(gpio_int_hdle, 2);

	gpio_int_hdle = gpio_request_ex(major_key, subkey);
	if (!gpio_int_hdle) {
		dev_err(&ts->client->dev, "Request ctp_int_port failed.\n");
		return -1;
	}
 
	gpio_get_one_pin_status(gpio_int_hdle, gpio_int_info, subkey, 1);

	reg_num = (gpio_int_info[0].port_num)%8;
	reg_addr = (gpio_int_info[0].port_num)/8;
	reg_val = readl(gpio_addr + int_cfg_addr[reg_addr]);
	reg_val &= (~(7 << (reg_num * 4)));
	reg_val |= (int_mode << (reg_num * 4));
	writel(reg_val, gpio_addr + int_cfg_addr[reg_addr]);

	goodix_clear_penirq();

	reg_val = readl(gpio_addr + PIO_INT_CTRL_OFFSET);
	reg_val |= (1 << (gpio_int_info[0].port_num));
	writel(reg_val, gpio_addr + PIO_INT_CTRL_OFFSET);

	return 0;
}

/**
 * goodix_read_config - Read the embedded configuration of the panel
 *
 * @ts: our goodix_ts_data pointer
 *
 * Must be called during probe
 */
static void goodix_read_config(struct goodix_ts_data *ts)
{
	u8 config[GOODIX_CONFIG_MAX_LENGTH];
	int error;

	error = goodix_i2c_read(ts->client, GOODIX_REG_CONFIG_DATA,
			      config,
			   GOODIX_CONFIG_MAX_LENGTH);
	if (error) {
		dev_warn(&ts->client->dev,
			 "Error reading config (%d), using defaults\n",
			 error);
		ts->abs_x_max = GOODIX_MAX_WIDTH;
		ts->abs_y_max = GOODIX_MAX_HEIGHT;
		ts->int_trigger_type = GOODIX_INT_TRIGGER;
		return;
	}

	ts->abs_x_max = get_unaligned_le16(&config[RESOLUTION_LOC]);
	ts->abs_y_max = get_unaligned_le16(&config[RESOLUTION_LOC + 2]);
	ts->int_trigger_type = (config[TRIGGER_LOC]) & 0x03;
	if (!ts->abs_x_max || !ts->abs_y_max) {
		dev_err(&ts->client->dev,
			"Invalid config, using defaults\n");
		ts->abs_x_max = GOODIX_MAX_WIDTH;
		ts->abs_y_max = GOODIX_MAX_HEIGHT;
	}
}


/**
 * goodix_read_version - Read goodix touchscreen version
 *
 * @client: the i2c client
 * @version: output buffer containing the version on success
 */
static int goodix_read_version(struct i2c_client *client, u16 *version)
{
	int error;
	u8 buf[6];

	error = goodix_i2c_read(client, GOODIX_REG_VERSION, buf, sizeof(buf));
	if (error) {
		dev_err(&client->dev, "read version failed: %d\n", error);
		return error;
	}

	if (version)
		*version = get_unaligned_le16(&buf[4]);

	dev_info(&client->dev, "IC VERSION: %6ph\n", buf);

	return 0;
}

/**
 * goodix_i2c_test - I2C test function to check if the device answers.
 *
 * @client: the i2c client
 */
static int goodix_i2c_test(struct i2c_client *client)
{
	int retry = 0;
	int error;
	u8 test;

	while (retry++ < 2) {
		error = goodix_i2c_read(client, GOODIX_REG_CONFIG_DATA,
					&test, 1);
		if (!error)
			return 0;

		dev_err(&client->dev, "i2c test failed attempt %d: %d\n",
			retry, error);
		msleep(20);
	}

	return error;
}

/**
 * goodix_request_input_dev - Allocate, populate and register the input device
 *
 * @ts: our goodix_ts_data pointer
 *
 * Must be called during probe
 */
static int goodix_request_input_dev(struct goodix_ts_data *ts)
{
	int error;

	ts->input_dev = input_allocate_device();
	if (!ts->input_dev) {
		dev_err(&ts->client->dev, "Failed to allocate input device.");
		return -ENOMEM;
	}

	ts->input_dev->evbit[0] = BIT_MASK(EV_SYN) |
				  BIT_MASK(EV_KEY) |
				  BIT_MASK(EV_ABS);

	#if GOODIX_CHANGE_X2Y
		swap(ts->abs_x_max, ts->abs_y_max);
	#endif

	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0,
				ts->abs_x_max, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0,
				ts->abs_y_max, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);

	input_mt_init_slots(ts->input_dev, GOODIX_MAX_CONTACTS);

	ts->input_dev->name = "Goodix Capacitive TouchScreen";
	ts->input_dev->phys = "input/ts";
	ts->input_dev->id.bustype = BUS_I2C;
	ts->input_dev->id.vendor = 0x0416;
	ts->input_dev->id.product = 0x1001;
	ts->input_dev->id.version = 10427;

	error = input_register_device(ts->input_dev);
	if (error) {
		dev_err(&ts->client->dev,
			"Failed to register input device: %d", error);
		return error;
	}

	return 0;
}

/**
 * goodix_fetch_sysconfig_para - get config info from sysconfig.fex file.
 * return value:
 *	= 0: success;
 *	< 0: err;
 */
static int goodix_fetch_sysconfig_para(void)
{
	int ret = -1;
	int ctp_used = 0;
	char name[I2C_NAME_SIZE];
	int screen_max_x = 0;
	int screen_max_y = 0;
	script_parser_value_type_t type = SCRIPT_PARSER_VALUE_TYPE_STRING;

	if (SCRIPT_PARSER_OK != script_parser_fetch("ctp_para", "ctp_used", &ctp_used, 1)) {
		printk(KERN_ERR "*** ctp_used set to 0!\n");
		printk(KERN_ERR "*** If use ctp, please set ctp_used to 1.\n");
		return ret;
	}

	if (SCRIPT_PARSER_OK != script_parser_fetch_ex("ctp_para", "ctp_name", (int *)(&name), &type, sizeof(name)/sizeof(int))) {
		printk(KERN_ERR "Failed to fetch ctp_name.\n");
		return ret;
	}
	printk(KERN_INFO "ctp_name is %s.\n", name);

	if (strcmp(GOODIX_CTP_NAME, name)) {
		printk(KERN_ERR "Name %s does not match GOODIX_CTP_NAME.\n", name);
		return ret;
	}

	if (SCRIPT_PARSER_OK != script_parser_fetch("ctp_para", "ctp_twi_id", &twi_id, sizeof(twi_id)/sizeof(__u32))) {
		printk(KERN_ERR "Failed to fetch ctp_twi_id.\n");
		return ret;
	}
	printk(KERN_INFO "ctp_twi_id is %d.\n", twi_id);

	if (SCRIPT_PARSER_OK != script_parser_fetch("ctp_para", "ctp_twi_addr", &twi_addr, sizeof(twi_addr)/sizeof(__u32))) {
		printk(KERN_ERR "Failed to fetch ctp_twi_addr.\n");
		return ret;
	}
	printk(KERN_INFO "ctp_twi_addr is 0x%hx.\n", twi_addr);

	normal_i2c[0] = twi_addr;
	normal_i2c[1] = I2C_CLIENT_END;

	if (SCRIPT_PARSER_OK != script_parser_fetch("ctp_para", "ctp_screen_max_x", &screen_max_x, 1)) {
		printk(KERN_ERR "Failed to fetch ctp_screen_max_x.\n");
		return ret;
	}
	printk(KERN_INFO "screen_max_x = %d.\n", screen_max_x);

	if (SCRIPT_PARSER_OK != script_parser_fetch("ctp_para", "ctp_screen_max_y", &screen_max_y, 1)) {
		printk(KERN_ERR "Failed to fetch ctp_screen_max_y.\n");
		return ret;
	}
	printk(KERN_INFO "screen_max_y = %d.\n", screen_max_y);

	return 0;
}


static int goodix_ts_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	struct goodix_ts_data *ts;
	unsigned long irq_flags;
	int error;
	u16 version_info;

	dev_dbg(&client->dev, "I2C Address: 0x%02x\n", client->addr);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "I2C check functionality failed.\n");
		return -ENXIO;
	}

	ts = kzalloc(sizeof(*ts), GFP_KERNEL);
	if (!ts)
		return -ENOMEM;

	ts->client = client;
	i2c_set_clientdata(client, ts);

	error = goodix_i2c_test(client);
	if (error) {
		dev_err(&client->dev, "I2C communication failure: %d\n", error);
		return error;
	}

	error = goodix_read_version(client, &version_info);
	if (error) {
		dev_err(&client->dev, "Read version failed.\n");
		return error;
	}

	goodix_read_config(ts);

	error = goodix_request_input_dev(ts);
	if (error)
		return error;

	gpio_addr = ioremap(PIO_BASE_ADDRESS, PIO_RANGE_SIZE);
	if (!gpio_addr)
		return -EIO;

	error = goodix_set_irq_mode(ts, "ctp_para", "ctp_int_port", \
				    CTP_IRQ_MODE);
	if (error < 0) {
		dev_err(&ts->client->dev, "Set irq mode failed.");
		enable_irq(SW_INT_IRQNO_PIO);
	}

	error = request_threaded_irq(SW_INT_IRQNO_PIO, NULL, \
				     goodix_ts_irq_handler, \
				     IRQF_TRIGGER_RISING | IRQF_ONESHOT, \
				     client->name, ts);
	if (error) {
		dev_err(&client->dev, "request IRQ failed: %d.\n", error);
		return error;
	}

	return 0;
}

static int goodix_ts_remove(struct i2c_client *client)
{
	struct goodix_ts_data *ts = i2c_get_clientdata(client);

	free_irq(SW_INT_IRQNO_PIO, ts);
	input_unregister_device(ts->input_dev);
	i2c_set_clientdata(client, NULL);
	kfree(ts);

	if (gpio_addr)
		iounmap(gpio_addr);

	if (gpio_int_hdle)
		gpio_release(gpio_int_hdle, 2);
 
 	return 0;
}

static const struct i2c_device_id goodix_ts_id[] = {
	{ "GDIX1001:00", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, goodix_ts_id);

static struct i2c_driver goodix_ts_driver = {
	.class = I2C_CLASS_HWMON,
	.probe = goodix_ts_probe,
	.remove = goodix_ts_remove,
	.id_table = goodix_ts_id,
	.driver = {
		.name = "Goodix-TS",
		.owner = THIS_MODULE,
	},
	.address_list = normal_i2c,
};
module_i2c_driver(goodix_ts_driver);

MODULE_AUTHOR("Benjamin Tissoires <benjamin.tissoires@gmail.com>");
MODULE_AUTHOR("Bastien Nocera <hadess@hadess.net>");
MODULE_DESCRIPTION("Goodix touchscreen driver");
MODULE_LICENSE("GPL v2");
