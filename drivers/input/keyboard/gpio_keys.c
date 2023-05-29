/*
 * Driver for keys on GPIO lines capable of generating interrupts.
 *
 * Copyright 2005 Phil Blundell
 * Copyright 2010, 2011 David Jander <david@protonic.nl>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>

#include <linux/init.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/sched.h>
#include <linux/pm.h>
#include <linux/slab.h>
#include <linux/sysctl.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/gpio_keys.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/spinlock.h>
#include <dt-bindings/input/gpio-keys.h>
#include <linux/hall_ic.h>

#ifdef CONFIG_LGE_HANDLE_PANIC
#include <soc/qcom/lge/lge_handle_panic.h>
#endif


#if defined(CONFIG_LGE_DUAL_SCREEN)
#include <linux/lge_ds3.h>
#include <soc/qcom/lge/board_lge.h>
extern bool lge_get_mfts_mode(void);
extern bool lge_get_factory_boot(void);
#endif

#ifdef CONFIG_LGE_TOUCH_CORE_SUB
extern void touch_notify_swivel(u32 type);
extern void touch_sub_notify_swivel(u32 type);
#endif

#if defined(CONFIG_LGE_DUAL_SCREEN)
extern int lge_get_dual_display_support(void);
struct hallic_dev luke_sdev = {
       .name = "coverdisplay",
};
struct hallic_dev cover_fw_dev = {
       .name = "cover_fw",
       .state = -1,
};
struct hallic_dev cover_recovery = {
	.name = "cover_recovery",
	.state = 0,
};
int ready_dd_on;
#endif

#define CONFIG_LGE_SUPPORT_HALLIC
#ifdef CONFIG_LGE_SUPPORT_HALLIC
#if IS_ENABLED(CONFIG_LGE_SWIVEL_HALLIC_SUPPORT)
/* This dev is only used for saving gpio state.
 * State transition is passed via Key event(SW_LID).
 * So it doesn't need to register as hallic dev.
 */
struct hallic_dev swivel_dev = {
	.name = "swivel",
	.state = 0,
	.state_front = 0,
	.state_back = 0,
};
#else
struct hallic_dev sdev = {
	.name = "smartcover",
	.state = 0,
	.state_front = 0,
	.state_back = 0,
};

struct hallic_dev ndev = {
	.name = "nfccover",
	.state = 0,
};
#endif
#endif

struct gpio_button_data {
	const struct gpio_keys_button *button;
	struct input_dev *input;
	struct gpio_desc *gpiod;

	unsigned short *code;

	struct timer_list release_timer;
	unsigned int release_delay;	/* in msecs, for IRQ-only buttons */

	struct delayed_work work;
	unsigned int software_debounce;	/* in msecs, for GPIO-driven buttons */

	unsigned int irq;
	unsigned int wakeup_trigger_type;
	spinlock_t lock;
	bool disabled;
	bool key_pressed;
	bool suspended;
};

struct gpio_keys_drvdata {
	const struct gpio_keys_platform_data *pdata;
	struct input_dev *input;
	struct mutex disable_lock;
	unsigned short *keymap;
	struct gpio_button_data data[0];
};

/*
 * SYSFS interface for enabling/disabling keys and switches:
 *
 * There are 4 attributes under /sys/devices/platform/gpio-keys/
 *	keys [ro]              - bitmap of keys (EV_KEY) which can be
 *	                         disabled
 *	switches [ro]          - bitmap of switches (EV_SW) which can be
 *	                         disabled
 *	disabled_keys [rw]     - bitmap of keys currently disabled
 *	disabled_switches [rw] - bitmap of switches currently disabled
 *
 * Userland can change these values and hence disable event generation
 * for each key (or switch). Disabling a key means its interrupt line
 * is disabled.
 *
 * For example, if we have following switches set up as gpio-keys:
 *	SW_DOCK = 5
 *	SW_CAMERA_LENS_COVER = 9
 *	SW_KEYPAD_SLIDE = 10
 *	SW_FRONT_PROXIMITY = 11
 * This is read from switches:
 *	11-9,5
 * Next we want to disable proximity (11) and dock (5), we write:
 *	11,5
 * to file disabled_switches. Now proximity and dock IRQs are disabled.
 * This can be verified by reading the file disabled_switches:
 *	11,5
 * If we now want to enable proximity (11) switch we write:
 *	5
 * to disabled_switches.
 *
 * We can disable only those keys which don't allow sharing the irq.
 */

/**
 * get_n_events_by_type() - returns maximum number of events per @type
 * @type: type of button (%EV_KEY, %EV_SW)
 *
 * Return value of this function can be used to allocate bitmap
 * large enough to hold all bits for given type.
 */
static int get_n_events_by_type(int type)
{
	BUG_ON(type != EV_SW && type != EV_KEY);

	return (type == EV_KEY) ? KEY_CNT : SW_CNT;
}

/**
 * get_bm_events_by_type() - returns bitmap of supported events per @type
 * @input: input device from which bitmap is retrieved
 * @type: type of button (%EV_KEY, %EV_SW)
 *
 * Return value of this function can be used to allocate bitmap
 * large enough to hold all bits for given type.
 */
static const unsigned long *get_bm_events_by_type(struct input_dev *dev,
						  int type)
{
	BUG_ON(type != EV_SW && type != EV_KEY);

	return (type == EV_KEY) ? dev->keybit : dev->swbit;
}

/**
 * gpio_keys_disable_button() - disables given GPIO button
 * @bdata: button data for button to be disabled
 *
 * Disables button pointed by @bdata. This is done by masking
 * IRQ line. After this function is called, button won't generate
 * input events anymore. Note that one can only disable buttons
 * that don't share IRQs.
 *
 * Make sure that @bdata->disable_lock is locked when entering
 * this function to avoid races when concurrent threads are
 * disabling buttons at the same time.
 */
static void gpio_keys_disable_button(struct gpio_button_data *bdata)
{
	if (!bdata->disabled) {
		/*
		 * Disable IRQ and associated timer/work structure.
		 */
		disable_irq(bdata->irq);

		if (bdata->gpiod)
			cancel_delayed_work_sync(&bdata->work);
		else
			del_timer_sync(&bdata->release_timer);

		bdata->disabled = true;
	}
}

/**
 * gpio_keys_enable_button() - enables given GPIO button
 * @bdata: button data for button to be disabled
 *
 * Enables given button pointed by @bdata.
 *
 * Make sure that @bdata->disable_lock is locked when entering
 * this function to avoid races with concurrent threads trying
 * to enable the same button at the same time.
 */
static void gpio_keys_enable_button(struct gpio_button_data *bdata)
{
	if (bdata->disabled) {
		enable_irq(bdata->irq);
		bdata->disabled = false;
	}
}

#if defined(CONFIG_LGE_DUAL_SCREEN)
static void send_recovery_event(int num)
{
	if (!cover_recovery.state) {
		pr_info("[DD] %s\n", __func__);
		hallic_set_state(&cover_recovery, num);
	}
}
#endif

#if defined(CONFIG_LGE_DUAL_SCREEN)
void request_dualscreen_recovery(void)
{
	pr_info("[DD] %s\n", __func__);
	send_recovery_event(1);
	cover_recovery.state = 0;
}
EXPORT_SYMBOL(request_dualscreen_recovery);
#endif
#if defined(CONFIG_LGE_DUAL_SCREEN)
static ssize_t virtual_mcu_firmware_write_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count){
	unsigned long val = simple_strtoul(buf, NULL, 10);

	pr_info("[MCU_FW] virtual_mcu_firmware_write set to [%d]\n", (int)val);

	switch ((int)val) {
	case DD_MCU_FORCE_UPDATE:
		hallic_set_state(&cover_fw_dev, UPDATE_NO_NEED);
		mdelay(100);
		hallic_set_state(&cover_fw_dev, UPDATE_NEED);
		pr_info("[MCU_FW][%s] cover_fw set to %d\n", __func__, UPDATE_NEED);
		break;
	case DD_MCU_CANCEL:
		hallic_set_state(&cover_fw_dev, UPDATE_NO_NEED);
		pr_info("[MCU_FW][%s] cover_fw set to %d\n", __func__, UPDATE_NO_NEED);
		break;
	case DD_MCU_RECOVERY:
		hallic_set_state(&cover_fw_dev, UPDATE_NO_NEED);
		mdelay(100);
		hallic_set_state(&cover_fw_dev, RECOVERY_NEED);
		pr_info("[MCU_FW][%s] cover_fw set to %d\n", __func__, RECOVERY_NEED);
	}

	return count;
}
#endif
#if defined(CONFIG_LGE_DUAL_SCREEN)
static ssize_t cover_recovery_req_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count){
	int delay;
	ssize_t ret = strnlen(buf, PAGE_SIZE);

	sscanf(buf, "%d", &delay);
	pr_err("%s : %d\n", __func__, delay);

	send_recovery_event(1);
	cover_recovery.state = 0;       //FIXME : This value is set if power on control receivced from framework in DS1.

    return ret;
}
#endif

#if IS_ENABLED(CONFIG_LGE_SWIVEL_HALLIC_SUPPORT)
static ssize_t swivel_event_refresh_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count){
	int i, current_state;
	ssize_t ret = strnlen(buf, PAGE_SIZE);
	struct platform_device *pdev = to_platform_device(dev);
	struct gpio_keys_drvdata *ddata = platform_get_drvdata(pdev);

	for (i = 0; i < ddata->pdata->nbuttons; i++) {
		struct gpio_button_data *bdata = &ddata->data[i];
		struct input_dev *input = bdata->input;

		if (!strncmp(bdata->button->desc, "swivel_end", 10) || !strncmp(bdata->button->desc, "swivel_start", 12)) {
			current_state = swivel_dev.state;
			if (swivel_dev.state) {
				hallic_set_state(&swivel_dev, 0);
				hallic_set_state(&swivel_dev, current_state);
				input_event(input, EV_SW, *bdata->code, 0);
				input_event(input, EV_SW, *bdata->code, current_state);
			} else {
				hallic_set_state(&swivel_dev, 1);
				hallic_set_state(&swivel_dev, current_state);
				input_event(input, EV_SW, *bdata->code, 1);
				input_event(input, EV_SW, *bdata->code, current_state);
			}
			pr_info("%s : code(%d), value(%d)\n", __func__, bdata->button->code, swivel_dev.state);
			input_sync(input);
			break;
		}
	}

	return ret;
}

static ssize_t swivel_event_injector_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count){
	int event, i;
	ssize_t ret = strnlen(buf, PAGE_SIZE);
	struct platform_device *pdev = to_platform_device(dev);
	struct gpio_keys_drvdata *ddata = platform_get_drvdata(pdev);

	sscanf(buf, "%d", &event);

	if (event < SWIVEL_HALF_OPENED || event > SWIVEL_OPENED) {
		pr_err("%s : Wrong input for swivel. evnet should be between 0 and 2.\n", __func__);
		return -EINVAL;
	}

	for (i = 0; i < ddata->pdata->nbuttons; i++) {
		struct gpio_button_data *bdata = &ddata->data[i];
		struct input_dev *input = bdata->input;

		if (!strncmp(bdata->button->desc, "swivel_end", 10) || !strncmp(bdata->button->desc, "swivel_start", 12)) {
			if (swivel_dev.state != event) {
				hallic_set_state(&swivel_dev, event);
				swivel_dev.state = event;
				input_event(input, EV_SW, *bdata->code, event);
				pr_info("%s : code(%d), value(%d)\n", __func__, bdata->button->code, event);
				input_sync(input);
			}
			break;
		}
	}

	return ret;
}

static ssize_t swivel_event_injector_show(struct device *dev, struct device_attribute *attr, char *buf){
	struct platform_device *pdev = to_platform_device(dev);
	struct gpio_keys_drvdata *ddata = platform_get_drvdata(pdev);
	int swivel_state, i;

	for (i = 0; i < ddata->pdata->nbuttons; i++) {
		struct gpio_button_data *bdata = &ddata->data[i];

		if (!strncmp(bdata->button->desc, "swivel_end", 10) || !strncmp(bdata->button->desc, "swivel_start", 12)) {
			swivel_state = swivel_dev.state;
			break;
		}
	}

	if (i == ddata->pdata->nbuttons)
		goto error;

	return sprintf(buf, "%d\n", swivel_state);

error:
	pr_err("swivel gpio_key is not registered.\n");
	return -ENODEV;
}
#endif
/**
 * gpio_keys_attr_show_helper() - fill in stringified bitmap of buttons
 * @ddata: pointer to drvdata
 * @buf: buffer where stringified bitmap is written
 * @type: button type (%EV_KEY, %EV_SW)
 * @only_disabled: does caller want only those buttons that are
 *                 currently disabled or all buttons that can be
 *                 disabled
 *
 * This function writes buttons that can be disabled to @buf. If
 * @only_disabled is true, then @buf contains only those buttons
 * that are currently disabled. Returns 0 on success or negative
 * errno on failure.
 */
static ssize_t gpio_keys_attr_show_helper(struct gpio_keys_drvdata *ddata,
					  char *buf, unsigned int type,
					  bool only_disabled)
{
	int n_events = get_n_events_by_type(type);
	unsigned long *bits;
	ssize_t ret;
	int i;

	bits = bitmap_zalloc(n_events, GFP_KERNEL);
	if (!bits)
		return -ENOMEM;

	for (i = 0; i < ddata->pdata->nbuttons; i++) {
		struct gpio_button_data *bdata = &ddata->data[i];

		if (bdata->button->type != type)
			continue;

		if (only_disabled && !bdata->disabled)
			continue;

		__set_bit(*bdata->code, bits);
	}

	ret = scnprintf(buf, PAGE_SIZE - 1, "%*pbl", n_events, bits);
	buf[ret++] = '\n';
	buf[ret] = '\0';

	bitmap_free(bits);

	return ret;
}

/**
 * gpio_keys_attr_store_helper() - enable/disable buttons based on given bitmap
 * @ddata: pointer to drvdata
 * @buf: buffer from userspace that contains stringified bitmap
 * @type: button type (%EV_KEY, %EV_SW)
 *
 * This function parses stringified bitmap from @buf and disables/enables
 * GPIO buttons accordingly. Returns 0 on success and negative error
 * on failure.
 */
static ssize_t gpio_keys_attr_store_helper(struct gpio_keys_drvdata *ddata,
					   const char *buf, unsigned int type)
{
	int n_events = get_n_events_by_type(type);
	const unsigned long *bitmap = get_bm_events_by_type(ddata->input, type);
	unsigned long *bits;
	ssize_t error;
	int i;

	bits = bitmap_zalloc(n_events, GFP_KERNEL);
	if (!bits)
		return -ENOMEM;

	error = bitmap_parselist(buf, bits, n_events);
	if (error)
		goto out;

	/* First validate */
	if (!bitmap_subset(bits, bitmap, n_events)) {
		error = -EINVAL;
		goto out;
	}

	for (i = 0; i < ddata->pdata->nbuttons; i++) {
		struct gpio_button_data *bdata = &ddata->data[i];

		if (bdata->button->type != type)
			continue;

		if (test_bit(*bdata->code, bits) &&
		    !bdata->button->can_disable) {
			error = -EINVAL;
			goto out;
		}
	}

	mutex_lock(&ddata->disable_lock);

	for (i = 0; i < ddata->pdata->nbuttons; i++) {
		struct gpio_button_data *bdata = &ddata->data[i];

		if (bdata->button->type != type)
			continue;

		if (test_bit(*bdata->code, bits))
			gpio_keys_disable_button(bdata);
		else
			gpio_keys_enable_button(bdata);
	}

	mutex_unlock(&ddata->disable_lock);

out:
	bitmap_free(bits);
	return error;
}

#define ATTR_SHOW_FN(name, type, only_disabled)				\
static ssize_t gpio_keys_show_##name(struct device *dev,		\
				     struct device_attribute *attr,	\
				     char *buf)				\
{									\
	struct platform_device *pdev = to_platform_device(dev);		\
	struct gpio_keys_drvdata *ddata = platform_get_drvdata(pdev);	\
									\
	return gpio_keys_attr_show_helper(ddata, buf,			\
					  type, only_disabled);		\
}

ATTR_SHOW_FN(keys, EV_KEY, false);
ATTR_SHOW_FN(switches, EV_SW, false);
ATTR_SHOW_FN(disabled_keys, EV_KEY, true);
ATTR_SHOW_FN(disabled_switches, EV_SW, true);

/*
 * ATTRIBUTES:
 *
 * /sys/devices/platform/gpio-keys/keys [ro]
 * /sys/devices/platform/gpio-keys/switches [ro]
 */
static DEVICE_ATTR(keys, S_IRUGO, gpio_keys_show_keys, NULL);
static DEVICE_ATTR(switches, S_IRUGO, gpio_keys_show_switches, NULL);

#define ATTR_STORE_FN(name, type)					\
static ssize_t gpio_keys_store_##name(struct device *dev,		\
				      struct device_attribute *attr,	\
				      const char *buf,			\
				      size_t count)			\
{									\
	struct platform_device *pdev = to_platform_device(dev);		\
	struct gpio_keys_drvdata *ddata = platform_get_drvdata(pdev);	\
	ssize_t error;							\
									\
	error = gpio_keys_attr_store_helper(ddata, buf, type);		\
	if (error)							\
		return error;						\
									\
	return count;							\
}

ATTR_STORE_FN(disabled_keys, EV_KEY);
ATTR_STORE_FN(disabled_switches, EV_SW);

/*
 * ATTRIBUTES:
 *
 * /sys/devices/platform/gpio-keys/disabled_keys [rw]
 * /sys/devices/platform/gpio-keys/disables_switches [rw]
 */
static DEVICE_ATTR(disabled_keys, S_IWUSR | S_IRUGO,
		   gpio_keys_show_disabled_keys,
		   gpio_keys_store_disabled_keys);
static DEVICE_ATTR(disabled_switches, S_IWUSR | S_IRUGO,
		   gpio_keys_show_disabled_switches,
		   gpio_keys_store_disabled_switches);

#if defined (CONFIG_LGE_DUAL_SCREEN)
static DEVICE_ATTR(virtual_mcu_firmware_write, S_IRUGO | S_IWUSR | S_IWGRP,
       NULL,
       virtual_mcu_firmware_write_store);
static DEVICE_ATTR(cover_recovery_req, S_IRUGO | S_IWUSR | S_IWGRP,
       NULL,
       cover_recovery_req_store);
#endif
#if IS_ENABLED(CONFIG_LGE_SWIVEL_HALLIC_SUPPORT)
static DEVICE_ATTR(swivel_event_refresh, S_IRUGO | S_IWUSR | S_IWGRP,
       NULL,
       swivel_event_refresh_store);
static DEVICE_ATTR(swivel_event_injector, S_IRUGO | S_IWUSR | S_IWGRP,
       swivel_event_injector_show,
       swivel_event_injector_store);
#endif
static struct attribute *gpio_keys_attrs[] = {
	&dev_attr_keys.attr,
	&dev_attr_switches.attr,
	&dev_attr_disabled_keys.attr,
	&dev_attr_disabled_switches.attr,
#if defined(CONFIG_LGE_DUAL_SCREEN)
	&dev_attr_virtual_mcu_firmware_write.attr,
	&dev_attr_cover_recovery_req.attr,
#endif
#if IS_ENABLED(CONFIG_LGE_SWIVEL_HALLIC_SUPPORT)
	&dev_attr_swivel_event_refresh.attr,
	&dev_attr_swivel_event_injector.attr,
#endif
	NULL,
};

static const struct attribute_group gpio_keys_attr_group = {
	.attrs = gpio_keys_attrs,
};

static void gpio_keys_gpio_report_event(struct gpio_button_data *bdata)
{
	const struct gpio_keys_button *button = bdata->button;
	struct input_dev *input = bdata->input;
	unsigned int type = button->type ?: EV_KEY;
	int state;

	state = gpiod_get_value_cansleep(bdata->gpiod);
	if (state < 0) {
		dev_err(input->dev.parent,
			"failed to get gpio state: %d\n", state);
		return;
	}

	if (type == EV_ABS) {
		if (state)
			input_event(input, type, button->code, button->value);
	} else {
#if IS_ENABLED(CONFIG_LGE_SWIVEL_HALLIC_SUPPORT)
		if (!strncmp(bdata->button->desc, "swivel_start", 12)) {
			pr_info("[Display] swivel_start dev.state = %d, state_front = %d, state_back = %d, state = %d\n", swivel_dev.state, swivel_dev.state_front, swivel_dev.state_back, state);
			if (swivel_dev.state_front != state) {
				swivel_dev.state_front = state;

				if (swivel_dev.state_front == 1 && swivel_dev.state_back == 0) {
					state = SWIVEL_CLOSED;
				} else if (swivel_dev.state_front == 0 && swivel_dev.state_back == 0){
					state = SWIVEL_HALF_OPENED;
				} else if (swivel_dev.state_front == 0 && swivel_dev.state_back == 1) {
					state = SWIVEL_OPENED;
				} else if (swivel_dev.state_front == 1 && swivel_dev.state_back == 1) {
					state = SWIVEL_CLOSED; // undefined state, set SWIVEL_CLOSED forcingly
				}

				if (swivel_dev.state != state) {
					pr_info("[Display] swivel_start changed dev.state from %d to %d\n", swivel_dev.state, state);
					if (swivel_dev.state > 0 && state > 0) {
						hallic_set_state(&swivel_dev, 0);
						input_event(input, type, *bdata->code, 0);
						pr_info("[Display] swivel_start send 0 state to avoid ingnoring by input event device\n");
						pr_info("gpio_keys_report_event: code(%d), value(0)\n", button->code);
						input_sync(input);
					}
					hallic_set_state(&swivel_dev, state);
					swivel_dev.state = state;
#ifdef CONFIG_LGE_SAR_CONTROLLER_USB_DETECT
                                        if( state == SWIVEL_OPENED || state == SWIVEL_CLOSED ){
                                            extern void sar_controller_notify_connect(u32 type, bool is_connected);
                                            sar_controller_notify_connect(state, 0); 
                                        }
#endif
#ifdef CONFIG_LGE_TOUCH_CORE_SUB
					pr_info("[Touch] swivel_start changed dev.state from %d to %d\n", swivel_dev.state, state);
					touch_notify_swivel((u32)state);
					touch_sub_notify_swivel((u32)state);
#endif
				} else {
					// do nothing at same state
					return;
				}
			} else {
				// do nothing at same state
				return;
			}
		}

		if (!strncmp(bdata->button->desc, "swivel_end", 10)) {
			pr_info("[Display] swivel_end dev.state = %d, state_front = %d, state_back = %d, state = %d\n", swivel_dev.state, swivel_dev.state_front, swivel_dev.state_back, state);
			if (swivel_dev.state_back != state) {
				swivel_dev.state_back = state;

				if (swivel_dev.state_front == 1 && swivel_dev.state_back == 0) {
					state = SWIVEL_CLOSED;
				} else if (swivel_dev.state_front == 0 && swivel_dev.state_back == 0){
					state = SWIVEL_HALF_OPENED;
				} else if (swivel_dev.state_front == 0 && swivel_dev.state_back == 1) {
					state = SWIVEL_OPENED;
				} else if (swivel_dev.state_front == 1 && swivel_dev.state_back == 1) {
					state = SWIVEL_CLOSED; // undefined state, set SWIVEL_CLOSED forcingly
				}

				if (swivel_dev.state != state) {
					pr_info("[Display] swivel_end changed dev.state from %d to %d\n", swivel_dev.state, state);
					if (swivel_dev.state > 0 && state > 0) {
						hallic_set_state(&swivel_dev, 0);
						input_event(input, type, *bdata->code, 0);
						pr_info("[Display] swivel_end send 0 state to avoid ignoring by input event device\n");
						pr_info("gpio_keys_report_event: code(%d), value(0)\n", button->code);
						input_sync(input);
					}
					hallic_set_state(&swivel_dev, state);
					swivel_dev.state = state;
#ifdef CONFIG_LGE_SAR_CONTROLLER_USB_DETECT
                                        if( state == SWIVEL_OPENED || state == SWIVEL_CLOSED ){
                                            extern void sar_controller_notify_connect(u32 type, bool is_connected);
                                            sar_controller_notify_connect(state, 1); 
                                        }
#endif
#ifdef CONFIG_LGE_TOUCH_CORE_SUB
					pr_info("[Touch] swivel_end changed dev.state from %d to %d\n", swivel_dev.state, state);
					touch_notify_swivel((u32)state);
					touch_sub_notify_swivel((u32)state);
#endif
				} else {
					// do nothing at same state
					return;
				}
			} else {
				// do nothing at same state
				return;
			}
		}
#endif

#if defined(CONFIG_LGE_DUAL_SCREEN)
		if (is_ds_connected() && ((*bdata->code == 115) || (*bdata->code == 377)) ) {
			pr_err("gpio_keys_report_event: skip report event! is_ds_connected(%d), code(%d), value(%d)\n", is_ds_connected(), button->code, state);
		} else {
			input_event(input, type, *bdata->code, state);
			pr_err("gpio_keys_report_event: code(%d), value(%d)\n",button->code, state);
		}
#else
		input_event(input, type, *bdata->code, state);
		pr_err("gpio_keys_report_event: code(%d), value(%d)\n", button->code, state);
#endif
#ifdef CONFIG_LGE_HANDLE_PANIC
		lge_gen_key_panic(button->code, state);
#endif

#ifdef CONFIG_LGE_SUPPORT_HALLIC
#if !IS_ENABLED(CONFIG_LGE_SWIVEL_HALLIC_SUPPORT)
#if defined(CONFIG_LGE_DUAL_SCREEN)
		if (lge_get_dual_display_support()) {
			if (!strncmp(bdata->button->desc, "smart_cover", 11) &&
				!is_ds_connected()) {
				if (sdev.state_front != state) {
					sdev.state_front = state;
					hallic_set_state(&sdev, state);
					pr_info("[Display] smart_cover state switched to %s \n", (state ? "CLOSE" : "OPEN"));
				}
			}

			if (!strncmp(bdata->button->desc, "cover_display_back", 18) &&
				!is_ds_connected()) {
				if (state) {
					state = BACKCOVER_CLOSE;
				}
				if (sdev.state_back != state) {
					sdev.state_back = state;
					hallic_set_state(&sdev, state);
					pr_info("[Display] cover_display_back state switched to %s\n", ((state==BACKCOVER_CLOSE) ? "CLOSE" : "OPEN"));
				}
			}

			if (!strncmp(bdata->button->desc, "ds3_smart_cover", 15)) {
				if (sdev.state_front != state) {
					sdev.state_front = state;
					hallic_set_state(&sdev, state);
					pr_info("[Display][hallIC] %s state switched to %s \n", "ds3_smart_cover", (state ? "CLOSE" : "OPEN"));
				}
			}

			if (!strncmp(bdata->button->desc, "ds3_cover_display_back", 22)) {
				if (state) {
					state = BACKCOVER_CLOSE;
				}
				if (sdev.state_back != state) {
					sdev.state_back = state;
					hallic_set_state(&sdev, state);
					pr_info("[Display][hallIC] %s state switched to %s \n", "ds3_cover_display_back", (state ? "CLOSE" : "OPEN"));
				}
			}

			if (!strncmp(bdata->button->desc, "luke", 4)) {
				if (state == 1) {
					if (lge_get_mfts_mode())
						luke_sdev.state = 1;
					if (!lge_get_factory_boot()) {
						if (sdev.state_front) {
							hallic_set_state(&sdev, sdev.state_front);
							pr_info("[Display][hallIC] Set ds3_smart_cover to close\n");
						}
						if (sdev.state_back) {
							hallic_set_state(&sdev, sdev.state_back);
							pr_info("[Display][hallIC] Set ds3_cover_display_back to close\n");
						}
					}
					set_hallic_status(true);
					pr_info("[Display][hallIC] DS3 cover hallic connected\n");
				} else if (state == 0) {
					if (lge_get_mfts_mode())
						luke_sdev.state = 0;
					if (!lge_get_factory_boot()) {
						if (sdev.state_front) {
							sdev.state_front = 0;
							pr_info("[Display][hallIC] Reset ds3_smart_cover to open");
						}
						if (sdev.state_back) {
							sdev.state_back = 0;
							pr_info("[Display][hallIC] Reset ds3_cover_display_back to open");
						}
						hallic_set_state(&sdev, 0);
					}
					pr_info("[Display][hallIC] DS3 cover hallic disconnected\n");
					set_hallic_status(false);
				}
			}
#ifdef CONFIG_LGE_PM
			if (!strncmp(bdata->button->desc, "luke", 4)) {
				extern void wa_update_hall_ic(bool hall_ic);
				wa_update_hall_ic(!!state);
			}
#endif
		}
#else
		if (!strncmp(bdata->button->desc, "smart_cover", 11)) {
			if (sdev.state_front != state) {
				sdev.state_front = state;
				hallic_set_state(&sdev, state);
				pr_info("[Display] smart_cover state switched to %s \n", (state ? "CLOSE" : "OPEN"));
			}
		}
#endif
		if (!strncmp(bdata->button->desc, "nfc_cover", 9)) {
			if (ndev.state != !!state) {
				hallic_set_state(&ndev, state);
				pr_info("[Display] nfc_cover state switched to %s \n", (state ? "CLOSE" : "OPEN"));
			} else {
				pr_err("[Display] %s: discard wrong nfc_cover irq %s \n", __func__, (state ? "CLOSE" : "OPEN"));
				return;
			}
		}
#endif
#endif
	}
#if defined (CONFIG_LGE_DUAL_SCREEN)
	if (!(is_ds_connected() && ((*bdata->code == 115) || (*bdata->code == 377))))
		input_sync(input);
#else
	input_sync(input);
#endif
}

static void gpio_keys_gpio_work_func(struct work_struct *work)
{
	struct gpio_button_data *bdata =
		container_of(work, struct gpio_button_data, work.work);

	gpio_keys_gpio_report_event(bdata);

	if (bdata->button->wakeup)
		pm_relax(bdata->input->dev.parent);
}

static irqreturn_t gpio_keys_gpio_isr(int irq, void *dev_id)
{
	struct gpio_button_data *bdata = dev_id;

	BUG_ON(irq != bdata->irq);

	if (bdata->button->wakeup) {
		const struct gpio_keys_button *button = bdata->button;

		pm_stay_awake(bdata->input->dev.parent);
		if (bdata->suspended  &&
		    (button->type == 0 || button->type == EV_KEY)) {
			/*
			 * Simulate wakeup key press in case the key has
			 * already released by the time we got interrupt
			 * handler to run.
			 */
			input_report_key(bdata->input, button->code, 1);
		}
	}

	mod_delayed_work(system_wq,
			 &bdata->work,
			 msecs_to_jiffies(bdata->software_debounce));

	return IRQ_HANDLED;
}

static void gpio_keys_irq_timer(struct timer_list *t)
{
	struct gpio_button_data *bdata = from_timer(bdata, t, release_timer);
	struct input_dev *input = bdata->input;
	unsigned long flags;

	spin_lock_irqsave(&bdata->lock, flags);
	if (bdata->key_pressed) {
		input_event(input, EV_KEY, *bdata->code, 0);
		input_sync(input);
		bdata->key_pressed = false;
	}
	spin_unlock_irqrestore(&bdata->lock, flags);
}

static irqreturn_t gpio_keys_irq_isr(int irq, void *dev_id)
{
	struct gpio_button_data *bdata = dev_id;
	struct input_dev *input = bdata->input;
	unsigned long flags;

	BUG_ON(irq != bdata->irq);

	spin_lock_irqsave(&bdata->lock, flags);

	if (!bdata->key_pressed) {
		if (bdata->button->wakeup)
			pm_wakeup_event(bdata->input->dev.parent, 0);

		input_event(input, EV_KEY, *bdata->code, 1);
		input_sync(input);

		if (!bdata->release_delay) {
			input_event(input, EV_KEY, *bdata->code, 0);
			input_sync(input);
			goto out;
		}

		bdata->key_pressed = true;
	}

	if (bdata->release_delay)
		mod_timer(&bdata->release_timer,
			jiffies + msecs_to_jiffies(bdata->release_delay));
out:
	spin_unlock_irqrestore(&bdata->lock, flags);
	return IRQ_HANDLED;
}

static void gpio_keys_quiesce_key(void *data)
{
	struct gpio_button_data *bdata = data;

	if (bdata->gpiod)
		cancel_delayed_work_sync(&bdata->work);
	else
		del_timer_sync(&bdata->release_timer);
}

static int gpio_keys_setup_key(struct platform_device *pdev,
				struct input_dev *input,
				struct gpio_keys_drvdata *ddata,
				const struct gpio_keys_button *button,
				int idx,
				struct fwnode_handle *child)
{
	const char *desc = button->desc ? button->desc : "gpio_keys";
	struct device *dev = &pdev->dev;
	struct gpio_button_data *bdata = &ddata->data[idx];
	irq_handler_t isr;
	unsigned long irqflags;
	int irq;
	int error;

	bdata->input = input;
	bdata->button = button;
	spin_lock_init(&bdata->lock);

	if (child) {
		bdata->gpiod = devm_fwnode_get_gpiod_from_child(dev, NULL,
								child,
								GPIOD_IN,
								desc);
		if (IS_ERR(bdata->gpiod)) {
			error = PTR_ERR(bdata->gpiod);
			if (error == -ENOENT) {
				/*
				 * GPIO is optional, we may be dealing with
				 * purely interrupt-driven setup.
				 */
				bdata->gpiod = NULL;
			} else {
				if (error != -EPROBE_DEFER)
					dev_err(dev, "failed to get gpio: %d\n",
						error);
				return error;
			}
		}
	} else if (gpio_is_valid(button->gpio)) {
		/*
		 * Legacy GPIO number, so request the GPIO here and
		 * convert it to descriptor.
		 */
		unsigned flags = GPIOF_IN;

		if (button->active_low)
			flags |= GPIOF_ACTIVE_LOW;

		error = devm_gpio_request_one(dev, button->gpio, flags, desc);
		if (error < 0) {
			dev_err(dev, "Failed to request GPIO %d, error %d\n",
				button->gpio, error);
			return error;
		}

		bdata->gpiod = gpio_to_desc(button->gpio);
		if (!bdata->gpiod)
			return -EINVAL;
	}

	if (bdata->gpiod) {
		bool active_low = gpiod_is_active_low(bdata->gpiod);

		if (button->debounce_interval) {
			error = gpiod_set_debounce(bdata->gpiod,
					button->debounce_interval * 1000);
			/* use timer if gpiolib doesn't provide debounce */
			if (error < 0)
				bdata->software_debounce =
						button->debounce_interval;
		}
#ifdef CONFIG_LGE_SUPPORT_HALLIC
#if IS_ENABLED(CONFIG_LGE_SWIVEL_HALLIC_SUPPORT)
		if (bdata->button->desc != NULL) {
			if (!strncmp(bdata->button->desc, "swivel_start", 12))
			{
				if (hallic_register(&swivel_dev) < 0) {
					pr_err("[Display] swivel_start switch registration failed\n");
				}
				pr_info("[Display] swivel_start_dev switch registration success\n");
			}
			if (!strncmp(bdata->button->desc, "swivel_end", 10))
			{
				pr_info("[Display] swivel_end_dev switch registration success\n");
			}
		}
#else
#if defined(CONFIG_LGE_DUAL_SCREEN)
		if (bdata->button->desc != NULL && lge_get_dual_display_support()) {
			if (!strncmp(bdata->button->desc, "smart_cover", 11)) {
				if (hallic_register(&sdev) < 0) {
					pr_err("[Display] smart_cover switch registration failed\n");
				}
				pr_info("[Display] smart_cover_dev switch registration success\n");
			}

			if (!strncmp(bdata->button->desc, "ds3_smart_cover", 15)) {
				if (hallic_register(&sdev) < 0) {
					pr_err("[Display] ds3_smart_cover switch registration failed\n");
				}
				pr_info("[Display] ds3_smart_cover_dev switch registration success\n");
			}

			if (!strncmp(bdata->button->desc, "ds3_cover_display_back", 22))
				pr_info("[Display] ds3_cover_display_back register");

			if (!strncmp(bdata->button->desc, "luke", 4)) {
				if (hallic_register(&luke_sdev) < 0) {
					pr_err("[Display] luke_dev switch registration failed\n");
				}
				pr_info("[Display] luke_dev switch registration success\n");
			}
		}
#else
		if (bdata->button->desc != NULL) {
			if (!strncmp(bdata->button->desc, "smart_cover", 11))
			{
				if (hallic_register(&sdev) < 0) {
					pr_err("[Display] smart_cover switch registration failed\n");
				}
				pr_info("[Display] smart_cover_dev switch registration success\n");
			}
		}
#endif
		if (bdata->button->desc != NULL) {
			if (!strncmp(bdata->button->desc, "nfc_cover", 9)) {
				hallic_register(&ndev);
				pr_info("[Display] hallic_dev switch registration success\n");
			}
		}
#endif
#endif

		if (button->irq) {
			bdata->irq = button->irq;
		} else {
			irq = gpiod_to_irq(bdata->gpiod);
			if (irq < 0) {
				error = irq;
				dev_err(dev,
					"Unable to get irq number for GPIO %d, error %d\n",
					button->gpio, error);
				return error;
			}
			bdata->irq = irq;
		}

		INIT_DELAYED_WORK(&bdata->work, gpio_keys_gpio_work_func);

		isr = gpio_keys_gpio_isr;
		irqflags = IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING;

		switch (button->wakeup_event_action) {
		case EV_ACT_ASSERTED:
			bdata->wakeup_trigger_type = active_low ?
				IRQ_TYPE_EDGE_FALLING : IRQ_TYPE_EDGE_RISING;
			break;
		case EV_ACT_DEASSERTED:
			bdata->wakeup_trigger_type = active_low ?
				IRQ_TYPE_EDGE_RISING : IRQ_TYPE_EDGE_FALLING;
			break;
		case EV_ACT_ANY:
			/* fall through */
		default:
			/*
			 * For other cases, we are OK letting suspend/resume
			 * not reconfigure the trigger type.
			 */
			break;
		}
	} else {
		if (!button->irq) {
			dev_err(dev, "Found button without gpio or irq\n");
			return -EINVAL;
		}

		bdata->irq = button->irq;

		if (button->type && button->type != EV_KEY) {
			dev_err(dev, "Only EV_KEY allowed for IRQ buttons.\n");
			return -EINVAL;
		}

		bdata->release_delay = button->debounce_interval;
		timer_setup(&bdata->release_timer, gpio_keys_irq_timer, 0);

		isr = gpio_keys_irq_isr;
		irqflags = 0;

		/*
		 * For IRQ buttons, there is no interrupt for release.
		 * So we don't need to reconfigure the trigger type for wakeup.
		 */
	}

	bdata->code = &ddata->keymap[idx];
	*bdata->code = button->code;
	input_set_capability(input, button->type ?: EV_KEY, *bdata->code);

	/*
	 * Install custom action to cancel release timer and
	 * workqueue item.
	 */
	error = devm_add_action(dev, gpio_keys_quiesce_key, bdata);
	if (error) {
		dev_err(dev, "failed to register quiesce action, error: %d\n",
			error);
		return error;
	}

	/*
	 * If platform has specified that the button can be disabled,
	 * we don't want it to share the interrupt line.
	 */
	if (!button->can_disable)
		irqflags |= IRQF_SHARED;

	error = devm_request_any_context_irq(dev, bdata->irq, isr, irqflags,
					     desc, bdata);
	if (error < 0) {
		dev_err(dev, "Unable to claim irq %d; error %d\n",
			bdata->irq, error);
		return error;
	}

	return 0;
}

static void gpio_keys_report_state(struct gpio_keys_drvdata *ddata)
{
	struct input_dev *input = ddata->input;
	int i;

	for (i = 0; i < ddata->pdata->nbuttons; i++) {
		struct gpio_button_data *bdata = &ddata->data[i];
		if (bdata->gpiod)
			gpio_keys_gpio_report_event(bdata);
	}
	input_sync(input);
}

static int gpio_keys_open(struct input_dev *input)
{
	struct gpio_keys_drvdata *ddata = input_get_drvdata(input);
	const struct gpio_keys_platform_data *pdata = ddata->pdata;
	int error;

	if (pdata->enable) {
		error = pdata->enable(input->dev.parent);
		if (error)
			return error;
	}

	/* Report current state of buttons that are connected to GPIOs */
	gpio_keys_report_state(ddata);

	return 0;
}

static void gpio_keys_close(struct input_dev *input)
{
	struct gpio_keys_drvdata *ddata = input_get_drvdata(input);
	const struct gpio_keys_platform_data *pdata = ddata->pdata;

	if (pdata->disable)
		pdata->disable(input->dev.parent);
}

/*
 * Handlers for alternative sources of platform_data
 */

/*
 * Translate properties into platform_data
 */
static struct gpio_keys_platform_data *
gpio_keys_get_devtree_pdata(struct device *dev)
{
	struct gpio_keys_platform_data *pdata;
	struct gpio_keys_button *button;
	struct fwnode_handle *child;
	int nbuttons;

	nbuttons = device_get_child_node_count(dev);
	if (nbuttons == 0)
		return ERR_PTR(-ENODEV);

	pdata = devm_kzalloc(dev,
			     sizeof(*pdata) + nbuttons * sizeof(*button),
			     GFP_KERNEL);
	if (!pdata)
		return ERR_PTR(-ENOMEM);

	button = (struct gpio_keys_button *)(pdata + 1);

	pdata->buttons = button;
	pdata->nbuttons = nbuttons;

	pdata->rep = device_property_read_bool(dev, "autorepeat");

	device_property_read_string(dev, "label", &pdata->name);

	device_for_each_child_node(dev, child) {
		if (is_of_node(child))
			button->irq =
				irq_of_parse_and_map(to_of_node(child), 0);

		if (fwnode_property_read_u32(child, "linux,code",
					     &button->code)) {
			dev_err(dev, "Button without keycode\n");
			fwnode_handle_put(child);
			return ERR_PTR(-EINVAL);
		}

		fwnode_property_read_string(child, "label", &button->desc);

		if (fwnode_property_read_u32(child, "linux,input-type",
					     &button->type))
			button->type = EV_KEY;

		button->wakeup =
			fwnode_property_read_bool(child, "wakeup-source") ||
			/* legacy name */
			fwnode_property_read_bool(child, "gpio-key,wakeup");

		fwnode_property_read_u32(child, "wakeup-event-action",
					 &button->wakeup_event_action);

		button->can_disable =
			fwnode_property_read_bool(child, "linux,can-disable");

		if (fwnode_property_read_u32(child, "debounce-interval",
					 &button->debounce_interval))
			button->debounce_interval = 5;

		button++;
	}

	return pdata;
}

static const struct of_device_id gpio_keys_of_match[] = {
	{ .compatible = "gpio-keys", },
	{ },
};
MODULE_DEVICE_TABLE(of, gpio_keys_of_match);

static int gpio_keys_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const struct gpio_keys_platform_data *pdata = dev_get_platdata(dev);
	struct fwnode_handle *child = NULL;
	struct gpio_keys_drvdata *ddata;
	struct input_dev *input;
	size_t size;
	int i, error;
	int wakeup = 0;

	if (!pdata) {
		pdata = gpio_keys_get_devtree_pdata(dev);
		if (IS_ERR(pdata))
			return PTR_ERR(pdata);
	}

	size = sizeof(struct gpio_keys_drvdata) +
			pdata->nbuttons * sizeof(struct gpio_button_data);
	ddata = devm_kzalloc(dev, size, GFP_KERNEL);
	if (!ddata) {
		dev_err(dev, "failed to allocate state\n");
		return -ENOMEM;
	}

	ddata->keymap = devm_kcalloc(dev,
				     pdata->nbuttons, sizeof(ddata->keymap[0]),
				     GFP_KERNEL);
	if (!ddata->keymap)
		return -ENOMEM;

	input = devm_input_allocate_device(dev);
	if (!input) {
		dev_err(dev, "failed to allocate input device\n");
		return -ENOMEM;
	}

	ddata->pdata = pdata;
	ddata->input = input;
	mutex_init(&ddata->disable_lock);

	platform_set_drvdata(pdev, ddata);
	input_set_drvdata(input, ddata);

	input->name = pdata->name ? : pdev->name;
	input->phys = "gpio-keys/input0";
	input->dev.parent = dev;
	input->open = gpio_keys_open;
	input->close = gpio_keys_close;

	input->id.bustype = BUS_HOST;
	input->id.vendor = 0x0001;
	input->id.product = 0x0001;
	input->id.version = 0x0100;

	input->keycode = ddata->keymap;
	input->keycodesize = sizeof(ddata->keymap[0]);
	input->keycodemax = pdata->nbuttons;

	/* Enable auto repeat feature of Linux input subsystem */
	if (pdata->rep)
		__set_bit(EV_REP, input->evbit);

	for (i = 0; i < pdata->nbuttons; i++) {
		const struct gpio_keys_button *button = &pdata->buttons[i];

		if (!dev_get_platdata(dev)) {
			child = device_get_next_child_node(dev, child);
			if (!child) {
				dev_err(dev,
					"missing child device node for entry %d\n",
					i);
				return -EINVAL;
			}
		}

		error = gpio_keys_setup_key(pdev, input, ddata,
					    button, i, child);
		if (error) {
			fwnode_handle_put(child);
			return error;
		}

		if (button->wakeup)
			wakeup = 1;
	}

	fwnode_handle_put(child);

#if defined(CONFIG_LGE_DUAL_SCREEN)
	if (!cover_fw_dev.dev) {
		error = hallic_register(&cover_fw_dev);
		if (error) {
			pr_err("cover_fw_dev switch dev register failed\n");
			return error;
		}
		else
			pr_err("cover_fw_dev switch registration success\n");
	}
	if (!cover_recovery.dev) {
		error = hallic_register(&cover_recovery);
		if(error) {
			pr_err("cover recovery switch dev register failed\n");
			return error;
		}
	}
#endif

	error = devm_device_add_group(dev, &gpio_keys_attr_group);
	if (error) {
		dev_err(dev, "Unable to export keys/switches, error: %d\n",
			error);
		return error;
	}

	error = input_register_device(input);
	if (error) {
		dev_err(dev, "Unable to register input device, error: %d\n",
			error);
		return error;
	}

	device_init_wakeup(dev, wakeup);

	return 0;
}

static int __maybe_unused
gpio_keys_button_enable_wakeup(struct gpio_button_data *bdata)
{
	int error;

	error = enable_irq_wake(bdata->irq);
	if (error) {
		dev_err(bdata->input->dev.parent,
			"failed to configure IRQ %d as wakeup source: %d\n",
			bdata->irq, error);
		return error;
	}

	if (bdata->wakeup_trigger_type) {
		error = irq_set_irq_type(bdata->irq,
					 bdata->wakeup_trigger_type);
		if (error) {
			dev_err(bdata->input->dev.parent,
				"failed to set wakeup trigger %08x for IRQ %d: %d\n",
				bdata->wakeup_trigger_type, bdata->irq, error);
			disable_irq_wake(bdata->irq);
			return error;
		}
	}

	return 0;
}

static void __maybe_unused
gpio_keys_button_disable_wakeup(struct gpio_button_data *bdata)
{
	int error;

	/*
	 * The trigger type is always both edges for gpio-based keys and we do
	 * not support changing wakeup trigger for interrupt-based keys.
	 */
	if (bdata->wakeup_trigger_type) {
		error = irq_set_irq_type(bdata->irq, IRQ_TYPE_EDGE_BOTH);
		if (error)
			dev_warn(bdata->input->dev.parent,
				 "failed to restore interrupt trigger for IRQ %d: %d\n",
				 bdata->irq, error);
	}

	error = disable_irq_wake(bdata->irq);
	if (error)
		dev_warn(bdata->input->dev.parent,
			 "failed to disable IRQ %d as wake source: %d\n",
			 bdata->irq, error);
}

static int __maybe_unused
gpio_keys_enable_wakeup(struct gpio_keys_drvdata *ddata)
{
	struct gpio_button_data *bdata;
	int error;
	int i;

	for (i = 0; i < ddata->pdata->nbuttons; i++) {
		bdata = &ddata->data[i];
		if (bdata->button->wakeup) {
			error = gpio_keys_button_enable_wakeup(bdata);
			if (error)
				goto err_out;
		}
		bdata->suspended = true;
	}

	return 0;

err_out:
	while (i--) {
		bdata = &ddata->data[i];
		if (bdata->button->wakeup)
			gpio_keys_button_disable_wakeup(bdata);
		bdata->suspended = false;
	}

	return error;
}

static void __maybe_unused
gpio_keys_disable_wakeup(struct gpio_keys_drvdata *ddata)
{
	struct gpio_button_data *bdata;
	int i;

	for (i = 0; i < ddata->pdata->nbuttons; i++) {
		bdata = &ddata->data[i];
		bdata->suspended = false;
		if (irqd_is_wakeup_set(irq_get_irq_data(bdata->irq)))
			gpio_keys_button_disable_wakeup(bdata);
	}
}

static int __maybe_unused gpio_keys_suspend(struct device *dev)
{
	struct gpio_keys_drvdata *ddata = dev_get_drvdata(dev);
	struct input_dev *input = ddata->input;
	int error;

	if (device_may_wakeup(dev)) {
		error = gpio_keys_enable_wakeup(ddata);
		if (error)
			return error;
	} else {
		mutex_lock(&input->mutex);
		if (input->users)
			gpio_keys_close(input);
		mutex_unlock(&input->mutex);
	}

	return 0;
}

static int __maybe_unused gpio_keys_resume(struct device *dev)
{
	struct gpio_keys_drvdata *ddata = dev_get_drvdata(dev);
	struct input_dev *input = ddata->input;
	int error = 0;

	if (device_may_wakeup(dev)) {
		gpio_keys_disable_wakeup(ddata);
	} else {
		mutex_lock(&input->mutex);
		if (input->users)
			error = gpio_keys_open(input);
		mutex_unlock(&input->mutex);
	}

	if (error)
		return error;

	gpio_keys_report_state(ddata);
	return 0;
}

static SIMPLE_DEV_PM_OPS(gpio_keys_pm_ops, gpio_keys_suspend, gpio_keys_resume);

static struct platform_driver gpio_keys_device_driver = {
	.probe		= gpio_keys_probe,
	.driver		= {
		.name	= "gpio-keys",
		.pm	= &gpio_keys_pm_ops,
		.of_match_table = gpio_keys_of_match,
	}
};

static int __init gpio_keys_init(void)
{
	return platform_driver_register(&gpio_keys_device_driver);
}

static void __exit gpio_keys_exit(void)
{
	platform_driver_unregister(&gpio_keys_device_driver);
}

late_initcall(gpio_keys_init);
module_exit(gpio_keys_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Phil Blundell <pb@handhelds.org>");
MODULE_DESCRIPTION("Keyboard driver for GPIOs");
MODULE_ALIAS("platform:gpio-keys");
