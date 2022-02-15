/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Sysfs APIs for Google Pixel devices.
 *
 * Copyright 2022 Google LLC.
 */

#ifndef _TOUCH_PM_H_
#define _TOUCH_PM_H_

#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <samsung/exynos_drm_connector.h>
#include <samsung/panel/panel-samsung-drv.h>

enum tpm_power_state {
	TPM_PWR_OFF = 0,
	TPM_PWR_ON,
};

/**
 * @brief: wakelock type.
 */
enum tpm_wakelock_type {
	TPM_WAKELOCK_TYPE_SCREEN_ON = 0x0001,
	TPM_WAKELOCK_TYPE_IRQ = 0x0002,
	TPM_WAKELOCK_TYPE_FW_UPDATE = 0x0004,
	TPM_WAKELOCK_TYPE_SYSFS = 0x0008,
	TPM_WAKELOCK_TYPE_FORCE_ACTIVE = 0x0010,
	TPM_WAKELOCK_TYPE_BUGREPORT = 0x0020,
};

struct touch_pm {
	/* PLatform device driver */
	struct platform_device *pdev;
#ifdef CONFIG_OF
	struct device_node *of_node;
#endif

	struct work_struct suspend_work;
	struct work_struct resume_work;
	struct workqueue_struct *event_wq;
	struct completion bus_resumed;

	u32 locks;
	struct mutex lock_mutex;

	/* flags */
	int pwr_state;

	struct drm_bridge panel_bridge;
	struct drm_connector *connector;
	bool is_panel_lp_mode;

	/* Specific function pointer to resume the device from suspend state.
	 *
	 * @param
	 *    [ in] dev: an instance of device
	 *
	 * @return
	 *    on success, 0; otherwise, negative value on error.
	 */
	int (*resume)(struct device *dev);

	/* Specific function pointer to put device into suspend state.
	 *
	 * @param
	 *    [ in] dev: an instance of device
	 *
	 * @return
	 *    on success, 0; otherwise, negative value on error.
	 */
	int (*suspend)(struct device *dev);
};

extern int tpm_lock_wakelock(struct touch_pm *tpm, enum tpm_wakelock_type lock);
extern int tpm_unlock_wakelock(
	struct touch_pm *tpm, enum tpm_wakelock_type lock);
extern int tpm_register_notification(struct touch_pm *tpm);
extern int tpm_unregister_notification(struct touch_pm *tpm);

#endif
