// SPDX-License-Identifier: GPL-2.0
/*
 * Sysfs APIs for Google Pixel devices.
 *
 * Copyright 2022 Google LLC.
 */

#include "touch_pm.h"

struct drm_connector *tpm_get_bridge_connector(struct drm_bridge *bridge)
{
	struct drm_connector *connector;
	struct drm_connector_list_iter conn_iter;

	drm_connector_list_iter_begin(bridge->dev, &conn_iter);
	drm_for_each_connector_iter(connector, &conn_iter)
	{
		if (connector->encoder == bridge->encoder)
			break;
	}
	drm_connector_list_iter_end(&conn_iter);
	return connector;
}

static bool tpm_bridge_is_lp_mode(struct drm_connector *connector)
{
	if (connector && connector->state) {
		struct exynos_drm_connector_state *s =
			to_exynos_connector_state(connector->state);
		return s->exynos_mode.is_lp_mode;
	}
	return false;
}

static void tpm_panel_bridge_enable(struct drm_bridge *bridge)
{
	struct touch_pm *tpm =
		container_of(bridge, struct touch_pm, panel_bridge);

	pr_debug("%s\n", __func__);
	if (!tpm->is_panel_lp_mode)
		tpm_lock_wakelock(tpm, TPM_WAKELOCK_TYPE_SCREEN_ON);
}

static void tpm_panel_bridge_disable(struct drm_bridge *bridge)
{
	struct touch_pm *tpm =
		container_of(bridge, struct touch_pm, panel_bridge);

	if (bridge->encoder && bridge->encoder->crtc) {
		const struct drm_crtc_state *crtc_state =
			bridge->encoder->crtc->state;

		if (drm_atomic_crtc_effectively_active(crtc_state))
			return;
	}

	pr_debug("%s\n", __func__);
	tpm_unlock_wakelock(tpm, TPM_WAKELOCK_TYPE_SCREEN_ON);
}

static void tpm_panel_bridge_mode_set(struct drm_bridge *bridge,
	const struct drm_display_mode *mode,
	const struct drm_display_mode *adjusted_mode)
{
	struct touch_pm *tpm =
		container_of(bridge, struct touch_pm, panel_bridge);

	pr_debug("%s\n", __func__);

	if (!tpm->connector || !tpm->connector->state) {
		pr_info("%s: Get bridge connector.\n", __func__);
		tpm->connector = tpm_get_bridge_connector(bridge);
	}

	tpm->is_panel_lp_mode = tpm_bridge_is_lp_mode(tpm->connector);
	if (tpm->is_panel_lp_mode)
		tpm_unlock_wakelock(tpm, TPM_WAKELOCK_TYPE_SCREEN_ON);
	else
		tpm_lock_wakelock(tpm, TPM_WAKELOCK_TYPE_SCREEN_ON);
}

static const struct drm_bridge_funcs panel_bridge_funcs = {
	.enable = tpm_panel_bridge_enable,
	.disable = tpm_panel_bridge_disable,
	.mode_set = tpm_panel_bridge_mode_set,
};

static int tpm_register_panel_bridge(struct touch_pm *tpm)
{
#ifdef CONFIG_OF
	tpm->panel_bridge.of_node = tpm->of_node;
#endif
	tpm->panel_bridge.funcs = &panel_bridge_funcs;
	drm_bridge_add(&tpm->panel_bridge);
	return 0;
}

static void tpm_unregister_panel_bridge(struct drm_bridge *bridge)
{
	struct drm_bridge *node;

	drm_bridge_remove(bridge);

	if (!bridge->dev) /* not attached */
		return;

	drm_modeset_lock(&bridge->dev->mode_config.connection_mutex, NULL);
	list_for_each_entry(node, &bridge->encoder->bridge_chain,
		chain_node) if (node == bridge)
	{
		if (bridge->funcs->detach)
			bridge->funcs->detach(bridge);
		list_del(&bridge->chain_node);
		break;
	}
	drm_modeset_unlock(&bridge->dev->mode_config.connection_mutex);
	bridge->dev = NULL;
}

int tpm_lock_wakelock(struct touch_pm *tpm, enum tpm_wakelock_type type)
{
	int ret = 0;
	int lock = type & TPM_WAKELOCK_TYPE_LOCK_MASK;
	bool wait_resume = false;

	mutex_lock(&tpm->lock_mutex);

	if (tpm->locks & lock) {
		dev_dbg(&tpm->pdev->dev,
			"unexpectedly lock: locks=0x%04X, lock=0x%04X\n",
			tpm->locks, lock);
		mutex_unlock(&tpm->lock_mutex);
		return -EINVAL;
	}

	/*
	 * If NON_WAKE_UP is set and the pm is suspend, we should ignore it.
	 * For example, IRQs should only keep the bus active. IRQs received
	 * while the pm is suspend should be ignored.
	 */
	if (type & TPM_WAKELOCK_TYPE_NON_WAKE_UP && tpm->locks == 0) {
		mutex_unlock(&tpm->lock_mutex);
		return -EAGAIN;
	}

	tpm->locks |= lock;

	if ((type & TPM_WAKELOCK_TYPE_NON_WAKE_UP) != 0) {
		mutex_unlock(&tpm->lock_mutex);
		return ret;
	}

	/*
	 * When triggering a wake, wait up to one second to resume.
	 * SCREEN_ON does not need to wait.
	 */
	if (lock != TPM_WAKELOCK_TYPE_SCREEN_ON) {
		wait_resume = true;
	}

	mutex_unlock(&tpm->lock_mutex);

	/* Complete or cancel any outstanding transitions */
	cancel_work_sync(&tpm->suspend_work);
	cancel_work_sync(&tpm->resume_work);
	queue_work(tpm->event_wq, &tpm->resume_work);

	if (wait_resume) {
		wait_for_completion_timeout(&tpm->bus_resumed, HZ);
		if (tpm->pwr_state != TPM_PWR_ON) {
			dev_err(&tpm->pdev->dev,
				"Failed to wake the touch bus.\n");
			ret = -ETIMEDOUT;
		}
	}

	return ret;
}

int tpm_unlock_wakelock(struct touch_pm *tpm, enum tpm_wakelock_type type)
{
	int ret = 0;
	int lock = type & TPM_WAKELOCK_TYPE_LOCK_MASK;

	mutex_lock(&tpm->lock_mutex);

	if (!(tpm->locks & lock)) {
		dev_dbg(&tpm->pdev->dev,
			"unexpectedly unlock: locks=0x%04X, lock=0x%04X\n",
			tpm->locks, lock);
		mutex_unlock(&tpm->lock_mutex);
		return -EINVAL;
	}

	tpm->locks &= ~lock;

	if (tpm->locks == 0) {
		mutex_unlock(&tpm->lock_mutex);
		/* Complete or cancel any outstanding transitions */
		cancel_work_sync(&tpm->suspend_work);
		cancel_work_sync(&tpm->resume_work);

		mutex_lock(&tpm->lock_mutex);
		if (tpm->locks == 0)
			queue_work(tpm->event_wq, &tpm->suspend_work);
	}
	mutex_unlock(&tpm->lock_mutex);

	return ret;
}

bool tpm_get_lock_state(struct touch_pm *tpm, enum tpm_wakelock_type type)
{
	return tpm->locks & type ? true : false;
}

int tpm_get_lock_states(struct touch_pm *tpm)
{
	return tpm->locks;
}

static void tpm_suspend_work(struct work_struct *work)
{
	struct touch_pm *tpm =
		container_of(work, struct touch_pm, suspend_work);

	/* exit directly if device is already in suspend state */
	if (tpm->pwr_state == TPM_PWR_OFF)
		return;
	tpm->pwr_state = TPM_PWR_OFF;

	reinit_completion(&tpm->bus_resumed);
	if (tpm->suspend) {
		tpm->suspend(&tpm->pdev->dev);
	}
}

static void tpm_resume_work(struct work_struct *work)
{
	struct touch_pm *tpm = container_of(work, struct touch_pm, resume_work);

	/* exit directly if device isn't in suspend state */
	if (tpm->pwr_state == TPM_PWR_ON)
		return;
	tpm->pwr_state = TPM_PWR_ON;

	if (tpm->resume) {
		tpm->resume(&tpm->pdev->dev);
	}
	complete_all(&tpm->bus_resumed);
}

int tpm_register_notification(struct touch_pm *tpm)
{
	int ret = 0;

	tpm->pwr_state = TPM_PWR_ON;
	tpm->locks = 0;
	tpm->event_wq = alloc_workqueue(
		"tpm_wq", WQ_UNBOUND | WQ_HIGHPRI | WQ_CPU_INTENSIVE, 1);
	if (!tpm->event_wq) {
		dev_err(&tpm->pdev->dev, "Cannot create work thread\n");
		ret = -ENOMEM;
		goto err_alloc_workqueue;
	}

	mutex_init(&tpm->lock_mutex);
	INIT_WORK(&tpm->suspend_work, tpm_suspend_work);
	INIT_WORK(&tpm->resume_work, tpm_resume_work);

	init_completion(&tpm->bus_resumed);
	complete_all(&tpm->bus_resumed);

	tpm_register_panel_bridge(tpm);
	return ret;

err_alloc_workqueue:
	return ret;
}

int tpm_unregister_notification(struct touch_pm *tpm)
{
	tpm_unregister_panel_bridge(&tpm->panel_bridge);
	tpm->resume = NULL;
	tpm->suspend = NULL;
	return 0;
}
