#include "goodix_ts_core.h"
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/version.h>

#define CMD_FW_UPDATE "fw_update"
#define CMD_AUTO_TEST "auto_test"
#define CMD_GET_VERSION "get_version"
#define CMD_GET_RAWDATA "get_raw"
#define CMD_GET_DIFFDATA "get_diff"
#define CMD_GET_SELF_RAWDATA "get_self_raw"
#define CMD_GET_SELF_DIFFDATA "get_self_diff"
#define CMD_SET_DOUBLE_TAP "set_double_tap"
#define CMD_SET_SINGLE_TAP "set_single_tap"
#define CMD_SET_CHARGE_MODE "set_charge_mode"
#define CMD_SET_IRQ_ENABLE "set_irq_enable"
#define CMD_SET_ESD_ENABLE "set_esd_enable"
#define CMD_SET_DEBUG_LOG "set_debug_log"

#define SHORT_SIZE 100
#define LARGE_SIZE 4096
static struct goodix_ts_core *cd;
static char wbuf[SHORT_SIZE];
static char *rbuf;
static uint32_t index;

static void *seq_start(struct seq_file *s, loff_t *pos)
{
	if (*pos >= index)
		return NULL;

	return rbuf + *pos;
}

static int seq_show(struct seq_file *s, void *v)
{
	seq_printf(s, (u8 *)v);
	return 0;
}

static void *seq_next(struct seq_file *s, void *v, loff_t *pos)
{
	*pos += index;
	return NULL;
}

static void seq_stop(struct seq_file *s, void *v)
{
	if (s->read_pos >= index) {
		// ts_info("read_pos:%d", (int)s->read_pos);
		kfree(rbuf);
		rbuf = NULL;
		index = 0;
	}
}

static const struct seq_operations seq_ops = {
	.start = seq_start, .next = seq_next, .stop = seq_stop, .show = seq_show
};

static int driver_test_open(struct inode *inode, struct file *file)
{
	memset(wbuf, 0, sizeof(wbuf));

	return seq_open(file, &seq_ops);
}

static int driver_test_release(struct inode *inode, struct file *file)
{
	return seq_release(inode, file);
}

static int get_cap_data(uint8_t *type)
{
	struct goodix_ts_cmd temp_cmd;
	int tx = cd->ic_info.parm.drv_num;
	int rx = cd->ic_info.parm.sen_num;
	uint8_t val;
	int retry = 20;
	struct frame_head *frame_head;
	unsigned char frame_buf[GOODIX_MAX_FRAMEDATA_LEN];
	unsigned char *cur_ptr;
	unsigned int flag_addr = cd->ic_info.misc.frame_data_addr;
	int i;
	int ret;

	/* disable irq & close esd */
	cd->hw_ops->irq_enable(cd, false);
	goodix_ts_blocking_notify(NOTIFY_ESD_OFF, NULL);

	if (strstr(type, CMD_GET_RAWDATA) ||
		strstr(type, CMD_GET_SELF_RAWDATA)) {
		temp_cmd.data[0] = 0x81;
	} else {
		temp_cmd.data[0] = 0x82;
	}
	temp_cmd.cmd = 0x90;
	temp_cmd.len = 5;
	ret = cd->hw_ops->send_cmd(cd, &temp_cmd);
	if (ret < 0) {
		ts_err("report rawdata failed, exit!");
		goto exit;
	}

	/* clean touch event flag */
	val = 0;
	ret = cd->hw_ops->write(cd, flag_addr, &val, 1);
	if (ret < 0) {
		ts_err("clean touch event failed, exit!");
		goto exit;
	}

	while (retry--) {
		usleep_range(2000, 2100);
		ret = cd->hw_ops->read(cd, flag_addr, &val, 1);
		if (!ret && (val & 0x80))
			break;
	}
	if (retry < 0) {
		ts_err("framedata is not ready val:0x%02x, exit!", val);
		ret = -EINVAL;
		goto exit;
	}

	ret = cd->hw_ops->read(
		cd, flag_addr, frame_buf, GOODIX_MAX_FRAMEDATA_LEN);
	if (ret < 0) {
		ts_err("read frame data failed");
		goto exit;
	}

	if (checksum_cmp(frame_buf, cd->ic_info.misc.frame_data_head_len,
		    CHECKSUM_MODE_U8_LE)) {
		ts_err("frame head checksum error");
		ret = -EINVAL;
		goto exit;
	}

	frame_head = (struct frame_head *)frame_buf;
	if (checksum_cmp(frame_buf, frame_head->cur_frame_len,
		    CHECKSUM_MODE_U16_LE)) {
		ts_err("frame body checksum error");
		ret = -EINVAL;
		goto exit;
	}
	cur_ptr = frame_buf;
	cur_ptr += cd->ic_info.misc.frame_data_head_len;
	cur_ptr += cd->ic_info.misc.fw_attr_len;
	cur_ptr += cd->ic_info.misc.fw_log_len;
	cur_ptr += 8;

	if (strstr(type, CMD_GET_RAWDATA) || strstr(type, CMD_GET_DIFFDATA)) {
		goodix_rotate_abcd2cbad(tx, rx, (s16 *)cur_ptr);
		for (i = 0; i < tx * rx; i++) {
			index += sprintf(
				&rbuf[index], "%5d,", *((s16 *)cur_ptr + i));
			if ((i + 1) % tx == 0)
				index += sprintf(&rbuf[index], "\n");
		}
	} else {
		cur_ptr += tx * rx * 2 + 10;
		index += sprintf(&rbuf[index], "TX:");
		for (i = 0; i < tx + rx; i++) {
			index += sprintf(
				&rbuf[index], "%5d,", *((s16 *)cur_ptr + i));
			if ((i + 1) == tx)
				index += sprintf(&rbuf[index], "\nRX:");
		}
		index += sprintf(&rbuf[index], "\n");
	}

exit:
	temp_cmd.cmd = 0x90;
	temp_cmd.data[0] = 0;
	temp_cmd.len = 5;
	cd->hw_ops->send_cmd(cd, &temp_cmd);
	/* enable irq & esd */
	cd->hw_ops->irq_enable(cd, true);
	goodix_ts_blocking_notify(NOTIFY_ESD_ON, NULL);
	return ret;
}

static ssize_t driver_test_write(
	struct file *file, const char __user *buf, size_t count, loff_t *pos)
{
	struct goodix_fw_version fw_ver;
	struct ts_rawdata_info *info = NULL;
	char *p = wbuf;
	int ret;

	if (count > SHORT_SIZE) {
		ts_err("invalid cmd size[%ld]", count);
		return count;
	}

	if (copy_from_user(p, buf, count) != 0) {
		ts_err("copy from user failed");
		return count;
	}

	kfree(rbuf);
	rbuf = NULL;
	index = 0;

	if (strstr(p, CMD_FW_UPDATE)) {
		rbuf = kzalloc(SHORT_SIZE, GFP_KERNEL);
		ret = goodix_do_fw_update(
			NULL, UPDATE_MODE_BLOCK | UPDATE_MODE_FORCE |
				      UPDATE_MODE_SRC_REQUEST);
		if (ret < 0) {
			index = sprintf(rbuf, "%s: NG\n", CMD_FW_UPDATE);
		} else {
			index = sprintf(rbuf, "%s: OK\n", CMD_FW_UPDATE);
		}
	} else if (strstr(p, CMD_GET_VERSION)) {
		rbuf = kzalloc(SHORT_SIZE, GFP_KERNEL);
		ret = cd->hw_ops->read_version(cd, &fw_ver);
		if (ret < 0) {
			index = sprintf(rbuf, "%s: NG\n", CMD_GET_VERSION);
		} else {
			index = sprintf(rbuf, "%s: %02x%02x%02x%02x\n",
				CMD_GET_VERSION, fw_ver.patch_vid[0],
				fw_ver.patch_vid[1], fw_ver.patch_vid[2],
				fw_ver.patch_vid[3]);
		}
	} else if (strstr(p, CMD_GET_RAWDATA)) {
		rbuf = kzalloc(LARGE_SIZE, GFP_KERNEL);
		ret = get_cap_data(CMD_GET_RAWDATA);
		if (ret < 0) {
			index = sprintf(rbuf, "%s: NG\n", CMD_GET_RAWDATA);
		}
	} else if (strstr(p, CMD_GET_DIFFDATA)) {
		rbuf = kzalloc(LARGE_SIZE, GFP_KERNEL);
		ret = get_cap_data(CMD_GET_DIFFDATA);
		if (ret < 0) {
			index = sprintf(rbuf, "%s: NG\n", CMD_GET_DIFFDATA);
		}
	} else if (strstr(p, CMD_GET_SELF_RAWDATA)) {
		rbuf = kzalloc(LARGE_SIZE, GFP_KERNEL);
		ret = get_cap_data(CMD_GET_SELF_RAWDATA);
		if (ret < 0) {
			index = sprintf(rbuf, "%s: NG\n", CMD_GET_SELF_RAWDATA);
		}
	} else if (strstr(p, CMD_GET_SELF_DIFFDATA)) {
		rbuf = kzalloc(LARGE_SIZE, GFP_KERNEL);
		ret = get_cap_data(CMD_GET_SELF_DIFFDATA);
		if (ret < 0) {
			index = sprintf(
				rbuf, "%s: NG\n", CMD_GET_SELF_DIFFDATA);
		}
	} else if (strstr(p, CMD_SET_DOUBLE_TAP)) {
		rbuf = kzalloc(SHORT_SIZE, GFP_KERNEL);
		if (strlen(p) - strlen(CMD_SET_DOUBLE_TAP) > 1) {
			if (p[strlen(CMD_SET_DOUBLE_TAP) + 1] == '0') {
				cd->gesture_type &= ~GESTURE_DOUBLE_TAP;
				index = sprintf(rbuf, "%s: disable OK\n",
					CMD_SET_DOUBLE_TAP);
			} else {
				cd->gesture_type |= GESTURE_DOUBLE_TAP;
				index = sprintf(rbuf, "%s: enable OK\n",
					CMD_SET_DOUBLE_TAP);
			}
		} else {
			index = sprintf(rbuf, "%s: NG\n", CMD_SET_DOUBLE_TAP);
		}
	} else if (strstr(p, CMD_SET_SINGLE_TAP)) {
		rbuf = kzalloc(SHORT_SIZE, GFP_KERNEL);
		if (strlen(p) - strlen(CMD_SET_SINGLE_TAP) > 1) {
			if (p[strlen(CMD_SET_SINGLE_TAP) + 1] == '0') {
				cd->gesture_type &= ~GESTURE_SINGLE_TAP;
				index = sprintf(rbuf, "%s: disable OK\n",
					CMD_SET_SINGLE_TAP);
			} else {
				cd->gesture_type |= GESTURE_SINGLE_TAP;
				index = sprintf(rbuf, "%s: enable OK\n",
					CMD_SET_SINGLE_TAP);
			}
		} else {
			index = sprintf(rbuf, "%s: NG\n", CMD_SET_SINGLE_TAP);
		}
	} else if (strstr(p, CMD_SET_IRQ_ENABLE)) {
		rbuf = kzalloc(SHORT_SIZE, GFP_KERNEL);
		if (strlen(p) - strlen(CMD_SET_IRQ_ENABLE) > 1) {
			if (p[strlen(CMD_SET_IRQ_ENABLE) + 1] == '0') {
				cd->hw_ops->irq_enable(cd, false);
				index = sprintf(rbuf, "%s: disable OK\n",
					CMD_SET_IRQ_ENABLE);
			} else {
				cd->hw_ops->irq_enable(cd, true);
				index = sprintf(rbuf, "%s: enable OK\n",
					CMD_SET_IRQ_ENABLE);
			}
		} else {
			index = sprintf(rbuf, "%s: NG\n", CMD_SET_IRQ_ENABLE);
		}
	} else if (strstr(p, CMD_SET_ESD_ENABLE)) {
		rbuf = kzalloc(SHORT_SIZE, GFP_KERNEL);
		if (strlen(p) - strlen(CMD_SET_ESD_ENABLE) > 1) {
			if (p[strlen(CMD_SET_ESD_ENABLE) + 1] == '0') {
				goodix_ts_blocking_notify(NOTIFY_ESD_OFF, NULL);
				index = sprintf(rbuf, "%s: disable OK\n",
					CMD_SET_ESD_ENABLE);
			} else {
				goodix_ts_blocking_notify(NOTIFY_ESD_ON, NULL);
				index = sprintf(rbuf, "%s: enable OK\n",
					CMD_SET_ESD_ENABLE);
			}
		} else {
			index = sprintf(rbuf, "%s: NG\n", CMD_SET_ESD_ENABLE);
		}
	} else if (strstr(p, CMD_SET_DEBUG_LOG)) {
		rbuf = kzalloc(SHORT_SIZE, GFP_KERNEL);
		if (strlen(p) - strlen(CMD_SET_DEBUG_LOG) > 1) {
			if (p[strlen(CMD_SET_DEBUG_LOG) + 1] == '0') {
				debug_log_flag = false;
				index = sprintf(rbuf, "%s: disable OK\n",
					CMD_SET_DEBUG_LOG);
			} else {
				debug_log_flag = true;
				index = sprintf(rbuf, "%s: enable OK\n",
					CMD_SET_DEBUG_LOG);
			}
		} else {
			index = sprintf(rbuf, "%s: NG\n", CMD_SET_DEBUG_LOG);
		}
	} else if (strstr(p, CMD_AUTO_TEST)) {
		rbuf = kzalloc(SHORT_SIZE, GFP_KERNEL);
		info = vmalloc(sizeof(*info));
		goodix_do_inspect(cd, info);
		index = sprintf(rbuf, "%s: %s", CMD_AUTO_TEST, info->result);
		vfree(info);
	} else {
		ts_err("not support cmd[%s]", p);
	}

	return count;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 6, 0))
static const struct proc_ops driver_test_ops = {
	.proc_open = driver_test_open,
	.proc_read = seq_read,
	.proc_write = driver_test_write,
	.proc_lseek = seq_lseek,
	.proc_release = driver_test_release,
};
#else
static const struct file_operations driver_test_ops = {
	.open = driver_test_open,
	.read = seq_read,
	.write = driver_test_write,
	.llseek = seq_lseek,
	.release = driver_test_release,
};
#endif

int driver_test_proc_init(struct goodix_ts_core *core_data)
{
	struct proc_dir_entry *proc_entry;

	proc_entry = proc_create(
		"goodix_ts/driver_test", 0777, NULL, &driver_test_ops);
	if (!proc_entry) {
		ts_err("failed to create proc entry");
		return -ENOMEM;
	}

	cd = core_data;
	return 0;
}

void driver_test_proc_remove(void)
{
	remove_proc_entry("goodix_ts/driver_test", NULL);
}