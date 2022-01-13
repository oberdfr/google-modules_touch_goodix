#include "goodix_ts_core.h"
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/version.h>

#define CMD_FW_UPDATE "fw_update"
#define CMD_AUTO_TEST "auto_test"
#define CMD_NOISE_TEST "noise_test"
#define CMD_GET_PACKAGE_ID "get_package_id"
#define CMD_GET_VERSION "get_version"
#define CMD_GET_RAWDATA "get_raw"
#define CMD_GET_DIFFDATA "get_diff"
#define CMD_GET_BASEDATA "get_base"
#define CMD_GET_SELF_RAWDATA "get_self_raw"
#define CMD_GET_SELF_DIFFDATA "get_self_diff"
#define CMD_GET_SELF_BASEDATA "get_self_base"
#define CMD_SET_DOUBLE_TAP "set_double_tap"
#define CMD_SET_SINGLE_TAP "set_single_tap"
#define CMD_SET_CHARGE_MODE "set_charge_mode"
#define CMD_SET_IRQ_ENABLE "set_irq_enable"
#define CMD_SET_ESD_ENABLE "set_esd_enable"
#define CMD_SET_DEBUG_LOG "set_debug_log"
#define CMD_SET_SCAN_MODE "set_scan_mode"
#define CMD_GET_CHANNEL_NUM "get_channel_num"
#define CMD_GET_TX_FREQ "get_tx_freq"
#define CMD_RESET "reset"
#define CMD_SET_SENSE_ENABLE "set_sense_enable"

#define SHORT_SIZE 100
#define LARGE_SIZE 4096
static struct goodix_ts_core *cd;
static char wbuf[SHORT_SIZE];
static char *rbuf;
static uint32_t index;

#define ABS(x) ((x >= 0) ? x : -x)

typedef struct __attribute__((packed)) {
	uint32_t checksum;
	uint32_t address;
	uint32_t length;
} flash_head_info_t;

#define FLASH_CMD_R_START 0x09
#define FLASH_CMD_W_START 0x0A
#define FLASH_CMD_RW_FINISH 0x0B
#define FLASH_CMD_STATE_READY 0x04
#define FLASH_CMD_STATE_CHECKERR 0x05
#define FLASH_CMD_STATE_DENY 0x06
#define FLASH_CMD_STATE_OKAY 0x07
static int goodix_flash_cmd(uint8_t cmd, uint8_t status, int retry_count)
{
	struct goodix_ts_cmd temp_cmd;
	int ret;
	int i;
	u8 r_sta;

	temp_cmd.len = 4;
	temp_cmd.cmd = cmd;
	ret = cd->hw_ops->send_cmd(cd, &temp_cmd);
	if (ret < 0)
		return ret;

	for (i = 0; i < retry_count; i++) {
		ret = cd->hw_ops->read(
			cd, cd->ic_info.misc.cmd_addr, &r_sta, 1);
		if (ret == 0 && r_sta == status)
			return 0;
	}

	ts_err("r_sta[0x%x] != status[0x%x]", r_sta, status);
	return -EINVAL;
}

static int goodix_flash_read(u32 addr, u8 *buf, int len)
{
	int i;
	int ret;
	u8 *tmp_buf;
	u32 buffer_addr = cd->ic_info.misc.fw_buffer_addr;
	struct goodix_ts_cmd temp_cmd;
	uint32_t checksum = 0;
	flash_head_info_t head_info;
	u8 *p = (u8 *)&head_info.address;

	tmp_buf = kzalloc(len + sizeof(flash_head_info_t), GFP_KERNEL);
	if (!tmp_buf)
		return -ENOMEM;

	head_info.address = cpu_to_le32(addr);
	head_info.length = cpu_to_le32(len);
	for (i = 0; i < 8; i += 2)
		checksum += p[i] | (p[i + 1] << 8);
	head_info.checksum = checksum;

	ret = goodix_flash_cmd(FLASH_CMD_R_START, FLASH_CMD_STATE_READY, 15);
	if (ret < 0) {
		ts_err("failed enter flash read state");
		goto read_end;
	}

	ret = cd->hw_ops->write(
		cd, buffer_addr, (u8 *)&head_info, sizeof(head_info));
	if (ret < 0) {
		ts_err("failed write flash head info");
		goto read_end;
	}

	ret = goodix_flash_cmd(FLASH_CMD_RW_FINISH, FLASH_CMD_STATE_OKAY, 50);
	if (ret) {
		ts_err("faild read flash ready state");
		goto read_end;
	}

	ret = cd->hw_ops->read(
		cd, buffer_addr, tmp_buf, len + sizeof(flash_head_info_t));
	if (ret < 0) {
		ts_err("failed read data len %lu",
			len + sizeof(flash_head_info_t));
		goto read_end;
	}

	checksum = 0;
	for (i = 0; i < len + sizeof(flash_head_info_t) - 4; i += 2)
		checksum += tmp_buf[4 + i] | (tmp_buf[5 + i] << 8);

	if (checksum != le32_to_cpup((__le32 *)tmp_buf)) {
		ts_err("read back data checksum error");
		ret = -EINVAL;
		goto read_end;
	}

	memcpy(buf, tmp_buf + sizeof(flash_head_info_t), len);
	ret = 0;
read_end:
	temp_cmd.len = 4;
	temp_cmd.cmd = 0x0C;
	cd->hw_ops->send_cmd(cd, &temp_cmd);
	return ret;
}

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

static int goodix_noise_test(uint16_t frame, int threshold)
{
	struct goodix_ts_cmd temp_cmd;
	int tx = cd->ic_info.parm.drv_num;
	int rx = cd->ic_info.parm.sen_num;
	s16 buf[17 * 35];
	u32 raw_addr;
	int retry = 50;
	u8 status;
	int ret;
	int i;
	int err_cnt = 0;

	raw_addr = cd->ic_info.misc.frame_data_addr +
		   cd->ic_info.misc.frame_data_head_len +
		   cd->ic_info.misc.fw_attr_len + cd->ic_info.misc.fw_log_len +
		   8;

	temp_cmd.len = 0x07;
	temp_cmd.cmd = 0x90;
	temp_cmd.data[0] = 0x86;
	temp_cmd.data[1] = frame & 0xFF;
	temp_cmd.data[2] = (frame >> 8) & 0xFF;
	ret = cd->hw_ops->send_cmd(cd, &temp_cmd);
	if (ret < 0) {
		ts_err("send noise cmd failed");
		goto fail;
	}
	msleep(frame * 10);

	while (retry--) {
		ret = cd->hw_ops->read(
			cd, cd->ic_info.misc.cmd_addr, &status, 1);
		if (ret == 0 && status == 0x80)
			break;
		msleep(50);
	}
	if (retry < 0) {
		ts_err("noise data is not ready");
		goto fail;
	}
	msleep(50);

	ret = cd->hw_ops->read(cd, raw_addr, (u8 *)buf, tx * rx * 2);
	if (ret < 0) {
		ts_err("read noise data faield");
		goto fail;
	}

	goodix_rotate_abcd2cbad(tx, rx, buf);
	for (i = 0; i < tx * rx; i++) {
		if (ABS(buf[i]) > threshold)
			err_cnt++;
		index += sprintf(&rbuf[index], "%3d,", buf[i]);
		if ((i + 1) % tx == 0)
			index += sprintf(&rbuf[index], "\n");
	}

	if (err_cnt > 0) {
		ts_err("noise test fail");
		goto fail;
	}

	ts_info("noise test pass");
	index += sprintf(&rbuf[index], "noise_test-[PASS]\n");
	return 0;

fail:
	index += sprintf(&rbuf[index], "noise_test-[FAIL]\n");
	return ret;
}

static int get_cap_data(uint8_t *type)
{
	struct goodix_ts_cmd temp_cmd;
	int tx = cd->ic_info.parm.drv_num;
	int rx = cd->ic_info.parm.sen_num;
	u8 val;
	int retry = 20;
	u8 frame_buf[GOODIX_MAX_FRAMEDATA_LEN];
	u32 flag_addr = cd->ic_info.misc.frame_data_addr;
	u32 mutual_addr;
	u32 self_addr;
	u32 tx_freq_addr;
	int i;
	int ret;

	mutual_addr = cd->ic_info.misc.frame_data_addr +
		      cd->ic_info.misc.frame_data_head_len +
		      cd->ic_info.misc.fw_attr_len +
		      cd->ic_info.misc.fw_log_len + 8;
	self_addr = cd->ic_info.misc.frame_data_addr +
		    cd->ic_info.misc.frame_data_head_len +
		    cd->ic_info.misc.fw_attr_len + cd->ic_info.misc.fw_log_len +
		    cd->ic_info.misc.mutual_struct_len + 10;
	tx_freq_addr = cd->ic_info.misc.frame_data_addr +
		       cd->ic_info.misc.frame_data_head_len +
		       cd->ic_info.misc.fw_attr_len +
		       cd->ic_info.misc.fw_log_len + 2;

	/* disable irq & close esd */
	cd->hw_ops->irq_enable(cd, false);
	goodix_ts_blocking_notify(NOTIFY_ESD_OFF, NULL);

	if (strstr(type, CMD_GET_BASEDATA) ||
		strstr(type, CMD_GET_SELF_BASEDATA)) {
		temp_cmd.data[0] = 0x83;
	} else if (strstr(type, CMD_GET_RAWDATA) ||
		   strstr(type, CMD_GET_SELF_RAWDATA) ||
		   strstr(type, CMD_GET_TX_FREQ)) {
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

	if (strstr(type, CMD_GET_TX_FREQ)) {
		ret = cd->hw_ops->read(cd, tx_freq_addr, frame_buf, 2);
		if (ret < 0) {
			ts_err("read frame data failed");
			goto exit;
		}

		index = sprintf(rbuf, "%s: %dHz\n", CMD_GET_TX_FREQ,
			le16_to_cpup((__le16 *)frame_buf) * 61);
		goto exit;
	}

	if (strstr(type, CMD_GET_RAWDATA) || strstr(type, CMD_GET_DIFFDATA) ||
		strstr(type, CMD_GET_BASEDATA)) {
		ret = cd->hw_ops->read(cd, mutual_addr, frame_buf, tx * rx * 2);
		if (ret < 0) {
			ts_err("read frame data failed");
			goto exit;
		}
		goodix_rotate_abcd2cbad(tx, rx, (s16 *)frame_buf);
		for (i = 0; i < tx * rx; i++) {
			index += sprintf(
				&rbuf[index], "%5d,", *((s16 *)frame_buf + i));
			if ((i + 1) % tx == 0)
				index += sprintf(&rbuf[index], "\n");
		}
	} else {
		ret = cd->hw_ops->read(cd, self_addr, frame_buf, (tx + rx) * 2);
		if (ret < 0) {
			ts_err("read frame data failed");
			goto exit;
		}
		index += sprintf(&rbuf[index], "TX:");
		for (i = 0; i < tx + rx; i++) {
			index += sprintf(
				&rbuf[index], "%5d,", *((s16 *)frame_buf + i));
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

static void goodix_set_sense_enable(bool on)
{
	static bool flag = true;

	if (on && !flag) {
		flag = true;
		cd->hw_ops->resume(cd);
		cd->hw_ops->irq_enable(cd, true);
		goodix_ts_blocking_notify(NOTIFY_ESD_ON, NULL);
		ts_info("set sense ON");
		return;
	} else if (!on && flag) {
		flag = false;
		goodix_ts_blocking_notify(NOTIFY_ESD_OFF, NULL);
		cd->hw_ops->irq_enable(cd, false);
		cd->hw_ops->suspend(cd);
		ts_info("set sense OFF");
		return;
	}

	ts_info("have already %s", on ? "ON" : "OFF");
}

static void goodix_set_scan_mode(u8 val)
{
	struct goodix_ts_cmd temp_cmd;

	temp_cmd.len = 5;
	temp_cmd.cmd = 0x9F;
	temp_cmd.data[0] = val;
	cd->hw_ops->send_cmd(cd, &temp_cmd);
}

static ssize_t driver_test_write(
	struct file *file, const char __user *buf, size_t count, loff_t *pos)
{
	struct goodix_fw_version fw_ver;
	struct goodix_ic_info ic_info;
	struct ts_rawdata_info *info = NULL;
	char *p = wbuf;
	char *token;
	int ret;
	int frames;
	int limit;
	int cmd_val;
	u8 id;

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

	ts_info("input cmd[%s]", p);

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
		goto exit;
	}

	if (strstr(p, CMD_GET_VERSION)) {
		rbuf = kzalloc(SHORT_SIZE, GFP_KERNEL);
		cd->hw_ops->read_version(cd, &fw_ver);
		cd->hw_ops->get_ic_info(cd, &ic_info);
		index = sprintf(rbuf, "%s: %02x%02x%02x%02x %x\n",
			CMD_GET_VERSION, fw_ver.patch_vid[0],
			fw_ver.patch_vid[1], fw_ver.patch_vid[2],
			fw_ver.patch_vid[3], ic_info.version.config_id);
		goto exit;
	}

	if (strstr(p, CMD_GET_RAWDATA)) {
		rbuf = kzalloc(LARGE_SIZE, GFP_KERNEL);
		ret = get_cap_data(CMD_GET_RAWDATA);
		if (ret < 0) {
			index = sprintf(rbuf, "%s: NG\n", CMD_GET_RAWDATA);
		}
		goto exit;
	}

	if (strstr(p, CMD_GET_BASEDATA)) {
		rbuf = kzalloc(LARGE_SIZE, GFP_KERNEL);
		ret = get_cap_data(CMD_GET_BASEDATA);
		if (ret < 0) {
			index = sprintf(rbuf, "%s: NG\n", CMD_GET_BASEDATA);
		}
		goto exit;
	}

	if (strstr(p, CMD_GET_DIFFDATA)) {
		rbuf = kzalloc(LARGE_SIZE, GFP_KERNEL);
		ret = get_cap_data(CMD_GET_DIFFDATA);
		if (ret < 0) {
			index = sprintf(rbuf, "%s: NG\n", CMD_GET_DIFFDATA);
		}
		goto exit;
	}

	if (strstr(p, CMD_GET_SELF_RAWDATA)) {
		rbuf = kzalloc(LARGE_SIZE, GFP_KERNEL);
		ret = get_cap_data(CMD_GET_SELF_RAWDATA);
		if (ret < 0) {
			index = sprintf(rbuf, "%s: NG\n", CMD_GET_SELF_RAWDATA);
		}
		goto exit;
	}

	if (strstr(p, CMD_GET_SELF_DIFFDATA)) {
		rbuf = kzalloc(LARGE_SIZE, GFP_KERNEL);
		ret = get_cap_data(CMD_GET_SELF_DIFFDATA);
		if (ret < 0) {
			index = sprintf(
				rbuf, "%s: NG\n", CMD_GET_SELF_DIFFDATA);
		}
		goto exit;
	}

	if (strstr(p, CMD_GET_SELF_BASEDATA)) {
		rbuf = kzalloc(LARGE_SIZE, GFP_KERNEL);
		ret = get_cap_data(CMD_GET_SELF_BASEDATA);
		if (ret < 0) {
			index = sprintf(
				rbuf, "%s: NG\n", CMD_GET_SELF_BASEDATA);
		}
		goto exit;
	}

	if (strstr(p, CMD_SET_DOUBLE_TAP)) {
		rbuf = kzalloc(SHORT_SIZE, GFP_KERNEL);
		token = strsep(&p, ",");
		if (!token || !p) {
			index = sprintf(rbuf, "%s: invalid cmd param\n",
				CMD_SET_DOUBLE_TAP);
			goto exit;
		}
		if (kstrtos32(p, 10, &cmd_val)) {
			index = sprintf(rbuf, "%s: invalid cmd param\n",
				CMD_SET_DOUBLE_TAP);
			goto exit;
		}
		if (cmd_val == 0) {
			cd->gesture_type &= ~GESTURE_DOUBLE_TAP;
			index = sprintf(
				rbuf, "%s: disable OK\n", CMD_SET_DOUBLE_TAP);
			ts_info("disable double tap");
		} else {
			cd->gesture_type |= GESTURE_DOUBLE_TAP;
			index = sprintf(
				rbuf, "%s: enable OK\n", CMD_SET_DOUBLE_TAP);
			ts_info("enable single tap");
		}
		goto exit;
	}

	if (strstr(p, CMD_SET_SINGLE_TAP)) {
		rbuf = kzalloc(SHORT_SIZE, GFP_KERNEL);
		token = strsep(&p, ",");
		if (!token || !p) {
			index = sprintf(rbuf, "%s: invalid cmd param\n",
				CMD_SET_SINGLE_TAP);
			goto exit;
		}
		if (kstrtos32(p, 10, &cmd_val)) {
			index = sprintf(rbuf, "%s: invalid cmd param\n",
				CMD_SET_SINGLE_TAP);
			goto exit;
		}
		if (cmd_val == 0) {
			cd->gesture_type &= ~GESTURE_SINGLE_TAP;
			index = sprintf(
				rbuf, "%s: disable OK\n", CMD_SET_SINGLE_TAP);
			ts_info("disable single tap");
		} else {
			cd->gesture_type |= GESTURE_SINGLE_TAP;
			index = sprintf(
				rbuf, "%s: enable OK\n", CMD_SET_SINGLE_TAP);
			ts_info("enable single tap");
		}
		goto exit;
	}

	if (strstr(p, CMD_SET_IRQ_ENABLE)) {
		rbuf = kzalloc(SHORT_SIZE, GFP_KERNEL);
		token = strsep(&p, ",");
		if (!token || !p) {
			index = sprintf(rbuf, "%s: invalid cmd param\n",
				CMD_SET_IRQ_ENABLE);
			goto exit;
		}
		if (kstrtos32(p, 10, &cmd_val)) {
			index = sprintf(rbuf, "%s: invalid cmd param\n",
				CMD_SET_IRQ_ENABLE);
			goto exit;
		}
		if (cmd_val == 0) {
			cd->hw_ops->irq_enable(cd, false);
			index = sprintf(
				rbuf, "%s: disable OK\n", CMD_SET_IRQ_ENABLE);
		} else {
			cd->hw_ops->irq_enable(cd, true);
			index = sprintf(
				rbuf, "%s: enable OK\n", CMD_SET_IRQ_ENABLE);
		}
		goto exit;
	}

	if (strstr(p, CMD_SET_ESD_ENABLE)) {
		rbuf = kzalloc(SHORT_SIZE, GFP_KERNEL);
		token = strsep(&p, ",");
		if (!token || !p) {
			index = sprintf(rbuf, "%s: invalid cmd param\n",
				CMD_SET_ESD_ENABLE);
			goto exit;
		}
		if (kstrtos32(p, 10, &cmd_val)) {
			index = sprintf(rbuf, "%s: invalid cmd param\n",
				CMD_SET_ESD_ENABLE);
			goto exit;
		}
		if (cmd_val == 0) {
			goodix_ts_blocking_notify(NOTIFY_ESD_OFF, NULL);
			index = sprintf(
				rbuf, "%s: disable OK\n", CMD_SET_ESD_ENABLE);
		} else {
			goodix_ts_blocking_notify(NOTIFY_ESD_ON, NULL);
			index = sprintf(
				rbuf, "%s: enable OK\n", CMD_SET_ESD_ENABLE);
		}
		goto exit;
	}

	if (strstr(p, CMD_SET_DEBUG_LOG)) {
		rbuf = kzalloc(SHORT_SIZE, GFP_KERNEL);
		token = strsep(&p, ",");
		if (!token || !p) {
			index = sprintf(rbuf, "%s: invalid cmd param\n",
				CMD_SET_DEBUG_LOG);
			goto exit;
		}
		if (kstrtos32(p, 10, &cmd_val)) {
			index = sprintf(rbuf, "%s: invalid cmd param\n",
				CMD_SET_DEBUG_LOG);
			goto exit;
		}
		if (cmd_val == 0) {
			debug_log_flag = false;
			index = sprintf(
				rbuf, "%s: disable OK\n", CMD_SET_DEBUG_LOG);
		} else {
			debug_log_flag = true;
			index = sprintf(
				rbuf, "%s: enable OK\n", CMD_SET_DEBUG_LOG);
		}
		goto exit;
	}

	if (strstr(p, CMD_AUTO_TEST)) {
		rbuf = kzalloc(SHORT_SIZE, GFP_KERNEL);
		info = vmalloc(sizeof(*info));
		goodix_do_inspect(cd, info);
		index = sprintf(rbuf, "%s", info->result);
		vfree(info);
		goto exit;
	}

	if (strstr(p, CMD_GET_CHANNEL_NUM)) {
		rbuf = kzalloc(SHORT_SIZE, GFP_KERNEL);
		index = sprintf(rbuf, "TX:%d RX:%d\n", cd->ic_info.parm.drv_num,
			cd->ic_info.parm.sen_num);
		goto exit;
	}

	if (strstr(p, CMD_GET_TX_FREQ)) {
		rbuf = kzalloc(SHORT_SIZE, GFP_KERNEL);
		ret = get_cap_data(CMD_GET_TX_FREQ);
		if (ret < 0) {
			index = sprintf(rbuf, "%s: NG\n", CMD_GET_TX_FREQ);
		}
		goto exit;
	}

	if (strstr(p, CMD_RESET)) {
		cd->hw_ops->irq_enable(cd, false);
		cd->hw_ops->reset(cd, 100);
		cd->hw_ops->irq_enable(cd, true);
		goto exit;
	}

	if (strstr(p, CMD_SET_SENSE_ENABLE)) {
		rbuf = kzalloc(SHORT_SIZE, GFP_KERNEL);
		token = strsep(&p, ",");
		if (!token || !p) {
			index = sprintf(rbuf, "%s: invalid cmd param\n",
				CMD_SET_SENSE_ENABLE);
			goto exit;
		}
		if (kstrtos32(p, 10, &cmd_val)) {
			index = sprintf(rbuf, "%s: invalid cmd param\n",
				CMD_SET_SENSE_ENABLE);
			goto exit;
		}
		if (cmd_val == 0) {
			goodix_set_sense_enable(false);
			index = sprintf(
				rbuf, "%s: OFF\n", CMD_SET_SENSE_ENABLE);
		} else {
			goodix_set_sense_enable(true);
			index = sprintf(rbuf, "%s: ON\n", CMD_SET_SENSE_ENABLE);
		}
		goto exit;
	}

	if (strstr(p, CMD_NOISE_TEST)) {
		rbuf = kzalloc(LARGE_SIZE, GFP_KERNEL);
		token = strsep(&p, ",");
		if (!token || !p) {
			index = sprintf(rbuf, "%s: invalid cmd param\n",
				CMD_NOISE_TEST);
			goto exit;
		}
		token = strsep(&p, ",");
		if (!token || !p) {
			index = sprintf(rbuf, "%s: invalid cmd param\n",
				CMD_NOISE_TEST);
			goto exit;
		}
		if (kstrtos32(token, 10, &frames)) {
			index = sprintf(rbuf, "%s: invalid cmd param\n",
				CMD_NOISE_TEST);
			goto exit;
		}
		if (kstrtos32(p, 10, &limit)) {
			index = sprintf(rbuf, "%s: invalid cmd param\n",
				CMD_NOISE_TEST);
			goto exit;
		}

		ts_info("noise test frames:%d limit:%d", frames, limit);
		cd->hw_ops->irq_enable(cd, false);
		goodix_ts_blocking_notify(NOTIFY_ESD_OFF, NULL);
		goodix_noise_test(frames, limit);
		cd->hw_ops->irq_enable(cd, true);
		goodix_ts_blocking_notify(NOTIFY_ESD_ON, NULL);
		goto exit;
	}

	if (strstr(p, CMD_GET_PACKAGE_ID)) {
		rbuf = kzalloc(SHORT_SIZE, GFP_KERNEL);
		ret = goodix_flash_read(0x1F301, &id, 1);
		if (ret < 0)
			index = sprintf(rbuf, "%s: NG\n", CMD_GET_PACKAGE_ID);
		else
			index = sprintf(
				rbuf, "%s: 0x%x\n", CMD_GET_PACKAGE_ID, id);
		goto exit;
	}

	if (strstr(p, CMD_SET_SCAN_MODE)) {
		rbuf = kzalloc(SHORT_SIZE, GFP_KERNEL);
		token = strsep(&p, ",");
		if (!token || !p) {
			index = sprintf(rbuf, "%s: invalid cmd param\n",
				CMD_SET_SCAN_MODE);
			goto exit;
		}
		if (kstrtos32(p, 10, &cmd_val)) {
			index = sprintf(rbuf, "%s: invalid cmd param\n",
				CMD_SET_SCAN_MODE);
			goto exit;
		}
		if (cmd_val == 0) {
			goodix_set_scan_mode(0);
			index = sprintf(
				rbuf, "%s: exit idle\n", CMD_SET_SCAN_MODE);
		} else {
			goodix_set_scan_mode(1);
			index = sprintf(
				rbuf, "%s: enter idle\n", CMD_SET_SCAN_MODE);
		}
		goto exit;
	}

	rbuf = kzalloc(SHORT_SIZE, GFP_KERNEL);
	index = sprintf(rbuf, "not support cmd %s", p);
	ts_err("not support cmd[%s]", p);
exit:
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