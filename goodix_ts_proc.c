#include "goodix_ts_core.h"
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/version.h>

#define CMD_FW_UPDATE "fw_update"
#define CMD_AUTO_TEST "auto_test"
#define CMD_OPEN_TEST "open_test"
#define CMD_SELF_OPEN_TEST "self_open_test"
#define CMD_NOISE_TEST "noise_test"
#define CMD_SHORT_TEST "short_test"
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
#define CMD_SET_CONTINUE_MODE "set_continue_mode"
#define CMD_GET_CHANNEL_NUM "get_channel_num"
#define CMD_GET_TX_FREQ "get_tx_freq"
#define CMD_RESET "reset"
#define CMD_SET_SENSE_ENABLE "set_sense_enable"

/* test limits keyword */
#define CSV_TP_SPECIAL_RAW_MIN "special_raw_min"
#define CSV_TP_SPECIAL_RAW_MAX "special_raw_max"
#define CSV_TP_SPECIAL_RAW_DELTA "special_raw_delta"
#define CSV_TP_SHORT_THRESHOLD "shortciurt_threshold"
#define CSV_TP_SPECIAL_SELFRAW_MAX "special_selfraw_max"
#define CSV_TP_SPECIAL_SELFRAW_MIN "special_selfraw_min"
#define CSV_TP_NOISE_LIMIT "noise_data_limit"
#define CSV_TP_SELFNOISE_LIMIT "noise_selfdata_limit"
#define CSV_TP_TEST_CONFIG "test_config"

#define SHORT_SIZE 100
#define LARGE_SIZE 4096
#define MAX_FRAME_CNT 50
#define HUGE_SIZE MAX_FRAME_CNT * 20 * 1024
static struct goodix_ts_core *cd;
static char wbuf[SHORT_SIZE];
static char *rbuf;
static uint32_t index;

/* factory test */
#define ABS(x) ((x >= 0) ? x : -x)
#define MAX(a, b) ((a > b) ? a : b)

#define GTP_CAP_TEST 1
#define GTP_DELTA_TEST 2
#define GTP_NOISE_TEST 3
#define GTP_SHORT_TEST 5
#define GTP_SELFCAP_TEST 6
#define GTP_SELFNOISE_TEST 7
#define MAX_TEST_ITEMS 10 /* 0P-1P-2P-3P-5P total test items */

#define TEST_OK 1
#define TEST_NG 0

#define MAX_LINE_LEN (1024 * 6)
#define MAX_DRV_NUM 17
#define MAX_SEN_NUM 35
#define MAX_SHORT_NUM 15

typedef struct __attribute__((packed)) {
	u8 result;
	u8 drv_drv_num;
	u8 sen_sen_num;
	u8 drv_sen_num;
	u8 drv_gnd_avdd_num;
	u8 sen_gnd_avdd_num;
	u16 checksum;
} test_result_t;

/* nottingham drv-sen map */
static u8 not_drv_map[] = { 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47,
	48, 49, 50, 51 };

static u8 not_sen_map[] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14,
	15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32,
	33, 34 };

struct ts_short_res {
	u8 short_num;
	s16 short_msg[4 * MAX_SHORT_NUM];
};

struct ts_test_rawdata {
	s16 data[MAX_DRV_NUM * MAX_SEN_NUM];
	u32 size;
};

struct ts_test_self_rawdata {
	s16 data[MAX_DRV_NUM + MAX_SEN_NUM];
	u32 size;
};

struct goodix_ts_test {
	bool item[MAX_TEST_ITEMS];
	char result[MAX_TEST_ITEMS];
	s16 min_limits[MAX_DRV_NUM * MAX_SEN_NUM];
	s16 max_limits[MAX_DRV_NUM * MAX_SEN_NUM];
	s16 deviation_limits[MAX_DRV_NUM * MAX_SEN_NUM];
	s16 self_max_limits[MAX_DRV_NUM + MAX_SEN_NUM];
	s16 self_min_limits[MAX_DRV_NUM + MAX_SEN_NUM];
	s16 noise_threshold;
	s16 short_threshold;
	s16 r_drv_drv_threshold;
	s16 r_drv_sen_threshold;
	s16 r_sen_sen_threshold;
	s16 r_drv_gnd_threshold;
	s16 r_sen_gnd_threshold;
	s16 avdd_value;

	int freq;
	int raw_data_cnt;
	struct ts_test_rawdata rawdata[MAX_FRAME_CNT];
	struct ts_test_rawdata deltadata[MAX_FRAME_CNT];
	int noise_data_cnt;
	struct ts_test_rawdata noisedata[MAX_FRAME_CNT];
	struct ts_test_self_rawdata selfrawdata;
	struct ts_short_res short_res;
};
static struct goodix_ts_test *ts_test;

static int malloc_test_resource(void)
{
	ts_test = kzalloc(sizeof(*ts_test), GFP_KERNEL);
	if (!ts_test)
		return -ENOMEM;
	return 0;
}

static void release_test_resource(void)
{
	kfree(ts_test);
	ts_test = NULL;
}

#define CHN_VDD 0xFF
#define CHN_GND 0x7F
#define DRV_CHANNEL_FLAG 0x80
#define SHORT_TEST_TIME_REG_NOT 0x1479E
#define SHORT_TEST_STATUS_REG_NOT 0x13400
#define SHORT_TEST_RESULT_REG_NOT 0x13408
#define DRV_DRV_SELFCODE_REG_NOT 0x13446
#define SEN_SEN_SELFCODE_REG_NOT 0x136EE
#define DRV_SEN_SELFCODE_REG_NOT 0x14152
#define DIFF_CODE_DATA_REG_NOT 0x14734
#define CAL_CHAN_TO_CHAN_RES(v1, v2) (v1 / v2 - 1) * 55 + 45
#define CAL_CHAN_TO_AVDD_RES(v1, v2) 64 * (2 * v2 - 25) * 76 / v1 - 15
#define CAL_CHAN_TO_GND_RES(v) 120000 / v - 16
static u32 map_die2pin(u32 chn_num)
{
	int i = 0;
	u32 res = 255;

	if (chn_num & DRV_CHANNEL_FLAG)
		chn_num = (chn_num & ~DRV_CHANNEL_FLAG) + MAX_SEN_NUM;

	for (i = 0; i < MAX_SEN_NUM; i++) {
		if (not_sen_map[i] == chn_num) {
			res = i;
			break;
		}
	}
	/* res != 255 mean found the corresponding channel num */
	if (res != 255)
		return res;
	/* if cannot find in SenMap try find in DrvMap */
	for (i = 0; i < MAX_DRV_NUM; i++) {
		if (not_drv_map[i] == chn_num) {
			res = i;
			break;
		}
	}
	if (i >= MAX_DRV_NUM)
		ts_err("Faild found corrresponding channel num:%d", chn_num);
	else
		res |= DRV_CHANNEL_FLAG;

	return res;
}

static void goodix_save_short_res(u16 chn1, u16 chn2, int r)
{
	int i;
	u8 repeat_cnt = 0;
	u8 repeat = 0;
	struct ts_short_res *short_res = &ts_test->short_res;

	if (chn1 == chn2 || short_res->short_num >= MAX_SHORT_NUM)
		return;

	for (i = 0; i < short_res->short_num; i++) {
		repeat_cnt = 0;
		if (short_res->short_msg[4 * i] == chn1)
			repeat_cnt++;
		if (short_res->short_msg[4 * i] == chn2)
			repeat_cnt++;
		if (short_res->short_msg[4 * i + 1] == chn1)
			repeat_cnt++;
		if (short_res->short_msg[4 * i + 1] == chn2)
			repeat_cnt++;
		if (repeat_cnt >= 2) {
			repeat = 1;
			break;
		}
	}
	if (repeat == 0) {
		short_res->short_msg[4 * short_res->short_num + 0] = chn1;
		short_res->short_msg[4 * short_res->short_num + 1] = chn2;
		short_res->short_msg[4 * short_res->short_num + 2] =
			(r >> 8) & 0xFF;
		short_res->short_msg[4 * short_res->short_num + 3] = r & 0xFF;
		if (short_res->short_num < MAX_SHORT_NUM)
			short_res->short_num++;
	}
}

static int gdix_check_tx_tx_shortcircut(u8 short_ch_num)
{
	int ret = 0, err = 0;
	u32 r_threshold = 0, short_r = 0;
	int size = 0, i = 0, j = 0;
	u16 adc_signal = 0;
	u8 master_pin_num, slave_pin_num;
	u8 *data_buf;
	u32 data_reg = DRV_DRV_SELFCODE_REG_NOT;
	int max_drv_num = MAX_DRV_NUM;
	int max_sen_num = MAX_SEN_NUM;
	u16 self_capdata, short_die_num = 0;

	size = 4 + max_drv_num * 2 + 2;
	data_buf = kzalloc(size, GFP_KERNEL);
	if (!data_buf) {
		ts_err("Failed to alloc memory");
		return -ENOMEM;
	}
	/* drv&drv shortcircut check */
	for (i = 0; i < short_ch_num; i++) {
		ret = cd->hw_ops->read(cd, data_reg, data_buf, size);
		if (ret < 0) {
			ts_err("Failed read Drv-to-Drv short rawdata");
			err = -EINVAL;
			break;
		}

		if (checksum_cmp(data_buf, size, CHECKSUM_MODE_U8_LE)) {
			ts_err("Drv-to-Drv adc data checksum error");
			err = -EINVAL;
			break;
		}

		r_threshold = ts_test->r_drv_drv_threshold;
		short_die_num = le16_to_cpup((__le16 *)&data_buf[0]);
		short_die_num -= max_sen_num;
		if (short_die_num >= max_drv_num) {
			ts_info("invalid short pad num:%d",
				short_die_num + max_sen_num);
			continue;
		}

		/* TODO: j start position need recheck */
		self_capdata = le16_to_cpup((__le16 *)&data_buf[2]);
		if (self_capdata == 0xffff || self_capdata == 0) {
			ts_info("invalid self_capdata:0x%x", self_capdata);
			continue;
		}

		for (j = short_die_num + 1; j < max_drv_num; j++) {
			adc_signal =
				le16_to_cpup((__le16 *)&data_buf[4 + j * 2]);

			if (adc_signal < ts_test->short_threshold)
				continue;

			short_r =
				CAL_CHAN_TO_CHAN_RES(self_capdata, adc_signal);
			if (short_r < r_threshold) {
				master_pin_num = map_die2pin(
					short_die_num + max_sen_num);
				slave_pin_num = map_die2pin(j + max_sen_num);
				if (master_pin_num == 0xFF ||
					slave_pin_num == 0xFF) {
					ts_info("WARNNING invalid pin");
					continue;
				}
				goodix_save_short_res(
					master_pin_num, slave_pin_num, short_r);
				ts_err("short circut:R=%dK,R_Threshold=%dK",
					short_r, r_threshold);
				ts_err("%s%d--%s%d shortcircut",
					(master_pin_num & DRV_CHANNEL_FLAG)
						? "DRV"
						: "SEN",
					(master_pin_num & ~DRV_CHANNEL_FLAG),
					(slave_pin_num & DRV_CHANNEL_FLAG)
						? "DRV"
						: "SEN",
					(slave_pin_num & ~DRV_CHANNEL_FLAG));
				err = -EINVAL;
			}
		}
		data_reg += size;
	}

	kfree(data_buf);
	return err;
}

static int gdix_check_rx_rx_shortcircut(u8 short_ch_num)
{
	int ret = 0, err = 0;
	u32 r_threshold = 0, short_r = 0;
	int size = 0, i = 0, j = 0;
	u16 adc_signal = 0;
	u8 master_pin_num, slave_pin_num;
	u8 *data_buf;
	u32 data_reg = SEN_SEN_SELFCODE_REG_NOT;
	int max_sen_num = MAX_SEN_NUM;
	u16 self_capdata, short_die_num = 0;

	size = 4 + max_sen_num * 2 + 2;
	data_buf = kzalloc(size, GFP_KERNEL);
	if (!data_buf) {
		ts_err("Failed to alloc memory");
		return -ENOMEM;
	}
	/* drv&drv shortcircut check */
	for (i = 0; i < short_ch_num; i++) {
		ret = cd->hw_ops->read(cd, data_reg, data_buf, size);
		if (ret) {
			ts_err("Failed read Sen-to-Sen short rawdata");
			err = -EINVAL;
			break;
		}

		if (checksum_cmp(data_buf, size, CHECKSUM_MODE_U8_LE)) {
			ts_err("Sen-to-Sen adc data checksum error");
			err = -EINVAL;
			break;
		}

		r_threshold = ts_test->r_sen_sen_threshold;
		short_die_num = le16_to_cpup((__le16 *)&data_buf[0]);
		if (short_die_num >= max_sen_num) {
			ts_info("invalid short pad num:%d", short_die_num);
			continue;
		}

		/* TODO: j start position need recheck */
		self_capdata = le16_to_cpup((__le16 *)&data_buf[2]);
		if (self_capdata == 0xffff || self_capdata == 0) {
			ts_info("invalid self_capdata:0x%x", self_capdata);
			continue;
		}

		for (j = short_die_num + 1; j < max_sen_num; j++) {
			adc_signal =
				le16_to_cpup((__le16 *)&data_buf[4 + j * 2]);

			if (adc_signal < ts_test->short_threshold)
				continue;

			short_r =
				CAL_CHAN_TO_CHAN_RES(self_capdata, adc_signal);
			if (short_r < r_threshold) {
				master_pin_num = map_die2pin(short_die_num);
				slave_pin_num = map_die2pin(j);
				if (master_pin_num == 0xFF ||
					slave_pin_num == 0xFF) {
					ts_info("WARNNING invalid pin");
					continue;
				}
				goodix_save_short_res(
					master_pin_num, slave_pin_num, short_r);
				ts_err("short circut:R=%dK,R_Threshold=%dK",
					short_r, r_threshold);
				ts_err("%s%d--%s%d shortcircut",
					(master_pin_num & DRV_CHANNEL_FLAG)
						? "DRV"
						: "SEN",
					(master_pin_num & ~DRV_CHANNEL_FLAG),
					(slave_pin_num & DRV_CHANNEL_FLAG)
						? "DRV"
						: "SEN",
					(slave_pin_num & ~DRV_CHANNEL_FLAG));
				err = -EINVAL;
			}
		}
		data_reg += size;
	}

	kfree(data_buf);
	return err;
}

static int gdix_check_tx_rx_shortcircut(u8 short_ch_num)
{
	int ret = 0, err = 0;
	u32 r_threshold = 0, short_r = 0;
	int size = 0, i = 0, j = 0;
	u16 adc_signal = 0;
	u8 master_pin_num, slave_pin_num;
	u8 *data_buf = NULL;
	u32 data_reg = DRV_SEN_SELFCODE_REG_NOT;
	int max_drv_num = MAX_DRV_NUM;
	int max_sen_num = MAX_SEN_NUM;
	u16 self_capdata, short_die_num = 0;

	size = 4 + max_drv_num * 2 + 2;
	data_buf = kzalloc(size, GFP_KERNEL);
	if (!data_buf) {
		ts_err("Failed to alloc memory");
		return -ENOMEM;
	}
	/* drv&sen shortcircut check */
	for (i = 0; i < short_ch_num; i++) {
		ret = cd->hw_ops->read(cd, data_reg, data_buf, size);
		if (ret) {
			ts_err("Failed read Drv-to-Sen short rawdata");
			err = -EINVAL;
			break;
		}

		if (checksum_cmp(data_buf, size, CHECKSUM_MODE_U8_LE)) {
			ts_err("Drv-to-Sen adc data checksum error");
			err = -EINVAL;
			break;
		}

		r_threshold = ts_test->r_drv_sen_threshold;
		short_die_num = le16_to_cpup((__le16 *)&data_buf[0]);
		if (short_die_num >= max_sen_num) {
			ts_info("invalid short pad num:%d", short_die_num);
			continue;
		}

		/* TODO: j start position need recheck */
		self_capdata = le16_to_cpup((__le16 *)&data_buf[2]);
		if (self_capdata == 0xffff || self_capdata == 0) {
			ts_info("invalid self_capdata:0x%x", self_capdata);
			continue;
		}

		for (j = 0; j < max_drv_num; j++) {
			adc_signal =
				le16_to_cpup((__le16 *)&data_buf[4 + j * 2]);

			if (adc_signal < ts_test->short_threshold)
				continue;

			short_r =
				CAL_CHAN_TO_CHAN_RES(self_capdata, adc_signal);
			if (short_r < r_threshold) {
				master_pin_num = map_die2pin(short_die_num);
				slave_pin_num = map_die2pin(j + max_sen_num);
				if (master_pin_num == 0xFF ||
					slave_pin_num == 0xFF) {
					ts_info("WARNNING invalid pin");
					continue;
				}
				goodix_save_short_res(
					master_pin_num, slave_pin_num, short_r);
				ts_err("short circut:R=%dK,R_Threshold=%dK",
					short_r, r_threshold);
				ts_err("%s%d--%s%d shortcircut",
					(master_pin_num & DRV_CHANNEL_FLAG)
						? "DRV"
						: "SEN",
					(master_pin_num & ~DRV_CHANNEL_FLAG),
					(slave_pin_num & DRV_CHANNEL_FLAG)
						? "DRV"
						: "SEN",
					(slave_pin_num & ~DRV_CHANNEL_FLAG));
				err = -EINVAL;
			}
		}
		data_reg += size;
	}

	kfree(data_buf);
	return err;
}

static int gdix_check_resistance_to_gnd(u16 adc_signal, u32 pos)
{
	long r = 0;
	u16 r_th = 0, avdd_value = 0;
	u16 chn_id_tmp = 0;
	u8 pin_num = 0;
	unsigned short short_type;
	int max_drv_num = MAX_DRV_NUM;
	int max_sen_num = MAX_SEN_NUM;

	avdd_value = ts_test->avdd_value;
	short_type = adc_signal & 0x8000;
	adc_signal &= ~0x8000;
	if (adc_signal == 0)
		adc_signal = 1;

	if (short_type == 0) {
		/* short to GND */
		r = CAL_CHAN_TO_GND_RES(adc_signal);
	} else {
		/* short to VDD */
		r = CAL_CHAN_TO_AVDD_RES(adc_signal, avdd_value);
	}

	if (pos < max_drv_num)
		r_th = ts_test->r_drv_gnd_threshold;
	else
		r_th = ts_test->r_sen_gnd_threshold;

	chn_id_tmp = pos;
	if (chn_id_tmp < max_drv_num)
		chn_id_tmp += max_sen_num;
	else
		chn_id_tmp -= max_drv_num;

	if (r < r_th) {
		pin_num = map_die2pin(chn_id_tmp);
		goodix_save_short_res(
			pin_num, short_type ? CHN_VDD : CHN_GND, r);
		ts_err("%s%d shortcircut to %s,R=%ldK,R_Threshold=%dK",
			(pin_num & DRV_CHANNEL_FLAG) ? "DRV" : "SEN",
			(pin_num & ~DRV_CHANNEL_FLAG),
			short_type ? "VDD" : "GND", r, r_th);

		return -EINVAL;
	}

	return 0;
}

static int gdix_check_gndvdd_shortcircut(void)
{
	int ret = 0, err = 0;
	int size = 0, i = 0;
	u16 adc_signal = 0;
	u32 data_reg = DIFF_CODE_DATA_REG_NOT;
	u8 *data_buf = NULL;
	int max_drv_num = MAX_DRV_NUM;
	int max_sen_num = MAX_SEN_NUM;

	size = (max_drv_num + max_sen_num) * 2 + 2;
	data_buf = kzalloc(size, GFP_KERNEL);
	if (!data_buf) {
		ts_err("Failed to alloc memory");
		return -ENOMEM;
	}
	/* read diff code, diff code will be used to calculate
	 * resistance between channel and GND */
	ret = cd->hw_ops->read(cd, data_reg, data_buf, size);
	if (ret < 0) {
		ts_err("Failed read to-gnd rawdata");
		err = -EINVAL;
		goto err_out;
	}

	if (checksum_cmp(data_buf, size, CHECKSUM_MODE_U8_LE)) {
		ts_err("diff code checksum error");
		err = -EINVAL;
		goto err_out;
	}

	for (i = 0; i < max_drv_num + max_sen_num; i++) {
		adc_signal = le16_to_cpup((__le16 *)&data_buf[i * 2]);
		ret = gdix_check_resistance_to_gnd(adc_signal, i);
		if (ret != 0) {
			ts_err("Resistance to-gnd/vdd short");
			err = ret;
		}
	}

err_out:
	kfree(data_buf);
	return err;
}

static int goodix_shortcircut_analysis(void)
{
	int ret;
	int err = 0;
	test_result_t test_result;

	ret = cd->hw_ops->read(cd, SHORT_TEST_RESULT_REG_NOT,
		(u8 *)&test_result, sizeof(test_result));
	if (ret < 0) {
		ts_err("Read TEST_RESULT_REG failed");
		return ret;
	}

	if (checksum_cmp((u8 *)&test_result, sizeof(test_result),
		    CHECKSUM_MODE_U8_LE)) {
		ts_err("shrot result checksum err");
		return -EINVAL;
	}

	if (!(test_result.result & 0x0F)) {
		ts_info(">>>>> No shortcircut");
		return 0;
	}
	ts_info("short flag 0x%02x, drv&drv:%d, sen&sen:%d, drv&sen:%d, drv/GNDVDD:%d, sen/GNDVDD:%d",
		test_result.result, test_result.drv_drv_num,
		test_result.sen_sen_num, test_result.drv_sen_num,
		test_result.drv_gnd_avdd_num, test_result.sen_gnd_avdd_num);

	if (test_result.drv_drv_num)
		err |= gdix_check_tx_tx_shortcircut(test_result.drv_drv_num);
	if (test_result.sen_sen_num)
		err |= gdix_check_rx_rx_shortcircut(test_result.sen_sen_num);
	if (test_result.drv_sen_num)
		err |= gdix_check_tx_rx_shortcircut(test_result.drv_sen_num);
	if (test_result.drv_gnd_avdd_num || test_result.sen_gnd_avdd_num)
		err |= gdix_check_gndvdd_shortcircut();

	ts_info(">>>>> short check return 0x%x", err);

	return err;
}

#define INSPECT_FW_SWITCH_CMD 0x85
#define SHORT_TEST_RUN_FLAG 0xAA
#define SHORT_TEST_RUN_REG 0x10400
static int goodix_short_test_prepare(void)
{
	struct goodix_ts_cmd tmp_cmd;
	int ret;
	int retry;
	int resend = 3;
	u8 status;

	ts_info("short test prepare IN");
	tmp_cmd.len = 4;
	tmp_cmd.cmd = INSPECT_FW_SWITCH_CMD;

resend_cmd:
	ret = cd->hw_ops->send_cmd(cd, &tmp_cmd);
	if (ret < 0) {
		ts_err("send test mode failed");
		return ret;
	}

	retry = 3;
	while (retry--) {
		msleep(40);
		ret = cd->hw_ops->read(cd, SHORT_TEST_RUN_REG, &status, 1);
		if (!ret && status == SHORT_TEST_RUN_FLAG)
			return 0;
		ts_info("short_mode_status=0x%02x ret=%d", status, ret);
	}

	if (resend--) {
		cd->hw_ops->reset(cd, 100);
		goto resend_cmd;
	}

	return -EINVAL;
}

#define MAX_TEST_TIME_MS 15000
#define DEFAULT_TEST_TIME_MS 7000
#define SHORT_TEST_FINISH_FLAG 0x88
static int goodix_shortcircut_test(void)
{
	int ret = 0;
	int res;
	int retry;
	u16 test_time;
	u8 status;

	ts_info("---------------------- short_test begin ----------------------");
	ret = goodix_short_test_prepare();
	if (ret < 0) {
		ts_err("Failed enter short test mode");
		return ret;
	}

	/* get short test time */
	ret = cd->hw_ops->read(
		cd, SHORT_TEST_TIME_REG_NOT, (u8 *)&test_time, 2);
	if (ret < 0) {
		ts_err("Failed to get test_time, default %dms",
			DEFAULT_TEST_TIME_MS);
		test_time = DEFAULT_TEST_TIME_MS;
	} else {
		if (test_time > MAX_TEST_TIME_MS) {
			ts_info("test time too long %d > %d", test_time,
				MAX_TEST_TIME_MS);
			test_time = MAX_TEST_TIME_MS;
		}
		ts_info("get test time %dms", test_time);
	}

	/* start short test */
	status = 0;
	cd->hw_ops->write(cd, SHORT_TEST_RUN_REG, &status, 1);

	/* wait short test finish */
	msleep(test_time);
	retry = 50;
	while (retry--) {
		ret = cd->hw_ops->read(
			cd, SHORT_TEST_STATUS_REG_NOT, &status, 1);
		if (!ret && status == SHORT_TEST_FINISH_FLAG)
			break;
		msleep(50);
	}
	if (retry < 0) {
		ts_err("short test failed, status:0x%02x", status);
		return -EINVAL;
	}

	/* start analysis short result */
	ts_info("short_test finished, start analysis");
	res = goodix_shortcircut_analysis();
	if (res == 0) {
		ts_test->result[GTP_SHORT_TEST] = TEST_OK;
		ret = 0;
	}

	return ret;
}

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
		release_test_resource();
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

static void goodix_save_header(void)
{
	int i;
	bool total_result = true;

	index = sprintf(
		&rbuf[index], "<?xml version=\"1.0\" encoding=\"utf-8\"?>\n");
	index += sprintf(&rbuf[index], "<TESTLOG>\n");
	index += sprintf(&rbuf[index], "<Header>\n");
	/* sava test result */
	for (i = 0; i < MAX_TEST_ITEMS; i++) {
		if (ts_test->item[i] && ts_test->result[i] == TEST_NG)
			total_result = false;
	}
	if (total_result == false) {
		index += sprintf(&rbuf[index], "<Result>NG</Result>\n");
	} else {
		index += sprintf(&rbuf[index], "<Result>OK</Result>\n");
	}

	index += sprintf(&rbuf[index], "<DeviceType>GT%s</DeviceType>\n",
		cd->fw_version.patch_pid);
	index += sprintf(&rbuf[index], "<SensorId>%d</SensorId>\n",
		cd->fw_version.sensor_id);
	index += sprintf(&rbuf[index], "</Header>\n");

	index += sprintf(&rbuf[index], "<ItemList>\n");
	if (ts_test->item[GTP_CAP_TEST]) {
		if (ts_test->result[GTP_CAP_TEST] == TEST_NG) {
			index += sprintf(&rbuf[index],
				"<Item name=\"Rawdata MAX/MIN Test\" result=\"NG\"/>\n");
		} else {
			index += sprintf(&rbuf[index],
				"<Item name=\"Rawdata MAX/MIN Test\" result=\"OK\"/>\n");
		}
		if (ts_test->result[GTP_DELTA_TEST] == TEST_NG) {
			index += sprintf(&rbuf[index],
				"<Item name=\"Rawdata Adjcent Deviation Test\" result=\"NG\"/>\n");
		} else {
			index += sprintf(&rbuf[index],
				"<Item name=\"Rawdata Adjcent Deviation Test\" result=\"OK\"/>\n");
		}
	}

	if (ts_test->item[GTP_NOISE_TEST]) {
		if (ts_test->result[GTP_NOISE_TEST] == TEST_NG) {
			index += sprintf(&rbuf[index],
				"<Item name=\"Diffdata Jitter Test\" result=\"NG\"/>\n");
		} else {
			index += sprintf(&rbuf[index],
				"<Item name=\"Diffdata Jitter Test\" result=\"OK\"/>\n");
		}
	}

	if (ts_test->item[GTP_SELFCAP_TEST]) {
		if (ts_test->result[GTP_SELFCAP_TEST] == TEST_NG) {
			index += sprintf(&rbuf[index],
				"<Item name=\"Self Rawdata Upper Limit Test\" result=\"NG\"/>\n");
		} else {
			index += sprintf(&rbuf[index],
				"<Item name=\"Self Rawdata Upper Limit Test\" result=\"OK\"/>\n");
		}
	}

	if (ts_test->item[GTP_SHORT_TEST]) {
		if (ts_test->result[GTP_SHORT_TEST] == TEST_NG) {
			index += sprintf(&rbuf[index],
				"<Item name=\"Short Test\" result=\"NG\"/>\n");
		} else {
			index += sprintf(&rbuf[index],
				"<Item name=\"Short Test\" result=\"OK\"/>\n");
		}
	}
	index += sprintf(&rbuf[index], "</ItemList>\n");
}

static void goodix_save_limits(void)
{
	int tx = cd->ic_info.parm.drv_num;
	int rx = cd->ic_info.parm.sen_num;
	int i;
	int chn1;
	int chn2;
	int r;

	index += sprintf(&rbuf[index], "<TestItems>\n");

	/* save short result */
	if (ts_test->item[GTP_SHORT_TEST]) {
		index += sprintf(&rbuf[index], "<Item name=\"Short Test\">\n");
		index += sprintf(&rbuf[index], "<ShortNum>%d</ShortNum>\n",
			ts_test->short_res.short_num);
		for (i = 0; i < ts_test->short_res.short_num; i++) {
			chn1 = ts_test->short_res.short_msg[4 * i];
			chn2 = ts_test->short_res.short_msg[4 * i + 1];
			r = (ts_test->short_res.short_msg[4 * i + 2] << 8) +
			    ts_test->short_res.short_msg[4 * i + 3];
			if (chn1 == CHN_VDD)
				index += sprintf(&rbuf[index],
					"<ShortMess Chn1=\"VDD\" ");
			else if (chn1 == CHN_GND)
				index += sprintf(&rbuf[index],
					"<ShortMess Chn1=\"GND\" ");
			else if (chn1 & DRV_CHANNEL_FLAG)
				index += sprintf(&rbuf[index],
					"<ShortMess Chn1=\"Tx%d\" ",
					chn1 & 0x7f);
			else
				index += sprintf(&rbuf[index],
					"<ShortMess Chn1=\"Rx%d\" ",
					chn1 & 0x7f);
			if (chn2 == CHN_VDD)
				index += sprintf(&rbuf[index],
					"Chn2=\"VDD\" ShortResistor= \"%dKom\"/>\n",
					r);
			else if (chn2 == CHN_GND)
				index += sprintf(&rbuf[index],
					"Chn2=\"GND\" ShortResistor= \"%dKom\"/>\n",
					r);
			else if (chn2 & DRV_CHANNEL_FLAG)
				index += sprintf(&rbuf[index],
					"Chn2=\"Tx%d\" ShortResistor= \"%dKom\"/>\n",
					chn2 & 0x7f, r);
			else
				index += sprintf(&rbuf[index],
					"Chn2=\"Rx%d\" ShortResistor= \"%dKom\"/>\n",
					chn2 & 0x7f, r);
		}
		index += sprintf(&rbuf[index], "</Item>\n");
	}

	/* save open limits */
	if (ts_test->item[GTP_CAP_TEST]) {
		index += sprintf(
			&rbuf[index], "<Item name=\"Rawdata Test Sets\">\n");
		index += sprintf(&rbuf[index],
			"<TotalFrameCnt>%d</TotalFrameCnt>\n",
			ts_test->raw_data_cnt);
		/* rawdata max limit */
		index += sprintf(&rbuf[index], "<MaxRawLimit>\n");
		for (i = 0; i < tx * rx; i++) {
			index += sprintf(
				&rbuf[index], "%d,", ts_test->max_limits[i]);
			if ((i + 1) % tx == 0)
				index += sprintf(&rbuf[index], "\n");
		}
		index += sprintf(&rbuf[index], "</MaxRawLimit>\n");
		/* rawdata min limit */
		index += sprintf(&rbuf[index], "<MinRawLimit>\n");
		for (i = 0; i < tx * rx; i++) {
			index += sprintf(
				&rbuf[index], "%d,", ts_test->min_limits[i]);
			if ((i + 1) % tx == 0)
				index += sprintf(&rbuf[index], "\n");
		}
		index += sprintf(&rbuf[index], "</MinRawLimit>\n");
		/* Max Accord limit */
		index += sprintf(&rbuf[index], "<MaxAccordLimit>\n");
		for (i = 0; i < tx * rx; i++) {
			index += sprintf(&rbuf[index], "%d,",
				ts_test->deviation_limits[i]);
			if ((i + 1) % tx == 0)
				index += sprintf(&rbuf[index], "\n");
		}
		index += sprintf(&rbuf[index], "</MaxAccordLimit>\n");
		index += sprintf(&rbuf[index], "</Item>\n");
	}

	/* save noise limit */
	if (ts_test->item[GTP_NOISE_TEST]) {
		index += sprintf(
			&rbuf[index], "<Item name=\"Diffdata Test Sets\">\n");
		index += sprintf(&rbuf[index],
			"<TotalFrameCnt>%d</TotalFrameCnt>\n",
			ts_test->noise_data_cnt);
		index += sprintf(&rbuf[index],
			"<MaxJitterLimit>%d</MaxJitterLimit>\n",
			ts_test->noise_threshold);
		index += sprintf(&rbuf[index], "</Item>\n");
	}

	/* save self rawdata limit */
	if (ts_test->item[GTP_SELFCAP_TEST]) {
		index += sprintf(&rbuf[index],
			"<Item name=\"Self Rawdata Test Sets\">\n");
		index += sprintf(
			&rbuf[index], "<TotalFrameCnt>1</TotalFrameCnt>\n");
		index += sprintf(&rbuf[index], "<MaxRawLimit>\n");
		for (i = 0; i < tx + rx; i++) {
			index += sprintf(&rbuf[index], "%d,",
				ts_test->self_max_limits[i]);
			if ((i + 1) % tx == 0)
				index += sprintf(&rbuf[index], "\n");
		}
		if ((tx + rx) % tx != 0)
			index += sprintf(&rbuf[index], "\n");
		index += sprintf(&rbuf[index], "</MaxRawLimit>\n");
		index += sprintf(&rbuf[index], "<MinRawLimit>\n");
		for (i = 0; i < tx + rx; i++) {
			index += sprintf(&rbuf[index], "%d,",
				ts_test->self_min_limits[i]);
			if ((i + 1) % tx == 0)
				index += sprintf(&rbuf[index], "\n");
		}
		if ((tx + rx) % tx != 0)
			index += sprintf(&rbuf[index], "\n");
		index += sprintf(&rbuf[index], "</MinRawLimit>\n");
		index += sprintf(&rbuf[index], "</Item>\n");
	}

	index += sprintf(&rbuf[index], "</TestItems>\n");
}

static void goodix_data_cal(s16 *data, size_t data_size, s16 *stat_result)
{
	int i = 0;
	s16 avg = 0;
	s16 min = 0;
	s16 max = 0;
	long long sum = 0;

	min = data[0];
	max = data[0];
	for (i = 0; i < data_size; i++) {
		sum += data[i];
		if (max < data[i])
			max = data[i];
		if (min > data[i])
			min = data[i];
	}
	avg = div_s64(sum, data_size);
	stat_result[0] = avg;
	stat_result[1] = max;
	stat_result[2] = min;
}

static void goodix_save_data(void)
{
	int tx = cd->ic_info.parm.drv_num;
	int rx = cd->ic_info.parm.sen_num;
	s16 stat_result[3];
	int i, j;

	index += sprintf(&rbuf[index], "<DataRecord>\n");

	/* save rawdata */
	if (ts_test->item[GTP_CAP_TEST]) {
		index += sprintf(&rbuf[index], "<RawDataRecord>\n");
		for (i = 0; i < ts_test->raw_data_cnt; i++) {
			goodix_data_cal(
				ts_test->rawdata[i].data, tx * rx, stat_result);
			index += sprintf(&rbuf[index],
				"<DataContent No.=\"%d\" DataCount=\"%d\" Maximum=\"%d\" Minimum=\"%d\" Average=\"%d\">\n",
				i, tx * rx, stat_result[1], stat_result[2],
				stat_result[0]);
			for (j = 0; j < tx * rx; j++) {
				index += sprintf(&rbuf[index], "%d,",
					ts_test->rawdata[i].data[j]);
				if ((j + 1) % tx == 0)
					index += sprintf(&rbuf[index], "\n");
			}
			index += sprintf(&rbuf[index], "</DataContent>\n");
			goodix_data_cal(ts_test->deltadata[i].data, tx * rx,
				stat_result);
			index += sprintf(&rbuf[index],
				"<RawAccord No.=\"%d\" DataCount=\"%d\" Maximum=\"%d\" Minimum=\"%d\" Average=\"%d\">\n",
				i, tx * rx, stat_result[1], stat_result[2],
				stat_result[0]);
			for (j = 0; j < tx * rx; j++) {
				index += sprintf(&rbuf[index], "%d,",
					ts_test->deltadata[i].data[j]);
				if ((j + 1) % tx == 0)
					index += sprintf(&rbuf[index], "\n");
			}
			index += sprintf(&rbuf[index], "</RawAccord>\n");
		}
		index += sprintf(&rbuf[index], "</RawDataRecord>\n");
	}

	/* save noisedata */
	if (ts_test->item[GTP_NOISE_TEST]) {
		index += sprintf(&rbuf[index], "<DiffDataRecord>\n");
		for (i = 0; i < ts_test->noise_data_cnt; i++) {
			goodix_data_cal(ts_test->noisedata[i].data, tx * rx,
				stat_result);
			index += sprintf(&rbuf[index],
				"<DataContent No.=\"%d\" DataCount=\"%d\" Maximum=\"%d\" Minimum=\"%d\" Average=\"%d\">\n",
				i, tx * rx, stat_result[1], stat_result[2],
				stat_result[0]);
			for (j = 0; j < tx * rx; j++) {
				index += sprintf(&rbuf[index], "%d,",
					ts_test->noisedata[i].data[j]);
				if ((j + 1) % tx == 0)
					index += sprintf(&rbuf[index], "\n");
			}
			index += sprintf(&rbuf[index], "</DataContent>\n");
		}
		index += sprintf(&rbuf[index], "</DiffDataRecord>\n");
	}

	/* save self rawdata */
	if (ts_test->item[GTP_SELFCAP_TEST]) {
		index += sprintf(&rbuf[index], "<selfDataRecord>\n");
		goodix_data_cal(
			ts_test->selfrawdata.data, tx + rx, stat_result);
		index += sprintf(&rbuf[index],
			"<DataContent No.=\"0\" DataCount=\"%d\" Maximum=\"%d\" Minimum=\"%d\" Average=\"%d\">\n",
			tx + rx, stat_result[1], stat_result[2],
			stat_result[0]);
		for (i = 0; i < tx + rx; i++) {
			index += sprintf(&rbuf[index], "%d,",
				ts_test->selfrawdata.data[i]);
			if ((i + 1) % tx == 0)
				index += sprintf(&rbuf[index], "\n");
		}
		if ((tx + rx) % tx != 0)
			index += sprintf(&rbuf[index], "\n");
		index += sprintf(&rbuf[index], "</DataContent>\n");
		index += sprintf(&rbuf[index], "</selfDataRecord>\n");
	}

	index += sprintf(&rbuf[index], "</DataRecord>\n");
}

static void goodix_save_tail(void)
{
	index += sprintf(&rbuf[index], "</TESTLOG>\n");
}

static void goodix_save_test_result(void)
{
	goodix_save_header();
	goodix_save_limits();
	goodix_save_data();
	goodix_save_tail();
}

static void goto_next_line(char **ptr)
{
	do {
		*ptr = *ptr + 1;
	} while (**ptr != '\n' && **ptr != '\0');
	if (**ptr == '\0') {
		return;
	}
	*ptr = *ptr + 1;
}

static void copy_this_line(char *dest, char *src)
{
	char *copy_from;
	char *copy_to;

	copy_from = src;
	copy_to = dest;
	do {
		*copy_to = *copy_from;
		copy_from++;
		copy_to++;
	} while ((*copy_from != '\n') && (*copy_from != '\r') &&
		 (*copy_from != '\0'));
	*copy_to = '\0';
}

static int getrid_space(s8 *data, s32 len)
{
	u8 *buf = NULL;
	s32 i;
	u32 count = 0;

	buf = (char *)kzalloc(len + 5, GFP_KERNEL);
	if (buf == NULL) {
		ts_err("get space kzalloc error");
		return -ESRCH;
	}

	for (i = 0; i < len; i++) {
		if (data[i] == ' ' || data[i] == '\r' || data[i] == '\n') {
			continue;
		}
		buf[count++] = data[i];
	}

	buf[count++] = '\0';

	memcpy(data, buf, count);
	kfree(buf);

	return count;
}

static int parse_valid_data(
	char *buf_start, loff_t buf_size, char *ptr, s16 *data, s32 rows)
{
	int i = 0;
	int j = 0;
	char *token = NULL;
	char *tok_ptr = NULL;
	char *row_data = NULL;
	long temp_val;

	if (!ptr || !data)
		return -EINVAL;

	row_data = (char *)kzalloc(MAX_LINE_LEN, GFP_KERNEL);
	if (!row_data) {
		ts_err("alloc index %d failed.", MAX_LINE_LEN);
		return -ENOMEM;
	}

	for (i = 0; i < rows; i++) {
		memset(row_data, 0, MAX_LINE_LEN);
		copy_this_line(row_data, ptr);
		getrid_space(row_data, strlen(row_data));
		tok_ptr = row_data;
		while ((token = strsep(&tok_ptr, ","))) {
			if (strlen(token) == 0)
				continue;
			if (kstrtol(token, 0, &temp_val)) {
				kfree(row_data);
				return -EINVAL;
			}
			data[j++] = (s16)temp_val;
		}
		if (i == rows - 1)
			break;
		goto_next_line(&ptr); // next row
		if (!ptr || (0 == strlen(ptr)) ||
			(ptr >= (buf_start + buf_size))) {
			ts_info("invalid ptr, return");
			kfree(row_data);
			row_data = NULL;
			return -EPERM;
		}
	}
	kfree(row_data);
	return j;
}

static int parse_csvfile(
	char *buf, size_t size, char *target_name, s16 *data, s32 rows, s32 col)
{
	char *ptr = NULL;

	if (!data || !buf)
		return -EIO;

	ptr = buf;
	ptr = strstr(ptr, target_name);
	if (!ptr) {
		ts_info("load %s failed 1, maybe not this item", target_name);
		return -EINTR;
	}

	goto_next_line(&ptr);
	if (!ptr || (0 == strlen(ptr))) {
		ts_err("load %s failed 2!", target_name);
		return -EIO;
	}

	return parse_valid_data(buf, size, ptr, data, rows);
}

static int goodix_obtain_testlimits(void)
{
	const struct firmware *firmware = NULL;
	struct device *dev = &cd->pdev->dev;
	int tx = cd->ic_info.parm.drv_num;
	int rx = cd->ic_info.parm.sen_num;
	char limit_file[100] = { 0 };
	char *temp_buf = NULL;
	s16 data_buf[7];
	int ret;

	sprintf(limit_file, "goodix_test_limits_%d.csv",
		cd->fw_version.sensor_id);
	ts_info("limit_file_name:%s", limit_file);

	ret = request_firmware(&firmware, limit_file, dev);
	if (ret < 0) {
		ts_err("limits file [%s] not available", limit_file);
		return -EINVAL;
	}
	if (firmware->size <= 0) {
		ts_err("request_firmware, limits param length error,len:%zu",
			firmware->size);
		ret = -EINVAL;
		goto exit_free;
	}
	temp_buf = kzalloc(firmware->size + 1, GFP_KERNEL);
	if (!temp_buf) {
		ret = -ENOMEM;
		goto exit_free;
	}
	memcpy(temp_buf, firmware->data, firmware->size);

	if (ts_test->item[GTP_CAP_TEST]) {
		/* obtain mutual_raw min */
		ret = parse_csvfile(temp_buf, firmware->size,
			CSV_TP_SPECIAL_RAW_MIN, ts_test->min_limits, rx, tx);
		if (ret < 0) {
			ts_err("Failed get min_limits");
			goto exit_free;
		}
		/* obtain mutual_raw max */
		ret = parse_csvfile(temp_buf, firmware->size,
			CSV_TP_SPECIAL_RAW_MAX, ts_test->max_limits, rx, tx);
		if (ret < 0) {
			ts_err("Failed get max_limits");
			goto exit_free;
		}
		/* obtain delta limit */
		ret = parse_csvfile(temp_buf, firmware->size,
			CSV_TP_SPECIAL_RAW_DELTA, ts_test->deviation_limits, rx,
			tx);
		if (ret < 0) {
			ts_err("Failed get delta limit");
			goto exit_free;
		}
	}

	if (ts_test->item[GTP_SELFCAP_TEST]) {
		/* obtain self_raw min */
		ret = parse_csvfile(temp_buf, firmware->size,
			CSV_TP_SPECIAL_SELFRAW_MIN, ts_test->self_min_limits, 1,
			tx + rx);
		if (ret < 0) {
			ts_err("Failed get self_min_limits");
			goto exit_free;
		}
		/* obtain self_raw max */
		ret = parse_csvfile(temp_buf, firmware->size,
			CSV_TP_SPECIAL_SELFRAW_MAX, ts_test->self_max_limits, 1,
			tx + rx);
		if (ret < 0) {
			ts_err("Failed get self_min_limits");
			goto exit_free;
		}
	}

	if (ts_test->item[GTP_NOISE_TEST]) {
		/* obtain noise_threshold */
		ret = parse_csvfile(temp_buf, firmware->size,
			CSV_TP_NOISE_LIMIT, &ts_test->noise_threshold, 1, 1);
		if (ret < 0) {
			ts_err("Failed get noise limits");
			goto exit_free;
		}
	}

	if (ts_test->item[GTP_SHORT_TEST]) {
		/* obtain short_params */
		ret = parse_csvfile(temp_buf, firmware->size,
			CSV_TP_SHORT_THRESHOLD, data_buf, 1, 7);
		if (ret < 0) {
			ts_err("Failed get short circuit limits");
			goto exit_free;
		}
		ts_test->short_threshold = data_buf[0];
		ts_test->r_drv_drv_threshold = data_buf[1];
		ts_test->r_drv_sen_threshold = data_buf[2];
		ts_test->r_sen_sen_threshold = data_buf[3];
		ts_test->r_drv_gnd_threshold = data_buf[4];
		ts_test->r_sen_gnd_threshold = data_buf[5];
		ts_test->avdd_value = data_buf[6];
	}

exit_free:
	kfree(temp_buf);
	if (firmware)
		release_firmware(firmware);
	return ret;
}

static int goodix_delta_test(void)
{
	int i, j;
	int max_val;
	int raw;
	int temp;
	int tx = cd->ic_info.parm.drv_num;
	int rx = cd->ic_info.parm.sen_num;
	u32 data_size = tx * rx;
	int ret = 0;

	for (i = 0; i < ts_test->raw_data_cnt; i++) {
		for (j = 0; j < data_size; j++) {
			raw = ts_test->rawdata[i].data[j];
			max_val = 0;
			/* calcu delta with above node */
			if (j - tx >= 0) {
				temp = ts_test->rawdata[i].data[j - tx];
				temp = ABS(temp - raw);
				max_val = MAX(max_val, temp);
			}
			/* calcu delta with bellow node */
			if (j + tx < data_size) {
				temp = ts_test->rawdata[i].data[j + tx];
				temp = ABS(temp - raw);
				max_val = MAX(max_val, temp);
			}
			/* calcu delta with left node */
			if (j % tx) {
				temp = ts_test->rawdata[i].data[j - 1];
				temp = ABS(temp - raw);
				max_val = MAX(max_val, temp);
			}
			/* calcu delta with right node */
			if ((j + 1) % tx) {
				temp = ts_test->rawdata[i].data[j + 1];
				temp = ABS(temp - raw);
				max_val = MAX(max_val, temp);
			}
			temp = max_val * 1000 / raw;
			ts_test->deltadata[i].data[j] = temp;
			if (temp > ts_test->deviation_limits[j]) {
				ts_err("delta_data[%d] > limits[%d]", temp,
					ts_test->deviation_limits[j]);
				ret = -EINVAL;
			}
		}
	}

	return ret;
}

static int goodix_open_test(void)
{
	unsigned char tmp_buf[GOODIX_MAX_FRAMEDATA_LEN];
	struct goodix_ts_cmd temp_cmd;
	int tx = cd->ic_info.parm.drv_num;
	int rx = cd->ic_info.parm.sen_num;
	u32 sync_addr = cd->ic_info.misc.frame_data_addr;
	u32 raw_addr;
	int ret;
	int err_cnt = 0;
	int i, j;
	s16 tmp_val;
	u16 tmp_freq;
	u8 val;
	int retry;

	raw_addr = cd->ic_info.misc.frame_data_addr +
		   cd->ic_info.misc.frame_data_head_len +
		   cd->ic_info.misc.fw_attr_len + cd->ic_info.misc.fw_log_len +
		   8;

	/* open test prepare */
	temp_cmd.cmd = 0x90;
	temp_cmd.data[0] = 0x84;
	temp_cmd.len = 5;
	ret = cd->hw_ops->send_cmd(cd, &temp_cmd);
	if (ret < 0) {
		ts_err("send rawdata cmd failed");
		return ret;
	}

	/* switch freq */
	if (ts_test->freq > 0) {
		ts_info("set freq %d", ts_test->freq);
		tmp_freq = ts_test->freq / 61;
		temp_cmd.len = 6;
		temp_cmd.cmd = 0xB1;
		temp_cmd.data[0] = tmp_freq & 0xFF;
		temp_cmd.data[1] = (tmp_freq >> 8) & 0xFF;
		ret = cd->hw_ops->send_cmd(cd, &temp_cmd);
		if (ret < 0) {
			ts_err("set freq %d failed", ts_test->freq);
			return ret;
		}
	}

	/* discard the first few frames */
	for (i = 0; i < 3; i++) {
		val = 0;
		cd->hw_ops->write(cd, sync_addr, &val, 1);
		usleep_range(20000, 21000);
	}

	/* read rawdata */
	for (i = 0; i < ts_test->raw_data_cnt; i++) {
		val = 0;
		cd->hw_ops->write(cd, sync_addr, &val, 1);
		retry = 20;
		while (retry--) {
			usleep_range(5000, 5100);
			cd->hw_ops->read(cd, sync_addr, &val, 1);
			if (val & 0x80)
				break;
		}
		if (retry < 0) {
			ts_err("rawdata is not ready val:0x%02x i:%d, exit",
				val, i);
			ret = -EINVAL;
			goto exit;
		}

		cd->hw_ops->read(cd, raw_addr, tmp_buf, tx * rx * 2);
		goodix_rotate_abcd2cbad(tx, rx, (s16 *)tmp_buf);
		memcpy((u8 *)ts_test->rawdata[i].data, tmp_buf, tx * rx * 2);
	}

	/* analysis results */
	ts_test->result[GTP_CAP_TEST] = TEST_OK;
	for (i = 0; i < ts_test->raw_data_cnt; i++) {
		for (j = 0; j < tx * rx; j++) {
			tmp_val = ts_test->rawdata[i].data[j];
			if (tmp_val > ts_test->max_limits[j] ||
				tmp_val < ts_test->min_limits[j]) {
				ts_err("rawdata[%d] out of range[%d %d]",
					tmp_val, ts_test->min_limits[j],
					ts_test->max_limits[j]);
				err_cnt++;
			}
		}
	}

	if (err_cnt > 0) {
		ret = -EINVAL;
		ts_test->result[GTP_CAP_TEST] = TEST_NG;
	}

	if (goodix_delta_test() == 0)
		ts_test->result[GTP_DELTA_TEST] = TEST_OK;

exit:
	return ret;
}

static int goodix_self_open_test(void)
{
	unsigned char tmp_buf[GOODIX_MAX_FRAMEDATA_LEN];
	struct goodix_ts_cmd temp_cmd;
	int tx = cd->ic_info.parm.drv_num;
	int rx = cd->ic_info.parm.sen_num;
	u32 sync_addr = cd->ic_info.misc.frame_data_addr;
	u32 raw_addr;
	int ret;
	int j;
	s16 tmp_val;
	u8 val;
	int retry;

	raw_addr = cd->ic_info.misc.frame_data_addr +
		   cd->ic_info.misc.frame_data_head_len +
		   cd->ic_info.misc.fw_attr_len + cd->ic_info.misc.fw_log_len +
		   cd->ic_info.misc.mutual_struct_len + 10;

	/* test prepare */
	temp_cmd.cmd = 0x90;
	temp_cmd.data[0] = 0x84;
	temp_cmd.len = 5;
	ret = cd->hw_ops->send_cmd(cd, &temp_cmd);
	if (ret < 0) {
		ts_err("send rawdata cmd failed");
		return ret;
	}

	/* discard the first few frames */
	for (j = 0; j < 3; j++) {
		val = 0;
		cd->hw_ops->write(cd, sync_addr, &val, 1);
		usleep_range(20000, 21000);
	}

	/* read self rawdata */
	val = 0;
	cd->hw_ops->write(cd, sync_addr, &val, 1);
	retry = 20;
	while (retry--) {
		usleep_range(5000, 5100);
		cd->hw_ops->read(cd, sync_addr, &val, 1);
		if (val & 0x80)
			break;
	}
	if (retry < 0) {
		ts_err("self rawdata is not ready val:0x%02x, exit", val);
		ret = -EINVAL;
		goto exit;
	}

	cd->hw_ops->read(cd, raw_addr, tmp_buf, (tx + rx) * 2);
	memcpy((u8 *)ts_test->selfrawdata.data, tmp_buf, (tx + rx) * 2);

	/* analysis results */
	ts_test->result[GTP_SELFCAP_TEST] = TEST_OK;
	for (j = 0; j < tx + rx; j++) {
		tmp_val = ts_test->selfrawdata.data[j];
		if (tmp_val > ts_test->self_max_limits[j] ||
			tmp_val < ts_test->self_min_limits[j]) {
			ts_err("self_rawdata[%d] out of range[%d %d]", tmp_val,
				ts_test->self_min_limits[j],
				ts_test->self_max_limits[j]);
			ts_test->result[GTP_SELFCAP_TEST] = TEST_NG;
			ret = -EINVAL;
		}
	}

exit:
	return ret;
}

static int goodix_noise_test(void)
{
	unsigned char tmp_buf[GOODIX_MAX_FRAMEDATA_LEN];
	struct goodix_ts_cmd temp_cmd;
	int tx = cd->ic_info.parm.drv_num;
	int rx = cd->ic_info.parm.sen_num;
	u32 sync_addr = cd->ic_info.misc.frame_data_addr;
	u32 raw_addr;
	int ret;
	int i, j;
	s16 tmp_val;
	u8 val;
	int retry;

	raw_addr = cd->ic_info.misc.frame_data_addr +
		   cd->ic_info.misc.frame_data_head_len +
		   cd->ic_info.misc.fw_attr_len + cd->ic_info.misc.fw_log_len +
		   8;

	/* open test prepare */
	temp_cmd.cmd = 0x90;
	temp_cmd.data[0] = 0x82;
	temp_cmd.len = 5;
	ret = cd->hw_ops->send_cmd(cd, &temp_cmd);
	if (ret < 0) {
		ts_err("send rawdata cmd failed");
		return ret;
	}

	/* discard the first few frames */
	for (i = 0; i < 3; i++) {
		val = 0;
		cd->hw_ops->write(cd, sync_addr, &val, 1);
		usleep_range(20000, 21000);
	}

	/* read noisedata */
	for (i = 0; i < ts_test->noise_data_cnt; i++) {
		val = 0;
		cd->hw_ops->write(cd, sync_addr, &val, 1);
		retry = 20;
		while (retry--) {
			usleep_range(5000, 5100);
			cd->hw_ops->read(cd, sync_addr, &val, 1);
			if (val & 0x80)
				break;
		}
		if (retry < 0) {
			ts_err("noisedata is not ready val:0x%02x i:%d, exit",
				val, i);
			ret = -EINVAL;
			goto exit;
		}

		cd->hw_ops->read(cd, raw_addr, tmp_buf, tx * rx * 2);
		goodix_rotate_abcd2cbad(tx, rx, (s16 *)tmp_buf);
		memcpy((u8 *)ts_test->noisedata[i].data, tmp_buf, tx * rx * 2);
	}

	/* analysis results */
	ts_test->result[GTP_NOISE_TEST] = TEST_OK;
	for (i = 0; i < ts_test->noise_data_cnt; i++) {
		for (j = 0; j < tx * rx; j++) {
			tmp_val = ts_test->noisedata[i].data[j];
			tmp_val = ABS(tmp_val);
			if (tmp_val > ts_test->noise_threshold) {
				ts_err("noise data[%d] > noise threshold[%d]",
					tmp_val, ts_test->noise_threshold);
				ts_test->result[GTP_NOISE_TEST] = TEST_NG;
				ret = -EINVAL;
			}
		}
	}

exit:
	return ret;
}

static int goodix_auto_test(void)
{
	struct goodix_ts_cmd temp_cmd;
	int ret;

	ret = goodix_obtain_testlimits();
	if (ret < 0) {
		ts_err("obtain open test limits failed");
		return ret;
	}

	cd->hw_ops->irq_enable(cd, false);
	goodix_ts_blocking_notify(NOTIFY_ESD_OFF, NULL);

	temp_cmd.len = 5;
	temp_cmd.cmd = 0x64;
	temp_cmd.data[0] = 1;

	if (ts_test->item[GTP_CAP_TEST]) {
		ret = cd->hw_ops->send_cmd(cd, &temp_cmd);
		if (ret < 0)
			ts_err("enter test mode failed");
		goodix_open_test();
		cd->hw_ops->reset(cd, 100);
	}

	if (ts_test->item[GTP_NOISE_TEST]) {
		ret = cd->hw_ops->send_cmd(cd, &temp_cmd);
		if (ret < 0)
			ts_err("enter test mode failed");
		goodix_noise_test();
		cd->hw_ops->reset(cd, 100);
	}

	if (ts_test->item[GTP_SELFCAP_TEST]) {
		goodix_self_open_test();
		cd->hw_ops->reset(cd, 100);
	}

	if (ts_test->item[GTP_SHORT_TEST]) {
		goodix_shortcircut_test();
		cd->hw_ops->reset(cd, 100);
	}

	cd->hw_ops->irq_enable(cd, true);
	goodix_ts_blocking_notify(NOTIFY_ESD_ON, NULL);
	goodix_save_test_result();
	return 0;
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

	if (val == 0) {
		ts_info("set scan mode to default");
		index = sprintf(rbuf, "set scan mode to default\n");
	} else if (val == 1) {
		ts_info("set scan mode to idle");
		index = sprintf(rbuf, "set scan mode to idle\n");
	} else {
		ts_info("set scan mode to active");
		index = sprintf(rbuf, "set scan mode to active\n");
	}

	temp_cmd.len = 5;
	temp_cmd.cmd = 0x9F;
	temp_cmd.data[0] = val;
	cd->hw_ops->send_cmd(cd, &temp_cmd);
}

static void goodix_set_continue_mode(u8 val)
{
	struct goodix_ts_cmd temp_cmd;

	if (val == 0) {
		ts_info("enable continue report");
		index = sprintf(rbuf, "enable continue report\n");
	} else {
		ts_info("disable continue report");
		index = sprintf(rbuf, "disable continue report\n");
	}

	temp_cmd.len = 5;
	temp_cmd.cmd = 0xC3;
	temp_cmd.data[0] = val;
	cd->hw_ops->send_cmd(cd, &temp_cmd);
}

static ssize_t driver_test_write(
	struct file *file, const char __user *buf, size_t count, loff_t *pos)
{
	struct goodix_fw_version fw_ver;
	struct goodix_ic_info ic_info;
	char *p = wbuf;
	char *token;
	int ret;
	int cmd_val;
	int cmd_val2;
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
	release_test_resource();

	ts_info("input cmd[%s]", p);

	if (!strncmp(p, CMD_FW_UPDATE, strlen(CMD_FW_UPDATE))) {
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

	if (!strncmp(p, CMD_GET_VERSION, strlen(CMD_GET_VERSION))) {
		rbuf = kzalloc(SHORT_SIZE, GFP_KERNEL);
		cd->hw_ops->read_version(cd, &fw_ver);
		cd->hw_ops->get_ic_info(cd, &ic_info);
		index = sprintf(rbuf, "%s: %02x%02x%02x%02x %x\n",
			CMD_GET_VERSION, fw_ver.patch_vid[0],
			fw_ver.patch_vid[1], fw_ver.patch_vid[2],
			fw_ver.patch_vid[3], ic_info.version.config_id);
		goto exit;
	}

	if (!strncmp(p, CMD_GET_RAWDATA, strlen(CMD_GET_RAWDATA))) {
		rbuf = kzalloc(LARGE_SIZE, GFP_KERNEL);
		ret = get_cap_data(CMD_GET_RAWDATA);
		if (ret < 0) {
			index = sprintf(rbuf, "%s: NG\n", CMD_GET_RAWDATA);
		}
		goto exit;
	}

	if (!strncmp(p, CMD_GET_BASEDATA, strlen(CMD_GET_BASEDATA))) {
		rbuf = kzalloc(LARGE_SIZE, GFP_KERNEL);
		ret = get_cap_data(CMD_GET_BASEDATA);
		if (ret < 0) {
			index = sprintf(rbuf, "%s: NG\n", CMD_GET_BASEDATA);
		}
		goto exit;
	}

	if (!strncmp(p, CMD_GET_DIFFDATA, strlen(CMD_GET_DIFFDATA))) {
		rbuf = kzalloc(LARGE_SIZE, GFP_KERNEL);
		ret = get_cap_data(CMD_GET_DIFFDATA);
		if (ret < 0) {
			index = sprintf(rbuf, "%s: NG\n", CMD_GET_DIFFDATA);
		}
		goto exit;
	}

	if (!strncmp(p, CMD_GET_SELF_RAWDATA, strlen(CMD_GET_SELF_RAWDATA))) {
		rbuf = kzalloc(LARGE_SIZE, GFP_KERNEL);
		ret = get_cap_data(CMD_GET_SELF_RAWDATA);
		if (ret < 0) {
			index = sprintf(rbuf, "%s: NG\n", CMD_GET_SELF_RAWDATA);
		}
		goto exit;
	}

	if (!strncmp(p, CMD_GET_SELF_DIFFDATA, strlen(CMD_GET_SELF_DIFFDATA))) {
		rbuf = kzalloc(LARGE_SIZE, GFP_KERNEL);
		ret = get_cap_data(CMD_GET_SELF_DIFFDATA);
		if (ret < 0) {
			index = sprintf(
				rbuf, "%s: NG\n", CMD_GET_SELF_DIFFDATA);
		}
		goto exit;
	}

	if (!strncmp(p, CMD_GET_SELF_BASEDATA, strlen(CMD_GET_SELF_BASEDATA))) {
		rbuf = kzalloc(LARGE_SIZE, GFP_KERNEL);
		ret = get_cap_data(CMD_GET_SELF_BASEDATA);
		if (ret < 0) {
			index = sprintf(
				rbuf, "%s: NG\n", CMD_GET_SELF_BASEDATA);
		}
		goto exit;
	}

	if (!strncmp(p, CMD_SET_DOUBLE_TAP, strlen(CMD_SET_DOUBLE_TAP))) {
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

	if (!strncmp(p, CMD_SET_SINGLE_TAP, strlen(CMD_SET_SINGLE_TAP))) {
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

	if (!strncmp(p, CMD_SET_IRQ_ENABLE, strlen(CMD_SET_IRQ_ENABLE))) {
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

	if (!strncmp(p, CMD_SET_ESD_ENABLE, strlen(CMD_SET_ESD_ENABLE))) {
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

	if (!strncmp(p, CMD_SET_DEBUG_LOG, strlen(CMD_SET_DEBUG_LOG))) {
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

	if (!strncmp(p, CMD_AUTO_TEST, strlen(CMD_AUTO_TEST))) {
		rbuf = kzalloc(HUGE_SIZE, GFP_KERNEL);
		ret = malloc_test_resource();
		if (ret < 0) {
			ts_err("malloc test resource failed");
			goto exit;
		}
		ts_test->item[GTP_CAP_TEST] = true;
		ts_test->item[GTP_NOISE_TEST] = true;
		ts_test->item[GTP_SELFCAP_TEST] = true;
		ts_test->item[GTP_SHORT_TEST] = true;
		ts_test->raw_data_cnt = 16;
		ts_test->noise_data_cnt = 1;
		goodix_auto_test();
		goto exit;
	}

	if (!strncmp(p, CMD_GET_CHANNEL_NUM, strlen(CMD_GET_CHANNEL_NUM))) {
		rbuf = kzalloc(SHORT_SIZE, GFP_KERNEL);
		index = sprintf(rbuf, "TX:%d RX:%d\n", cd->ic_info.parm.drv_num,
			cd->ic_info.parm.sen_num);
		goto exit;
	}

	if (!strncmp(p, CMD_GET_TX_FREQ, strlen(CMD_GET_TX_FREQ))) {
		rbuf = kzalloc(SHORT_SIZE, GFP_KERNEL);
		ret = get_cap_data(CMD_GET_TX_FREQ);
		if (ret < 0) {
			index = sprintf(rbuf, "%s: NG\n", CMD_GET_TX_FREQ);
		}
		goto exit;
	}

	if (!strncmp(p, CMD_RESET, strlen(CMD_RESET))) {
		cd->hw_ops->irq_enable(cd, false);
		cd->hw_ops->reset(cd, 100);
		cd->hw_ops->irq_enable(cd, true);
		goto exit;
	}

	if (!strncmp(p, CMD_SET_SENSE_ENABLE, strlen(CMD_SET_SENSE_ENABLE))) {
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

	if (!strncmp(p, CMD_NOISE_TEST, strlen(CMD_NOISE_TEST))) {
		rbuf = kzalloc(HUGE_SIZE, GFP_KERNEL);
		token = strsep(&p, ",");
		if (!token || !p) {
			index = sprintf(rbuf, "%s: invalid cmd param\n",
				CMD_NOISE_TEST);
			goto exit;
		}
		if (kstrtos32(p, 10, &cmd_val)) {
			index = sprintf(rbuf, "%s: invalid cmd param\n",
				CMD_NOISE_TEST);
			goto exit;
		}
		if (cmd_val > MAX_FRAME_CNT) {
			index = sprintf(rbuf,
				"%s: frame cnt:%d > MAX_CNT[%d]\n",
				CMD_NOISE_TEST, (int)cmd_val, MAX_FRAME_CNT);
			goto exit;
		}
		ret = malloc_test_resource();
		if (ret < 0) {
			ts_err("malloc test resource failed");
			goto exit;
		}
		ts_test->item[GTP_NOISE_TEST] = true;
		ts_test->noise_data_cnt = cmd_val;
		goodix_auto_test();
		goto exit;
	}

	if (!strncmp(p, CMD_GET_PACKAGE_ID, strlen(CMD_GET_PACKAGE_ID))) {
		rbuf = kzalloc(SHORT_SIZE, GFP_KERNEL);
		ret = goodix_flash_read(0x1F301, &id, 1);
		if (ret < 0)
			index = sprintf(rbuf, "%s: NG\n", CMD_GET_PACKAGE_ID);
		else
			index = sprintf(
				rbuf, "%s: 0x%x\n", CMD_GET_PACKAGE_ID, id);
		goto exit;
	}

	if (!strncmp(p, CMD_SET_SCAN_MODE, strlen(CMD_SET_SCAN_MODE))) {
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
		if (cmd_val > 2 || cmd_val < 0) {
			index = sprintf(rbuf, "%s: invalid cmd param\n",
				CMD_SET_SCAN_MODE);
			goto exit;
		}
		goodix_set_scan_mode(cmd_val);
		goto exit;
	}

	if (!strncmp(p, CMD_SET_CONTINUE_MODE, strlen(CMD_SET_CONTINUE_MODE))) {
		rbuf = kzalloc(SHORT_SIZE, GFP_KERNEL);
		token = strsep(&p, ",");
		if (!token || !p) {
			index = sprintf(rbuf, "%s: invalid cmd param\n",
				CMD_SET_CONTINUE_MODE);
			goto exit;
		}
		if (kstrtos32(p, 10, &cmd_val)) {
			index = sprintf(rbuf, "%s: invalid cmd param\n",
				CMD_SET_CONTINUE_MODE);
			goto exit;
		}
		if (cmd_val > 1 || cmd_val < 0) {
			index = sprintf(rbuf, "%s: invalid cmd param\n",
				CMD_SET_CONTINUE_MODE);
			goto exit;
		}
		goodix_set_continue_mode(cmd_val);
		goto exit;
	}

	/* open test */
	if (!strncmp(p, CMD_OPEN_TEST, strlen(CMD_OPEN_TEST))) {
		rbuf = kzalloc(HUGE_SIZE, GFP_KERNEL);
		token = strsep(&p, ",");
		if (!token || !p) {
			index = sprintf(
				rbuf, "%s: invalid cmd param\n", CMD_OPEN_TEST);
			goto exit;
		}
		token = strsep(&p, ",");
		if (!token || !p) {
			index = sprintf(
				rbuf, "%s: invalid cmd param\n", CMD_OPEN_TEST);
			goto exit;
		}
		if (kstrtos32(token, 10, &cmd_val)) {
			index = sprintf(
				rbuf, "%s: invalid cmd param\n", CMD_OPEN_TEST);
			goto exit;
		}
		if (kstrtos32(p, 10, &cmd_val2)) {
			index = sprintf(
				rbuf, "%s: invalid cmd param\n", CMD_OPEN_TEST);
			goto exit;
		}
		ret = malloc_test_resource();
		if (ret < 0) {
			ts_err("malloc test resource failed");
			goto exit;
		}
		ts_test->item[GTP_CAP_TEST] = true;
		ts_test->raw_data_cnt = cmd_val;
		ts_test->freq = cmd_val2;
		goodix_auto_test();
		goto exit;
	}

	if (!strncmp(p, CMD_SELF_OPEN_TEST, strlen(CMD_SELF_OPEN_TEST))) {
		rbuf = kzalloc(HUGE_SIZE, GFP_KERNEL);
		ret = malloc_test_resource();
		if (ret < 0) {
			ts_err("malloc test resource failed");
			goto exit;
		}
		ts_test->item[GTP_SELFCAP_TEST] = true;
		goodix_auto_test();
		goto exit;
	}

	if (!strncmp(p, CMD_SHORT_TEST, strlen(CMD_SHORT_TEST))) {
		rbuf = kzalloc(HUGE_SIZE, GFP_KERNEL);
		ret = malloc_test_resource();
		if (ret < 0) {
			ts_err("malloc test resource failed");
			goto exit;
		}
		ts_test->item[GTP_SHORT_TEST] = true;
		goodix_auto_test();
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