/* SPDX-License-Identifier: GPL-2.0-only */
// BQ25756 Charger Driver
// Copyright (C) 2020 Texas Instruments Incorporated - http://www.ti.com/
// SPDX-License-Identifier: GPL-2.0
// BQ25756 driver
// Copyright (C) 2020 Texas Instruments Incorporated - http://www.ti.com/

#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "stdbool.h"
#include "bq25756_i2c.h"
typedef unsigned char  u8;
#define BQ25756_NUM_WD_VAL	8
#define	EINVAL		22
#define I2C_NAME_SIZE	20

#ifdef __ASSEMBLY__
#else
#define __AC(X,Y)	(X##Y)
#define _AC(X,Y)	__AC(X,Y)
#endif
#define _UL(x)		(_AC(x, UL))
#ifndef __VDSO_CONST_H
#define __VDSO_CONST_H
#define UL(x)		(_UL(x))
#endif
#ifndef __VDSO_BITS_H
#define __VDSO_BITS_H
#define BIT(nr)			(UL(1) << (nr))
#endif

enum {
	POWER_SUPPLY_STATUS_UNKNOWN = 0,
	POWER_SUPPLY_STATUS_CHARGING,
	POWER_SUPPLY_STATUS_DISCHARGING,
	POWER_SUPPLY_STATUS_NOT_CHARGING,
	POWER_SUPPLY_STATUS_FULL,
};
enum {
	POWER_SUPPLY_CHARGE_TYPE_UNKNOWN = 0,
	POWER_SUPPLY_CHARGE_TYPE_NONE,
	POWER_SUPPLY_CHARGE_TYPE_TRICKLE,
	POWER_SUPPLY_CHARGE_TYPE_FAST,
	POWER_SUPPLY_CHARGE_TYPE_STANDARD,
	POWER_SUPPLY_CHARGE_TYPE_ADAPTIVE,
	POWER_SUPPLY_CHARGE_TYPE_CUSTOM,
	POWER_SUPPLY_CHARGE_TYPE_LONGLIFE,
};
enum power_supply_usb_type {
	POWER_SUPPLY_USB_TYPE_UNKNOWN = 0,
	POWER_SUPPLY_USB_TYPE_SDP,
	POWER_SUPPLY_USB_TYPE_DCP,
	POWER_SUPPLY_USB_TYPE_CDP,
	POWER_SUPPLY_USB_TYPE_ACA,
	POWER_SUPPLY_USB_TYPE_C,
	POWER_SUPPLY_USB_TYPE_PD,
	POWER_SUPPLY_USB_TYPE_PD_DRP,
	POWER_SUPPLY_USB_TYPE_PD_PPS,
	POWER_SUPPLY_USB_TYPE_APPLE_BRICK_ID,
};
enum {
	POWER_SUPPLY_HEALTH_UNKNOWN = 0,
	POWER_SUPPLY_HEALTH_GOOD,
	POWER_SUPPLY_HEALTH_OVERHEAT,
	POWER_SUPPLY_HEALTH_DEAD,
	POWER_SUPPLY_HEALTH_OVERVOLTAGE,
	POWER_SUPPLY_HEALTH_UNSPEC_FAILURE,
	POWER_SUPPLY_HEALTH_COLD,
	POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE,
	POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE,
	POWER_SUPPLY_HEALTH_OVERCURRENT,
	POWER_SUPPLY_HEALTH_CALIBRATION_REQUIRED,
	POWER_SUPPLY_HEALTH_WARM,
	POWER_SUPPLY_HEALTH_COOL,
	POWER_SUPPLY_HEALTH_HOT,
};
enum power_supply_property {
	POWER_SUPPLY_PROP_STATUS = 0,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_AUTHENTIC,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_VOLTAGE_MIN,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_AVG,
	POWER_SUPPLY_PROP_VOLTAGE_OCV,
	POWER_SUPPLY_PROP_VOLTAGE_BOOT,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_AVG,
	POWER_SUPPLY_PROP_CURRENT_BOOT,
	POWER_SUPPLY_PROP_POWER_NOW,
	POWER_SUPPLY_PROP_POWER_AVG,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CHARGE_EMPTY_DESIGN,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_EMPTY,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_CHARGE_AVG,
	POWER_SUPPLY_PROP_CHARGE_COUNTER,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT,
	POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT_MAX,
	POWER_SUPPLY_PROP_CHARGE_CONTROL_START_THRESHOLD,
	POWER_SUPPLY_PROP_CHARGE_CONTROL_END_THRESHOLD,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
	POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT,
	POWER_SUPPLY_PROP_INPUT_POWER_LIMIT,
	POWER_SUPPLY_PROP_ENERGY_FULL_DESIGN,
	POWER_SUPPLY_PROP_ENERGY_EMPTY_DESIGN,
	POWER_SUPPLY_PROP_ENERGY_FULL,
	POWER_SUPPLY_PROP_ENERGY_EMPTY,
	POWER_SUPPLY_PROP_ENERGY_NOW,
	POWER_SUPPLY_PROP_ENERGY_AVG,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CAPACITY_ALERT_MIN,
	POWER_SUPPLY_PROP_CAPACITY_ALERT_MAX,
	POWER_SUPPLY_PROP_CAPACITY_ERROR_MARGIN,
	POWER_SUPPLY_PROP_CAPACITY_LEVEL,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TEMP_MAX,
	POWER_SUPPLY_PROP_TEMP_MIN,
	POWER_SUPPLY_PROP_TEMP_ALERT_MIN,
	POWER_SUPPLY_PROP_TEMP_ALERT_MAX,
	POWER_SUPPLY_PROP_TEMP_AMBIENT,
	POWER_SUPPLY_PROP_TEMP_AMBIENT_ALERT_MIN,
	POWER_SUPPLY_PROP_TEMP_AMBIENT_ALERT_MAX,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG,
	POWER_SUPPLY_PROP_TIME_TO_FULL_NOW,
	POWER_SUPPLY_PROP_TIME_TO_FULL_AVG,
	POWER_SUPPLY_PROP_TYPE,
	POWER_SUPPLY_PROP_USB_TYPE,
	POWER_SUPPLY_PROP_SCOPE,
	POWER_SUPPLY_PROP_PRECHARGE_CURRENT,
	POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT,
	POWER_SUPPLY_PROP_CALIBRATE,
	POWER_SUPPLY_PROP_MANUFACTURE_YEAR,
	POWER_SUPPLY_PROP_MANUFACTURE_MONTH,
	POWER_SUPPLY_PROP_MANUFACTURE_DAY,
	POWER_SUPPLY_PROP_MODEL_NAME,
	POWER_SUPPLY_PROP_MANUFACTURER,
	POWER_SUPPLY_PROP_SERIAL_NUMBER,
};
union power_supply_propval {
	int intval;
	const char *strval;
};

struct power_supply {
	const struct power_supply_desc *desc;
	char **supplied_to;
	size_t num_supplicants;
	char **supplied_from;
	size_t num_supplies;
	struct device_node *of_node;
	void *drv_data;
	bool changed;
	bool initialized;
	bool removing;
#ifdef CONFIG_THERMAL
	struct thermal_zone_device *tzd;
	struct thermal_cooling_device *tcd;
#endif
#ifdef CONFIG_LEDS_TRIGGERS
	struct led_trigger *charging_full_trig;
	char *charging_full_trig_name;
	struct led_trigger *charging_trig;
	char *charging_trig_name;
	struct led_trigger *full_trig;
	char *full_trig_name;
	struct led_trigger *online_trig;
	char *online_trig_name;
	struct led_trigger *charging_blink_full_solid_trig;
	char *charging_blink_full_solid_trig_name;
#endif
};

struct bq25756_init_data {
	u32 ichg;
	int ilim;
	u32 vreg;
	u32 iterm;
	u32 iprechg;
	int vlim;
	u32 ichg_max;
	u32 vreg_max;
};
struct bq25756_state {
	bool online;
	u8 chrg_status;
	u8 chrg_type;
	u8 health;
	u8 chrg_fault;
	u8 vsys_status;
	u8 fault_0;
	u8 fault_1;
	u32 vbat_adc;
	u32 vac_adc;
	u32 ibat_adc;
};
struct bq25756_device {
	char model_name[I2C_NAME_SIZE];
	struct i2c_client *client;
	struct device *dev;
	struct power_supply *charger;
	struct usb_phy *usb2_phy;
	struct usb_phy *usb3_phy;
	unsigned long usb_event;
	struct regmap *regmap;
	int device_id;
	struct bq25756_init_data init_data;
	struct bq25756_state state;
	int watchdog_timer;
};


struct power_supply_battery_info {
	unsigned int technology;
	int energy_full_design_uwh;
	int charge_full_design_uah;
	int voltage_min_design_uv;
	int voltage_max_design_uv;
	int tricklecharge_current_ua;
	int precharge_current_ua;
	int precharge_voltage_max_uv;
	int charge_term_current_ua;
	int charge_restart_voltage_uv;
	int overvoltage_limit_uv;
	int constant_charge_current_max_ua;
	int constant_charge_voltage_max_uv;
	int temp_ambient_alert_min;
	int temp_ambient_alert_max;
	int temp_alert_min;
	int temp_alert_max;
	int temp_min;
	int temp_max;
	struct power_supply_resistance_temp_table *resist_table;
	int resist_table_size;
};
extern int power_supply_get_battery_info(struct power_supply *psy,
		struct power_supply_battery_info *info);


struct power_supply_config {
	struct device_node *of_node;
	struct fwnode_handle *fwnode;
	void *drv_data;
	const struct attribute_group **attr_grp;
	char **supplied_to;
	size_t num_supplicants;
};


#ifndef _BQ25756_CHARGER_H
#define _BQ25756_CHARGER_H
#define BQ25756_MANUFACTURER	"Texas Instruments"
#define BQ25756_ADD 0x6b
#define BQ25756_CHRG_V_LIM_MSB	0x00
#define BQ25756_CHRG_V_LIM_LSB	0x01
#define BQ25756_CHRG_I_LIM_MSB	0x02
#define BQ25756_CHRG_I_LIM_LSB	0x03
#define BQ25756_INPUT_I_DPM_LIM_MSB	0x06
#define BQ25756_INPUT_I_DPM_LIM_LSB	0x07
#define BQ25756_INPUT_V_DPM_LIM_MSB	0x08
#define BQ25756_INPUT_V_DPM_LIM_LSB	0x09
#define BQ25756_REV_INPUT_I_LIM_MSB	0x0A
#define BQ25756_REV_INPUT_I_LIM_LSB	0x0B
#define BQ25756_REV_INPUT_V_LIM_MSB	0x0C
#define BQ25756_REV_INPUT_V_LIM_LSB	0x0D
#define BQ25756_PRECHRG_I_LIM_MSB	0x10
#define BQ25756_PRECHRG_I_LIM_LSB	0x11
#define BQ25756_TERM_I_LIM_MSB	0x12
#define BQ25756_TERM_I_LIM_LSB	0x13
#define BQ25756_RECHRG_AND_TERM_CTRL	0x14
#define BQ25756_TIMER_CTRL	0x15
#define BQ25756_THREE_STAGE_CHRG_CTRL	0x16
#define BQ25756_CHRG_CTRL	0x17
#define BQ25756_PIN_CTRL	0x18
#define BQ25756_POW_AND_REV_CTRL	0x19
#define BQ25756_MPPT_CTRL	0x1A
#define BQ25756_TS_CHAG_THRESHOLD_CTRL	0x1B
#define BQ25756_TS_CHAG_BEHAVIOR_CTRL	0x1C
#define BQ25756_REV_THRESHOLD_CTRL	0x1D
#define BQ25756_REV_UNDER_V_CONTROL	0x1E
#define BQ25756_VAC_MAX_POWER_POINT_DETECTED_MSB 0x1F
#define BQ25756_VAC_MAX_POWER_POINT_DETECTED_LSB 0x20
#define BQ25756_CHRG_STAT_1	0x21
#define BQ25756_CHRG_STAT_2	0x22
#define BQ25756_CHRG_STAT_3	0x23
#define BQ25756_FAULT_STAT	0x24
#define BQ25756_CHRG_FLAG_1	0x25
#define BQ25756_CHRG_FLAG_2	0x26
#define BQ25756_FAULT_FLAG	0x27
#define BQ25756_CHRG_MSK_1	0x28
#define BQ25756_CHRG_MSK_2	0x29
#define BQ25756_FAULT_MSK_0	0x2A
#define BQ25756_ADC_CTRL	0x2B
#define BQ25756_ADC_CHANNEL_CTRL	0x2C
#define BQ25756_ADC_IAC_MSB		0x2D
#define BQ25756_ADC_IAC_LSB		0x2E
#define BQ25756_ADC_IBAT_MSB	0x2F
#define BQ25756_ADC_IBAT_LSB	0x30
#define BQ25756_ADC_VAC_MSB	0x31
#define BQ25756_ADC_VAC_LSB	0x32
#define BQ25756_ADC_VBAT_MSB	0x33
#define BQ25756_ADC_VBAT_LSB	0x34
#define BQ25756_ADC_TS_MSB		0x37
#define BQ25756_ADC_TS_LSB		0x38
#define BQ25756_ADC_VFB_MSB		0x39
#define BQ25756_ADC_VFB_LSB		0x3A
#define BQ25756_GATE_DRV_STRENGTH_CTRL	0x3B
#define BQ25756_GATE_DEAD_TIME_CTRL	0x3C
#define BQ25756_PART_INFO	0x3D
#define BQ25756_REV_DISCHAG_CURRENT	0x62

#define BQ25756_CHRG_EN		BIT(0)
#define BQ25756_ADC_EN		BIT(7)

#define BQ25756_CHRG_I_LIM_MOVE_STEP		2
#define BQ25756_INPUT_I_DPM_LIM_MOVE_STEP	2
#define BQ25756_INPUT_V_DPM_LIM_MOVE_STEP	2
#define BQ25756_PRECHRG_I_LIM_MOVE_STEP		2
#define BQ25756_TERM_I_LIM_MOVE_STEP		2

#define BIT_MASK(bit) (1UL << (bit))
#define BIT_RANGE_MASK(h, l) (BIT_MASK((h) + 1) - BIT_MASK(l))
#define BQ25756_CHG_STAT_MSK	BIT_RANGE_MASK(2, 0)
#define BQ25756_NOT_CHRGING	0
#define BQ25756_TRICKLE_CHRG	BIT(0)
#define BQ25756_PRECHRG		BIT(1)
#define BQ25756_FAST_CHRG	(BIT(0) | BIT(1))
#define BQ25756_TAPER_CHRG	BIT(2)
#define BQ25756_FLOAT_CHRG	(BIT(2) | BIT(0))
#define BQ25756_TOP_OFF_CHRG	(BIT(1) | BIT(2))
#define BQ25756_TERM_CHRG	(BIT(0) | BIT(1) | BIT(2))
#define BQ25756_VBUS_PRESENT	BIT(0)

#define BQ25756_TEMP_WARM	BIT(0)
#define BQ25756_TEMP_COOL	BIT(1)
#define BQ25756_TEMP_COLD	(BIT(0) | BIT(1))
#define BQ25756_TEMP_HOT	BIT(2)
#define BQ25756_TEMP_MASK	BIT_RANGE_MASK(6, 4)
#define BQ25756_TEMP_MOVE_STEP	4

#define BQ25756_VBAT_OV_STAT	BIT(4)
#define BQ25756_VAC_OV_STAT		BIT(6)
#define BQ25756_PG_STAT_STAT		BIT(7)

#define BQ25756_PRECHRG_CUR_MASK		BIT_RANGE_MASK(9, 2)
#define BQ25756_PRECHRG_CURRENT_STEP_uA		50000
#define BQ25756_PRECHRG_I_MIN_uA		250000
#define BQ25756_PRECHRG_I_MAX_uA		10000000
#define BQ25756_PRECHRG_I_DEF_uA		4000000

#define BQ25756_TERMCHRG_CUR_MASK		BIT_RANGE_MASK(9, 2)
#define BQ25756_TERMCHRG_CURRENT_STEP_uA	50000
#define BQ25756_TERMCHRG_I_MIN_uA		250000
#define BQ25756_TERMCHRG_I_MAX_uA		10000000
#define BQ25756_TERMCHRG_I_DEF_uA		2000000

#define BQ25756_ICHRG_CURRENT_STEP_uA		50000
#define BQ25756_ICHRG_I_MIN_uA			400000
#define BQ25756_ICHRG_I_MAX_uA			20000000
#define BQ25756_ICHRG_I_DEF_uA			20000000

#define BQ25756_VREG_V_MAX_uV	1566000
#define BQ25756_VREG_V_MIN_uV	1504000
#define BQ25756_VREG_V_DEF_uV	1536000
#define BQ25756_VREG_V_STEP_uV	2000

#define BQ25756_IINDPM_I_MIN_uA	400000
#define BQ25756_IINDPM_I_MAX_uA	20000000
#define BQ25756_IINDPM_STEP_uA	50000
#define BQ25756_IINDPM_DEF_uA	20000000

#define BQ25756_VINDPM_V_MIN_uV 4200000
#define BQ25756_VINDPM_V_MAX_uV 65000000
#define BQ25756_VINDPM_STEP_uV	20000
#define BQ25756_VINDPM_DEF_uV	4200000

#define BQ25756_ADC_VOLT_STEP_uV	2000
#define BQ25756_ADC_CURR_STEP_uA	2000

#define BQ25756_WATCHDOG_MASK	BIT_RANGE_MASK(5, 4)
#define BQ25756_WATCHDOG_DIS	0
#define BQ25756_WATCHDOG_MAX	160000

int bq25756_get_vbat_adc(unsigned char vbat_adc_lsb, unsigned char vbat_adc_msb);
int bq25756_get_vac_adc(unsigned char vac_adc_lsb, unsigned char vac_adc_msb);
int bq25756_get_ibat_adc(unsigned char ibat_adc_lsb, unsigned char ibat_adc_msb);
int bq25756_set_term_curr(unsigned char iterm_lsb, unsigned char iterm_msb,int term_current);
int bq25756_get_term_curr(unsigned char iterm_lsb, unsigned char iterm_msb);
int bq25756_set_prechrg_curr(unsigned char iprechg_lsb, unsigned char iprechg_msb, int pre_current);
int bq25756_get_prechrg_curr(unsigned char iprechg_lsb, unsigned char iprechg_msb);
int bq25756_set_ichrg_curr(int chrg_curr, unsigned char ichg_msb, unsigned char ichg_lsb);
int bq25756_get_ichg_curr(unsigned char ichg_lsb, unsigned char ichg_msb);
int bq25756_set_chrg_volt(unsigned char vlim_lsb, unsigned char vlim_msb, int chrg_volt);
int bq25756_get_chrg_volt(unsigned char vlim_lsb, unsigned char vlim_msb);
int bq25756_set_input_volt_lim(unsigned char vlim_lsb, unsigned char vlim_msb, int vindpm);
int bq25756_get_input_volt_lim(unsigned char vlim_lsb, unsigned char vlim_msb);
int bq25756_set_input_curr_lim(unsigned char ilim_lsb, unsigned char ilim_msb, int iindpm);
int bq25756_get_input_curr_lim(unsigned char ilim_msb, unsigned char ilim_lsb);
int bq25756_get_state(unsigned char chrg_stat_2, unsigned char chrg_ctrl, unsigned char chrg_stat_1, unsigned char fault);
int bq25756_hw_init(struct bq25756_device *bq);
int bq25756_get_property(struct power_supply *psy, enum power_supply_property psp, union power_supply_propval *val);
int bq25756_set_property(struct power_supply *psy, enum power_supply_property prop, const union power_supply_propval *val);
int bq25756_get_state(unsigned char chrg_stat_2, unsigned char chrg_ctrl, unsigned char chrg_stat_1, unsigned char fault);

#endif
