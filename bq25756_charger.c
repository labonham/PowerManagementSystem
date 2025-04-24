#include "FreeRTOS.h"
#include "bq25756_charger.h"
#define	EINVAL		22
typedef unsigned char  u8;

#define BQ25756_NUM_WD_VAL	8

enum bq25756_id {
	BQ25756,
};
static int bq25756_watchdog_time[BQ25756_NUM_WD_VAL] = {
		0,
		40000,
		80000,
		160000
};

int bq25756_get_vbat_adc(unsigned char vbat_adc_lsb, unsigned char vbat_adc_msb)
{
	int ret;
	u16 vbat_adc;

	vbat_adc_msb = 1;
	ret = BQ25756_ReadReg(BQ25756_ADD, BQ25756_ADC_VBAT_MSB, &vbat_adc_msb, 1);
	if (ret)
		return ret;
	ret = BQ25756_ReadReg(BQ25756_ADD, BQ25756_ADC_VBAT_LSB, &vbat_adc_lsb, 1);
	if (ret)
		return ret;
	vbat_adc = vbat_adc_msb | vbat_adc_lsb;
	return vbat_adc * BQ25756_ADC_VOLT_STEP_uV;
}

int bq25756_get_vac_adc(unsigned char vac_adc_lsb, unsigned char vac_adc_msb)
{
	int ret;
	u16 vac_adc;
	
	
	ret = BQ25756_ReadReg(BQ25756_ADD, BQ25756_ADC_VAC_MSB, &vac_adc_msb, 1);
	if (ret)
		return ret;
	ret = BQ25756_ReadReg(BQ25756_ADD, BQ25756_ADC_VAC_LSB, &vac_adc_lsb, 1);
	if (ret)
		return ret;
	vac_adc = vac_adc_msb | vac_adc_lsb;

	return vac_adc * BQ25756_ADC_VOLT_STEP_uV;
}

int bq25756_get_ibat_adc(unsigned char ibat_adc_lsb, unsigned char ibat_adc_msb)
{
	int ret;
	u16 ibat_adc;

	ret = BQ25756_ReadReg(BQ25756_ADD, BQ25756_ADC_IBAT_MSB, &ibat_adc_msb, 1);
	if (ret)
		return ret;
	ret = BQ25756_ReadReg(BQ25756_ADD, BQ25756_ADC_IBAT_LSB, &ibat_adc_lsb, 1);
	if (ret)
		return ret;
	ibat_adc = ibat_adc_msb | ibat_adc_lsb;

	return ibat_adc * BQ25756_ADC_CURR_STEP_uA;
}

int bq25756_set_term_curr(unsigned char iterm_lsb,
		unsigned char iterm_msb, int term_current)
{
	int ret;
	u8 iterm;

	term_current = clamp(term_current, BQ25756_TERMCHRG_I_MIN_uA,
					BQ25756_TERMCHRG_I_MAX_uA);
	iterm = (term_current / BQ25756_TERMCHRG_CURRENT_STEP_uA) <<
		BQ25756_TERM_I_LIM_MOVE_STEP;
	iterm_msb = (iterm >> 8) & 0xff;
	ret = BQ25756_WriteReg(BQ25756_ADD, BQ25756_TERM_I_LIM_MSB, iterm_msb);
	if (ret)
		return ret;
	iterm_lsb = iterm & 0xff;

	return BQ25756_WriteReg(BQ25756_ADD, BQ25756_TERM_I_LIM_LSB, iterm_lsb);
}


int bq25756_get_term_curr(unsigned char iterm_lsb, unsigned char iterm_msb)
{
	int ret;
	int iterm;

	ret = BQ25756_ReadReg(BQ25756_ADD, BQ25756_TERM_I_LIM_LSB, &iterm_lsb, 1);
	if (ret)
		return ret;
	ret = BQ25756_ReadReg(BQ25756_ADD, BQ25756_TERM_I_LIM_MSB, &iterm_msb, 1);
	if (ret)
		return ret;
	iterm = (iterm_msb | iterm_lsb) >> BQ25756_TERM_I_LIM_MOVE_STEP;

	return iterm * BQ25756_TERMCHRG_CURRENT_STEP_uA;
}

int bq25756_set_prechrg_curr(unsigned char iprechg_lsb,
		unsigned char iprechg_msb, int pre_current)
{
	int ret;
	int iprechg;

	pre_current = clamp(pre_current, BQ25756_PRECHRG_I_MIN_uA,
					BQ25756_PRECHRG_I_MAX_uA);
	iprechg = (pre_current / BQ25756_PRECHRG_CURRENT_STEP_uA) <<
		BQ25756_PRECHRG_I_LIM_MOVE_STEP;
	iprechg_msb = (iprechg >> 8) & 0xff;
	ret = BQ25756_WriteReg(BQ25756_ADD, BQ25756_CHRG_I_LIM_MSB, iprechg_msb);
	if (ret)
		return ret;
	iprechg_lsb = iprechg & 0xff;

	return BQ25756_WriteReg(BQ25756_ADD, BQ25756_CHRG_I_LIM_LSB, iprechg_lsb);
}

int bq25756_get_prechrg_curr(unsigned char iprechg_lsb,
			unsigned char iprechg_msb)
{
	int ret;
	int iprechg;

	ret = BQ25756_ReadReg(BQ25756_ADD, BQ25756_PRECHRG_I_LIM_LSB, &iprechg_lsb, 1);
	if (ret)
		return ret;
	ret = BQ25756_ReadReg(BQ25756_ADD, BQ25756_PRECHRG_I_LIM_MSB, &iprechg_msb, 1);
	if (ret)
		return ret;
	iprechg = (iprechg_msb | iprechg_lsb) >>
			BQ25756_PRECHRG_I_LIM_MOVE_STEP;

	return iprechg * BQ25756_PRECHRG_CURRENT_STEP_uA;
}

int bq25756_set_ichrg_curr(int chrg_curr, unsigned char ichg_msb,
				unsigned char ichg_lsb)
{
	struct bq25756_device *bq;
	int chrg_curr_max = bq->init_data.ichg_max;
	int ichg;
	int ret;

	chrg_curr = clamp(chrg_curr, BQ25756_ICHRG_I_MIN_uA, chrg_curr_max);
	ichg = (chrg_curr / BQ25756_ICHRG_CURRENT_STEP_uA) <<
		BQ25756_CHRG_I_LIM_MOVE_STEP;
	ichg_msb = (ichg >> 8) & 0xff;
	ret = BQ25756_WriteReg(BQ25756_ADD, BQ25756_CHRG_I_LIM_MSB, ichg_msb);
	if (ret)
		return ret;
	ichg_lsb = ichg & 0xff;

	return BQ25756_WriteReg(BQ25756_ADD, BQ25756_CHRG_I_LIM_LSB, ichg_lsb);
}

int bq25756_get_ichg_curr(unsigned char ichg_lsb, unsigned char ichg_msb)
{
	int ret;
	int ichg;

	ret = BQ25756_ReadReg(BQ25756_ADD, BQ25756_CHRG_I_LIM_LSB, &ichg_lsb, 1);
	if (ret)
		return ret;
	ret = BQ25756_ReadReg(BQ25756_ADD, BQ25756_CHRG_I_LIM_MSB, &ichg_msb, 1);
	if (ret)
		return ret;
	ichg = (ichg_msb | ichg_lsb) >> BQ25756_CHRG_I_LIM_MOVE_STEP;

	return ichg * BQ25756_ICHRG_CURRENT_STEP_uA;
}

int bq25756_set_chrg_volt(unsigned char vlim_lsb,
			unsigned char vlim_msb, int chrg_volt)
{
	struct bq25756_device *bq;
	int chrg_volt_max = bq->init_data.vreg_max;
	int vlim;
	int ret;

	chrg_volt = clamp(chrg_volt, BQ25756_VREG_V_MIN_uV, chrg_volt_max);
	vlim = chrg_volt / BQ25756_VREG_V_STEP_uV;
	vlim_msb = (vlim >> 8) & 0xff;
	ret = BQ25756_WriteReg(BQ25756_ADD, BQ25756_CHRG_V_LIM_MSB, vlim_msb);
	if (ret)
		return ret;
	vlim_lsb = vlim & 0xff;

	return BQ25756_WriteReg(BQ25756_ADD, BQ25756_CHRG_V_LIM_LSB, vlim_lsb);
}

int bq25756_get_chrg_volt(unsigned char vlim_lsb, unsigned char vlim_msb)
{
	int ret;
	int chrg_volt;

	ret = BQ25756_ReadReg(BQ25756_ADD, BQ25756_CHRG_V_LIM_MSB, &vlim_msb, 1);
	if (ret)
		return ret;
	ret = BQ25756_ReadReg(BQ25756_ADD, BQ25756_CHRG_V_LIM_LSB, &vlim_lsb, 1);
	if (ret)
		return ret;
	chrg_volt = vlim_msb | vlim_lsb;

	return chrg_volt * BQ25756_VREG_V_STEP_uV;
}

int bq25756_set_input_volt_lim(unsigned char vlim_lsb,
			unsigned char vlim_msb, int vindpm)
{
	int ret;
	int vlim;

	vindpm = clamp(vindpm, BQ25756_VINDPM_V_MIN_uV,
						BQ25756_VINDPM_V_MAX_uV);
	vlim = (vindpm / BQ25756_VINDPM_STEP_uV) <<
		BQ25756_INPUT_V_DPM_LIM_MOVE_STEP;
	vlim_msb = (vlim >> 8) & 0xff;
	ret = BQ25756_WriteReg(BQ25756_ADD, BQ25756_INPUT_V_DPM_LIM_MSB, vlim_msb);
	if (ret)
		return ret;
	vlim_lsb = vlim & 0xff;

	return BQ25756_WriteReg(BQ25756_ADD, BQ25756_INPUT_V_DPM_LIM_LSB, vlim_lsb);
}

int bq25756_get_input_volt_lim(unsigned char vlim_lsb,
				unsigned char vlim_msb)
{
	int ret;
	u16 ilim;

	ret = BQ25756_ReadReg(BQ25756_ADD, BQ25756_INPUT_V_DPM_LIM_MSB, &vlim_msb, 1);
	if (ret)
		return ret;
	ret = BQ25756_ReadReg(BQ25756_ADD, BQ25756_INPUT_V_DPM_LIM_LSB, &vlim_lsb, 1);
	if (ret)
		return ret;
	ilim = (vlim_msb | vlim_lsb) >>
		BQ25756_INPUT_V_DPM_LIM_MOVE_STEP;

	return ilim * BQ25756_VINDPM_STEP_uV;
}

int bq25756_set_input_curr_lim(unsigned char ilim_lsb,
				unsigned char ilim_msb, int iindpm)
{
	int ret;
	int ilim;

	iindpm = clamp(iindpm, BQ25756_IINDPM_I_MIN_uA,
			BQ25756_IINDPM_I_MAX_uA);
	ilim = (iindpm / BQ25756_IINDPM_STEP_uA) <<
		BQ25756_INPUT_I_DPM_LIM_MOVE_STEP;
	ilim_msb = (ilim >> 8) & 0xff;
	ret = BQ25756_WriteReg(BQ25756_ADD, BQ25756_INPUT_I_DPM_LIM_MSB, ilim_msb);
	if (ret)
		return ret;
	ilim_lsb = ilim & 0xff;

	return BQ25756_WriteReg(BQ25756_ADD, BQ25756_INPUT_I_DPM_LIM_LSB, ilim_lsb);
}

int bq25756_get_input_curr_lim(unsigned char ilim_msb, unsigned char ilim_lsb)
{ 
	int ret;
	u16 ilim;

	ret = BQ25756_ReadReg(BQ25756_ADD, BQ25756_INPUT_I_DPM_LIM_MSB, &ilim_msb, 1);
	if (ret)
		return ret;
	ret = BQ25756_ReadReg(BQ25756_ADD, BQ25756_INPUT_I_DPM_LIM_LSB, &ilim_lsb, 1);
	if (ret)
		return ret;
	ilim = (ilim_msb| ilim_lsb) >>
		BQ25756_INPUT_I_DPM_LIM_MOVE_STEP;

	return ilim * BQ25756_IINDPM_STEP_uA;
}

int bq25756_get_state(unsigned char chrg_stat_2, unsigned char chrg_ctrl,
			unsigned char chrg_stat_1, unsigned char fault)
{
	struct bq25756_state *state;
	int ret;
	int b = (1<<7);
	unsigned char a = b;
	unsigned char lsb, msb;

	a &= ~(1<<7);
	ret = BQ25756_WriteReg(BQ25756_ADD, BQ25756_ADC_CTRL, a);
	if (ret)
		return ret;
	ret = BQ25756_ReadReg(BQ25756_ADD, BQ25756_CHRG_STAT_2, &chrg_stat_2, 1);
	if (ret)
		return ret;
	ret = BQ25756_ReadReg(BQ25756_ADD, BQ25756_CHRG_CTRL, &chrg_ctrl, 1);
	if (ret)
		return ret;
	if (chrg_ctrl & BQ25756_CHRG_EN)
		state->chrg_status = chrg_stat_2 & BQ25756_CHG_STAT_MSK;
	else
		state->chrg_status = BQ25756_NOT_CHRGING;
	ret = BQ25756_ReadReg(BQ25756_ADD, BQ25756_CHRG_STAT_1, &chrg_stat_1, 1);
	if (ret)
		return ret;
	state->health = (chrg_stat_1 & BQ25756_TEMP_MASK) >>
			BQ25756_TEMP_MOVE_STEP;
	ret = BQ25756_ReadReg(BQ25756_ADD, BQ25756_FAULT_STAT, &fault, 1);
	if (ret)
		return ret;
	state->fault_0 = fault;
	state->online = chrg_stat_2 & BQ25756_PG_STAT_STAT;
	state->vbat_adc = bq25756_get_vbat_adc(lsb, msb);
	state->vac_adc = bq25756_get_vac_adc(lsb, msb);
	state->ibat_adc = bq25756_get_ibat_adc(lsb, msb);

	return 0;
}


int bq25756_set_property(struct power_supply *psy,
		enum power_supply_property prop,
		const union power_supply_propval *val)
{
	int ret = -EINVAL;
	unsigned char lsb, msb;

	switch (prop) {
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		ret = bq25756_set_input_curr_lim(lsb, msb, val->intval);
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
		ret = bq25756_set_chrg_volt(lsb, msb, val->intval);
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		ret = bq25756_set_ichrg_curr(lsb, msb, val->intval);
		break;
	case POWER_SUPPLY_PROP_PRECHARGE_CURRENT:
		ret = bq25756_set_prechrg_curr(lsb, msb, val->intval);
		break;
	case POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT:
		ret = bq25756_set_term_curr(lsb, msb, val->intval);
		break;
	case POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT:
		ret = bq25756_set_input_volt_lim(lsb, msb, val->intval);
		break;
	default:
		return -EINVAL;
	}

	return ret;
}

int bq25756_get_property(struct power_supply *psy,
				enum power_supply_property psp,
				union power_supply_propval *val)
{
	unsigned char lsb, msb;
	struct bq25756_state state;
	int ret = 0;
	unsigned char chrg_stat_2;
	unsigned char chrg_ctrl;
	unsigned char chrg_stat_1;
	unsigned char fault;
	struct bq25756_device *bq;

	ret = bq25756_get_state(chrg_stat_2, chrg_ctrl, chrg_stat_1, fault);
	if (ret)
		return ret;
	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		if (!state.chrg_status)
			val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
		else if (state.chrg_status == BQ25756_TERM_CHRG)
			val->intval = POWER_SUPPLY_STATUS_FULL;
		else
			val->intval = POWER_SUPPLY_STATUS_CHARGING;
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		switch (state.chrg_status) {
		case BQ25756_TRICKLE_CHRG:
			val->intval = POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
			break;
		case BQ25756_PRECHRG:
			val->intval = POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
			break;
		case BQ25756_FAST_CHRG:
			val->intval = POWER_SUPPLY_CHARGE_TYPE_FAST;
			break;
		case BQ25756_TAPER_CHRG:
			val->intval = POWER_SUPPLY_CHARGE_TYPE_STANDARD;
			break;
		case BQ25756_FLOAT_CHRG:
			val->intval = POWER_SUPPLY_CHARGE_TYPE_FAST;
			break;
		case BQ25756_TOP_OFF_CHRG:
			val->intval = POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
			break;
		case BQ25756_NOT_CHRGING:
			val->intval = POWER_SUPPLY_CHARGE_TYPE_NONE;
			break;
		default:
			val->intval = POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;
		}
		break;
	case POWER_SUPPLY_PROP_MANUFACTURER:
		val->strval = BQ25756_MANUFACTURER;
		break;
		case POWER_SUPPLY_PROP_MODEL_NAME:
		val->strval = bq->model_name;
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = state.online;
		break;
	case POWER_SUPPLY_PROP_USB_TYPE:
		if (!state.chrg_type) {
			val->intval = POWER_SUPPLY_USB_TYPE_UNKNOWN;
			break;
		}
	case POWER_SUPPLY_PROP_HEALTH:
		if (state.fault_0 & (BQ25756_VBAT_OV_STAT |
			BQ25756_VAC_OV_STAT))
			val->intval = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
		else
			val->intval = POWER_SUPPLY_HEALTH_GOOD;
		switch (state.health) {
		case BQ25756_TEMP_HOT:
			val->intval = POWER_SUPPLY_HEALTH_HOT;
			break;
		case BQ25756_TEMP_WARM:
			val->intval = POWER_SUPPLY_HEALTH_WARM;
			break;
		case BQ25756_TEMP_COOL:
			val->intval = POWER_SUPPLY_HEALTH_COOL;
			break;
		case BQ25756_TEMP_COLD:
			val->intval = POWER_SUPPLY_HEALTH_COLD;
			break;
		}
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		ret = bq25756_get_ichg_curr(lsb, msb);
		if (ret < 0)
			return ret;
		val->intval = ret;
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
		ret = bq25756_get_chrg_volt(lsb, msb);
		if (ret < 0)
			return ret;
		val->intval = ret;
		break;
	case POWER_SUPPLY_PROP_PRECHARGE_CURRENT:
		ret = bq25756_get_prechrg_curr(lsb, msb);
		if (ret < 0)
			return ret;
		val->intval = ret;
		break;
	case POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT:
		ret = bq25756_get_term_curr(lsb, msb);
		if (ret < 0)
			return ret;
		val->intval = ret;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = state.vac_adc;
		break;
	case POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT:
		ret = bq25756_get_input_volt_lim(lsb, msb);
		if (ret < 0)
			return ret;
		val->intval = ret;
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		ret = bq25756_get_input_curr_lim(lsb, msb);
		if (ret < 0)
			return ret;
		val->intval = ret;
		break;
	default:
		return -EINVAL;
	}

	return ret;
}

int bq25756_battery_get_property(struct power_supply *psy,
				       enum power_supply_property psp,
				       union power_supply_propval *val)
{
	unsigned char chrg_stat_2, chrg_ctrl, chrg_stat_1, fault;
	struct bq25756_state state;
	struct bq25756_device *bq;
	int ret = 0;

	ret = bq25756_get_state(chrg_stat_2, chrg_ctrl, chrg_stat_1, fault);
	if (ret)
		return ret;
	switch (psp) {
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
	val->intval = bq->init_data.ichg_max;
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX:
		val->intval = bq->init_data.vreg_max;
	break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = state.vbat_adc;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = state.ibat_adc;
		break;
	default:
		return -EINVAL;
	}

	return ret;
}

int bq25756_hw_init(struct bq25756_device *bq)
{
	struct power_supply_battery_info bat_info = { 1 };
	int wd_reg_val = BQ25756_WATCHDOG_DIS;
	unsigned char lsb, msb;
	unsigned char a = BIT_RANGE_MASK(5, 4);
	int wd_max_val = BQ25756_NUM_WD_VAL - 1;
	int ret = 0;
	int i;

	if (0 == 1)
		return wd_reg_val;
	if (bq->watchdog_timer) {
		if (bq->watchdog_timer >= bq25756_watchdog_time[wd_max_val])
			wd_reg_val = wd_max_val;
		else {
			for (i = 0; i < wd_max_val; i++) {
				if (bq->watchdog_timer >
					bq25756_watchdog_time[i] &&
				    bq->watchdog_timer <
					bq25756_watchdog_time[i + 1]) {
					wd_reg_val = i;
					break;
				}
			}
		}
	}
	a &= ~wd_reg_val;
	ret = BQ25756_WriteReg(BQ25756_ADD, BQ25756_TIMER_CTRL, a);
	if (ret)
		return ret;
	ret = bq25756_set_ichrg_curr(bat_info.constant_charge_current_max_ua,
		msb, lsb);
	if (ret)
		return ret;
	ret = bq25756_set_prechrg_curr(bat_info.precharge_current_ua, msb, lsb);
	if (ret)
		return ret;
	ret = bq25756_set_chrg_volt(bat_info.constant_charge_voltage_max_uv,
		msb, lsb);
	if (ret)
		return ret;
	ret = bq25756_set_term_curr(bat_info.charge_term_current_ua, msb, lsb);
	if (ret)
		return ret;
	ret = bq25756_set_input_volt_lim(bq->init_data.vlim, msb, lsb);
	if (ret)
		return ret;
	ret = bq25756_set_input_curr_lim(bq->init_data.ilim, msb, lsb);
	if (ret)
		return ret;

	return 0;
}
