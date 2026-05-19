// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * ADMV1355 Microwave Upconverter Driver
 *
 * Copyright (C) 2026 Analog Devices, Inc.
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/cleanup.h>
#include <linux/clk.h>
#include <linux/clk/clkscale.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/iio/iio.h>
#include <linux/module.h>
#include <linux/notifier.h>
#include <linux/regmap.h>
#include <linux/spi/spi.h>
#include <linux/units.h>

#define ADMV1355_REG_SDO_CTRL		0x000
#define ADMV1355_REG_PRODUCT_ID_LSB	0x004
#define ADMV1355_REG_SCRATCH_PAD	0x00A
#define ADMV1355_REG_NVM_CTRL		0x078
#define ADMV1355_REG_NVM_ADDR		0x07B
#define ADMV1355_REG_NVM_DATA		0x07C
#define ADMV1355_REG_NVM_LOAD		0x07E
#define ADMV1355_REG_RF_HS_PD		0x13E
#define ADMV1355_REG_FILTER_LUT_EN	0x202
#define ADMV1355_REG_FILTER_LOAD_EN	0x208
#define ADMV1355_REG_GAIN_LOAD_EN	0x281
#define ADMV1355_REG_GAIN_LUT_EN	0x285
#define ADMV1355_REG_GAIN_TBL_BYP	0x28A
#define ADMV1355_REG_DSA_BYPASS		0x28B
#define ADMV1355_REG_RF_LPF		0x2A0
#define ADMV1355_REG_RF_HPF		0x2A1
#define ADMV1355_REG_DSA_DIRECT		0x600
#define ADMV1355_REG_GPO_G_DIRECT	0x601
#define ADMV1355_REG_IF_ENABLE		0x602
#define ADMV1355_REG_LO_X4_FILTER	0x800
#define ADMV1355_REG_LO_X3_FILTER	0x801
#define ADMV1355_REG_LO_X1_FILTER	0x802
#define ADMV1355_REG_LO_TRAP_7_0	0x804
#define ADMV1355_REG_LO_TRAP_9_8	0x805
#define ADMV1355_REG_LO_BAND		0x806
#define ADMV1355_REG_LON_OFFSET_I	0x808
#define ADMV1355_REG_LON_OFFSET_Q	0x809
#define ADMV1355_REG_IF_GAIN		0x80A
#define ADMV1355_REG_RF_FILTER		0x80B
#define ADMV1355_REG_LO_DBL_BAND	0x80C

/* 0x28A: GAIN_LUT_BYPASS_EN */
#define ADMV1355_GAIN_LUT_BYP_EN_MSK	BIT(0)

/* 0x28B: DSA2_BYPASS_VALUE[7:4], DSA1_BYPASS_VALUE[3:0] */
#define ADMV1355_DSA1_BYPASS_MSK	GENMASK(3, 0)
#define ADMV1355_DSA2_BYPASS_MSK	GENMASK(7, 4)

/* 0x2A0: LPF_SELECT[4], LPF_BYPASS_VALUE[3:0] */
#define ADMV1355_LPF_SELECT_MSK		BIT(4)
#define ADMV1355_LPF_BYPASS_VAL_MSK	GENMASK(3, 0)

/* 0x2A1: HPF_SELECT[7], HPF_BYPASS_VALUE[5:0] */
#define ADMV1355_HPF_SELECT_MSK		BIT(7)
#define ADMV1355_HPF_BYPASS_VAL_MSK	GENMASK(5, 0)

/* 0x600: RF_DSA2_GAIN[7:4], RF_DSA1_GAIN[3:0] */
#define ADMV1355_DSA1_DIRECT_MSK	GENMASK(3, 0)
#define ADMV1355_DSA2_DIRECT_MSK	GENMASK(7, 4)

/* 0x602: IF_HYBRID_EN[3], IF_HIGH_FREQ_EN[2], IF_LOW_FREQ_EN[1], BB_EN[0] */
#define ADMV1355_BB_EN_MSK		BIT(0)
#define ADMV1355_IF_LOW_FREQ_EN_MSK	BIT(1)
#define ADMV1355_IF_HIGH_FREQ_EN_MSK	BIT(2)
#define ADMV1355_IF_HYBRID_EN_MSK	BIT(3)
#define ADMV1355_IF_MODE_MSK		GENMASK(3, 0)

/* 0x805: RF_BAND_SELECT[7], LO_PHASE_Q[6:2], LO_TRAP_FILTER[9:8] at [1:0] */
#define ADMV1355_RF_BAND_805_MSK	BIT(7)
#define ADMV1355_LO_PHASE_Q_MSK	GENMASK(6, 2)

/* 0x806: LO_BAND[6], MIXER_SIDEBAND[5], LO_PHASE_I[4:0] */
#define ADMV1355_LO_BAND_806_MSK	BIT(6)
#define ADMV1355_MIXER_SIDEBAND_MSK	BIT(5)
#define ADMV1355_LO_PHASE_I_MSK	GENMASK(4, 0)

/* 0x80A: DSAQ[7:4], DSAI[3:0] */
#define ADMV1355_DSAI_MSK		GENMASK(3, 0)
#define ADMV1355_DSAQ_MSK		GENMASK(7, 4)

#define ADMV1355_SCRATCH_TEST_VAL	0xA5

static const char * const admv1355_rf_band_items[] = {
	"low_band",
	"high_band",
};

static const char * const admv1355_mixer_sideband_items[] = {
	"LSB",
	"USB",
};

static const char * const admv1355_if_mode_items[] = {
	"baseband",
	"complex_if_low",
	"complex_if_high",
	"if_hybrid",
};

static const char * const admv1355_dsa_4bit_items[] = {
	"0dB", "-1dB", "-2dB", "-3dB", "-4dB", "-5dB", "-6dB", "-7dB",
	"-8dB", "-9dB", "-10dB", "-11dB", "-12dB", "-13dB", "-14dB", "-15dB"
};

static const char * const admv1355_dsa_iq_items[] = {
	"0dB", "-100mdB", "-200mdB", "-300mdB",
	"-400mdB", "-500mdB", "-600mdB", "-700mdB",
	"-800mdB", "-900mdB", "-1000mdB", "-1100mdB",
	"-1200mdB", "-1300mdB", "-1400mdB", "-1500mdB"
};

enum admv1355_ext_info {
	ADMV1355_RF_LPF_BYPASS_EN,
	ADMV1355_RF_LPF_BYPASS_VAL,
	ADMV1355_RF_HPF_BYPASS_EN,
	ADMV1355_RF_HPF_BYPASS_VAL,
	ADMV1355_LO_PHASE_I,
	ADMV1355_LO_PHASE_Q,
	ADMV1355_LO_LON_OFFSET_I,
	ADMV1355_LO_LON_OFFSET_Q,
};

enum admv1355_dev_attr_id {
	ADMV1355_DEV_ATTR_FILTER_LUT_EN,
	ADMV1355_DEV_ATTR_FILTER_LOAD_EN,
	ADMV1355_DEV_ATTR_GAIN_LUT_EN,
	ADMV1355_DEV_ATTR_GAIN_LUT_BYPASS_EN,
	ADMV1355_DEV_ATTR_GAIN_LOAD_EN,
};

struct admv1355_priv {
	struct spi_device	*spi;
	struct gpio_desc	*cen_gpio;
	struct gpio_desc	*reset_gpio;
	struct gpio_desc	*pwr_en_gpio;
	struct gpio_desc	*lvl_en_gpio;
	struct regmap		*regmap;
	struct clk		*lo_input;
	struct clock_scale	clkscale;
	struct notifier_block	nb;
	struct mutex		lock;
};

static int admv1355_spi_read(struct admv1355_priv *priv, unsigned int reg,
			     unsigned int *val)
{
	int ret;

	ret = regmap_read(priv->regmap, reg, val);
	if (ret)
		dev_err(&priv->spi->dev, "%s: REG 0x%03X read failed (%d)\n",
			__func__, reg, ret);
	else
		dev_dbg(&priv->spi->dev, "%s: REG 0x%03X VAL 0x%02X\n",
			__func__, reg, *val);

	return ret;
}

static int admv1355_spi_write(struct admv1355_priv *priv, unsigned int reg,
			      unsigned int val)
{
	int ret;

	ret = regmap_write(priv->regmap, reg, val);
	if (ret)
		dev_err(&priv->spi->dev, "%s: REG 0x%03X VAL 0x%02X write failed (%d)\n",
			__func__, reg, val, ret);
	else
		dev_dbg(&priv->spi->dev, "%s: REG 0x%03X VAL 0x%02X\n",
			__func__, reg, val);

	return ret;
}

static int admv1355_reg_access(struct iio_dev *indio_dev, unsigned int reg,
			       unsigned int write_val, unsigned int *read_val)
{
	struct admv1355_priv *priv = iio_priv(indio_dev);

	guard(mutex)(&priv->lock);

	if (read_val)
		return admv1355_spi_read(priv, reg, read_val);

	return admv1355_spi_write(priv, reg, write_val);
}

/*
 * Macro to generate get/set/iio_enum for simple register-mask-enum patterns
 */
#define ADMV1355_REG_ENUM(_name, _reg, _mask, _items)				\
static int admv1355_get_##_name(struct iio_dev *indio_dev,			\
				const struct iio_chan_spec *chan)			\
{										\
	struct admv1355_priv *priv = iio_priv(indio_dev);			\
	unsigned int data;							\
	int ret;								\
										\
	guard(mutex)(&priv->lock);						\
	ret = regmap_read(priv->regmap, _reg, &data);				\
	return ret ? ret : FIELD_GET(_mask, data);				\
}										\
										\
static int admv1355_set_##_name(struct iio_dev *indio_dev,			\
				const struct iio_chan_spec *chan, u32 val)	\
{										\
	struct admv1355_priv *priv = iio_priv(indio_dev);			\
										\
	guard(mutex)(&priv->lock);						\
	return regmap_update_bits(priv->regmap, _reg, _mask,			\
				  FIELD_PREP(_mask, val));			\
}										\
										\
static const struct iio_enum admv1355_##_name##_enum = {			\
	.items = _items,							\
	.num_items = ARRAY_SIZE(_items),					\
	.get = admv1355_get_##_name,						\
	.set = admv1355_set_##_name,						\
}

/* RF DSA enums */
ADMV1355_REG_ENUM(rf_direct_dsa1, ADMV1355_REG_DSA_DIRECT,
		  ADMV1355_DSA1_DIRECT_MSK, admv1355_dsa_4bit_items);
ADMV1355_REG_ENUM(rf_direct_dsa2, ADMV1355_REG_DSA_DIRECT,
		  ADMV1355_DSA2_DIRECT_MSK, admv1355_dsa_4bit_items);
ADMV1355_REG_ENUM(rf_bypass_dsa1, ADMV1355_REG_DSA_BYPASS,
		  ADMV1355_DSA1_BYPASS_MSK, admv1355_dsa_4bit_items);
ADMV1355_REG_ENUM(rf_bypass_dsa2, ADMV1355_REG_DSA_BYPASS,
		  ADMV1355_DSA2_BYPASS_MSK, admv1355_dsa_4bit_items);

/* Mixer sideband enum */
ADMV1355_REG_ENUM(mixer_sideband, ADMV1355_REG_LO_BAND,
		  ADMV1355_MIXER_SIDEBAND_MSK, admv1355_mixer_sideband_items);

/* IF fine-adjust DSA enums */
ADMV1355_REG_ENUM(dsai, ADMV1355_REG_IF_GAIN,
		  ADMV1355_DSAI_MSK, admv1355_dsa_iq_items);
ADMV1355_REG_ENUM(dsaq, ADMV1355_REG_IF_GAIN,
		  ADMV1355_DSAQ_MSK, admv1355_dsa_iq_items);

/* RF band select — controls bits in two registers (0x805[7] and 0x806[6]) */
static int admv1355_get_rf_band(struct iio_dev *indio_dev,
				const struct iio_chan_spec *chan)
{
	struct admv1355_priv *priv = iio_priv(indio_dev);
	unsigned int data;
	int ret;

	guard(mutex)(&priv->lock);
	ret = regmap_read(priv->regmap, ADMV1355_REG_LO_TRAP_9_8, &data);
	return ret ? ret : FIELD_GET(ADMV1355_RF_BAND_805_MSK, data);
}

static int admv1355_set_rf_band(struct iio_dev *indio_dev,
				const struct iio_chan_spec *chan,
				unsigned int val)
{
	struct admv1355_priv *priv = iio_priv(indio_dev);
	int ret;

	guard(mutex)(&priv->lock);
	ret = regmap_update_bits(priv->regmap, ADMV1355_REG_LO_TRAP_9_8,
				 ADMV1355_RF_BAND_805_MSK,
				 FIELD_PREP(ADMV1355_RF_BAND_805_MSK, val));
	if (ret)
		return ret;

	return regmap_update_bits(priv->regmap, ADMV1355_REG_LO_BAND,
				  ADMV1355_LO_BAND_806_MSK,
				  FIELD_PREP(ADMV1355_LO_BAND_806_MSK, val));
}

static const struct iio_enum admv1355_rf_band_enum = {
	.items = admv1355_rf_band_items,
	.num_items = ARRAY_SIZE(admv1355_rf_band_items),
	.get = admv1355_get_rf_band,
	.set = admv1355_set_rf_band,
};

/* IF mode select — maps enum index to individual enable bits in 0x602 */
static const u8 admv1355_if_mode_reg_values[] = {
	0x01, /* baseband */
	0x02, /* complex_if_low */
	0x04, /* complex_if_high */
	0x08, /* if_hybrid */
};

static int admv1355_get_if_mode(struct iio_dev *indio_dev,
				const struct iio_chan_spec *chan)
{
	struct admv1355_priv *priv = iio_priv(indio_dev);
	unsigned int data;
	int ret, i;

	guard(mutex)(&priv->lock);
	ret = regmap_read(priv->regmap, ADMV1355_REG_IF_ENABLE, &data);
	if (ret)
		return ret;

	data &= ADMV1355_IF_MODE_MSK;
	for (i = 0; i < ARRAY_SIZE(admv1355_if_mode_reg_values); i++) {
		if (admv1355_if_mode_reg_values[i] == data)
			return i;
	}

	return -EINVAL;
}

static int admv1355_set_if_mode(struct iio_dev *indio_dev,
				const struct iio_chan_spec *chan,
				unsigned int val)
{
	struct admv1355_priv *priv = iio_priv(indio_dev);

	if (val >= ARRAY_SIZE(admv1355_if_mode_reg_values))
		return -EINVAL;

	guard(mutex)(&priv->lock);
	return regmap_update_bits(priv->regmap, ADMV1355_REG_IF_ENABLE,
				  ADMV1355_IF_MODE_MSK,
				  admv1355_if_mode_reg_values[val]);
}

static const struct iio_enum admv1355_if_mode_enum = {
	.items = admv1355_if_mode_items,
	.num_items = ARRAY_SIZE(admv1355_if_mode_items),
	.get = admv1355_get_if_mode,
	.set = admv1355_set_if_mode,
};

static ssize_t admv1355_ext_info_read(struct iio_dev *indio_dev,
				      uintptr_t private,
				      const struct iio_chan_spec *chan,
				      char *buf)
{
	struct admv1355_priv *priv = iio_priv(indio_dev);
	unsigned int val, data;
	int ret;

	guard(mutex)(&priv->lock);

	switch (private) {
	case ADMV1355_RF_LPF_BYPASS_EN:
		ret = regmap_read(priv->regmap, ADMV1355_REG_RF_LPF, &data);
		if (ret)
			return ret;
		return sysfs_emit(buf, "%s\n",
				  FIELD_GET(ADMV1355_LPF_SELECT_MSK, data) ?
				  "true" : "false");
	case ADMV1355_RF_LPF_BYPASS_VAL:
		ret = regmap_read(priv->regmap, ADMV1355_REG_RF_LPF, &data);
		if (ret)
			return ret;
		val = FIELD_GET(ADMV1355_LPF_BYPASS_VAL_MSK, data);
		break;
	case ADMV1355_RF_HPF_BYPASS_EN:
		ret = regmap_read(priv->regmap, ADMV1355_REG_RF_HPF, &data);
		if (ret)
			return ret;
		return sysfs_emit(buf, "%s\n",
				  FIELD_GET(ADMV1355_HPF_SELECT_MSK, data) ?
				  "true" : "false");
	case ADMV1355_RF_HPF_BYPASS_VAL:
		ret = regmap_read(priv->regmap, ADMV1355_REG_RF_HPF, &data);
		if (ret)
			return ret;
		val = FIELD_GET(ADMV1355_HPF_BYPASS_VAL_MSK, data);
		break;
	case ADMV1355_LO_PHASE_I:
		ret = regmap_read(priv->regmap, ADMV1355_REG_LO_BAND, &data);
		if (ret)
			return ret;
		val = FIELD_GET(ADMV1355_LO_PHASE_I_MSK, data);
		break;
	case ADMV1355_LO_PHASE_Q:
		ret = regmap_read(priv->regmap, ADMV1355_REG_LO_TRAP_9_8,
				  &data);
		if (ret)
			return ret;
		val = FIELD_GET(ADMV1355_LO_PHASE_Q_MSK, data);
		break;
	case ADMV1355_LO_LON_OFFSET_I:
		ret = regmap_read(priv->regmap, ADMV1355_REG_LON_OFFSET_I,
				  &data);
		if (ret)
			return ret;
		val = data;
		break;
	case ADMV1355_LO_LON_OFFSET_Q:
		ret = regmap_read(priv->regmap, ADMV1355_REG_LON_OFFSET_Q,
				  &data);
		if (ret)
			return ret;
		val = data;
		break;
	default:
		return -EINVAL;
	}

	return sysfs_emit(buf, "%u\n", val);
}

static ssize_t admv1355_ext_info_write(struct iio_dev *indio_dev,
				       uintptr_t private,
				       const struct iio_chan_spec *chan,
				       const char *buf, size_t len)
{
	struct admv1355_priv *priv = iio_priv(indio_dev);
	unsigned int val;
	bool bval;
	int ret;

	guard(mutex)(&priv->lock);

	switch (private) {
	case ADMV1355_RF_LPF_BYPASS_EN:
		ret = kstrtobool(buf, &bval);
		if (ret)
			return ret;
		ret = regmap_update_bits(priv->regmap, ADMV1355_REG_RF_LPF,
					 ADMV1355_LPF_SELECT_MSK,
					 bval ? ADMV1355_LPF_SELECT_MSK : 0);
		break;
	case ADMV1355_RF_LPF_BYPASS_VAL:
		ret = kstrtouint(buf, 0, &val);
		if (ret)
			return ret;
		if (val > 15)
			return -EINVAL;
		ret = regmap_update_bits(priv->regmap, ADMV1355_REG_RF_LPF,
					 ADMV1355_LPF_BYPASS_VAL_MSK,
					 FIELD_PREP(ADMV1355_LPF_BYPASS_VAL_MSK,
						    val));
		break;
	case ADMV1355_RF_HPF_BYPASS_EN:
		ret = kstrtobool(buf, &bval);
		if (ret)
			return ret;
		ret = regmap_update_bits(priv->regmap, ADMV1355_REG_RF_HPF,
					 ADMV1355_HPF_SELECT_MSK,
					 bval ? ADMV1355_HPF_SELECT_MSK : 0);
		break;
	case ADMV1355_RF_HPF_BYPASS_VAL:
		ret = kstrtouint(buf, 0, &val);
		if (ret)
			return ret;
		if (val > 63)
			return -EINVAL;
		ret = regmap_update_bits(priv->regmap, ADMV1355_REG_RF_HPF,
					 ADMV1355_HPF_BYPASS_VAL_MSK,
					 FIELD_PREP(ADMV1355_HPF_BYPASS_VAL_MSK,
						    val));
		break;
	case ADMV1355_LO_PHASE_I:
		ret = kstrtouint(buf, 0, &val);
		if (ret)
			return ret;
		if (val > 31)
			return -EINVAL;
		ret = regmap_update_bits(priv->regmap, ADMV1355_REG_LO_BAND,
					 ADMV1355_LO_PHASE_I_MSK,
					 FIELD_PREP(ADMV1355_LO_PHASE_I_MSK,
						    val));
		break;
	case ADMV1355_LO_PHASE_Q:
		ret = kstrtouint(buf, 0, &val);
		if (ret)
			return ret;
		if (val > 31)
			return -EINVAL;
		ret = regmap_update_bits(priv->regmap, ADMV1355_REG_LO_TRAP_9_8,
					 ADMV1355_LO_PHASE_Q_MSK,
					 FIELD_PREP(ADMV1355_LO_PHASE_Q_MSK,
						    val));
		break;
	case ADMV1355_LO_LON_OFFSET_I:
		ret = kstrtouint(buf, 0, &val);
		if (ret)
			return ret;
		if (val > 255)
			return -EINVAL;
		ret = regmap_write(priv->regmap, ADMV1355_REG_LON_OFFSET_I,
				   val);
		break;
	case ADMV1355_LO_LON_OFFSET_Q:
		ret = kstrtouint(buf, 0, &val);
		if (ret)
			return ret;
		if (val > 255)
			return -EINVAL;
		ret = regmap_write(priv->regmap, ADMV1355_REG_LON_OFFSET_Q,
				   val);
		break;
	default:
		return -EINVAL;
	}

	return ret ? ret : len;
}

#define ADMV1355_EXT_INFO(_name, _ident) { \
	.name = _name, \
	.read = admv1355_ext_info_read, \
	.write = admv1355_ext_info_write, \
	.private = _ident, \
	.shared = IIO_SEPARATE, \
}

static const struct iio_chan_spec_ext_info admv1355_rf_ext_info[] = {
	IIO_ENUM("band", IIO_SEPARATE, &admv1355_rf_band_enum),
	IIO_ENUM_AVAILABLE("band", IIO_SEPARATE, &admv1355_rf_band_enum),
	IIO_ENUM("direct_dsa1_gain", IIO_SEPARATE,
		 &admv1355_rf_direct_dsa1_enum),
	IIO_ENUM_AVAILABLE("direct_dsa1_gain", IIO_SEPARATE,
			   &admv1355_rf_direct_dsa1_enum),
	IIO_ENUM("direct_dsa2_gain", IIO_SEPARATE,
		 &admv1355_rf_direct_dsa2_enum),
	IIO_ENUM_AVAILABLE("direct_dsa2_gain", IIO_SEPARATE,
			   &admv1355_rf_direct_dsa2_enum),
	IIO_ENUM("bypass_dsa1_gain", IIO_SEPARATE,
		 &admv1355_rf_bypass_dsa1_enum),
	IIO_ENUM_AVAILABLE("bypass_dsa1_gain", IIO_SEPARATE,
			   &admv1355_rf_bypass_dsa1_enum),
	IIO_ENUM("bypass_dsa2_gain", IIO_SEPARATE,
		 &admv1355_rf_bypass_dsa2_enum),
	IIO_ENUM_AVAILABLE("bypass_dsa2_gain", IIO_SEPARATE,
			   &admv1355_rf_bypass_dsa2_enum),
	ADMV1355_EXT_INFO("bypass_lpf_en", ADMV1355_RF_LPF_BYPASS_EN),
	ADMV1355_EXT_INFO("bypass_lpf_val", ADMV1355_RF_LPF_BYPASS_VAL),
	ADMV1355_EXT_INFO("bypass_hpf_en", ADMV1355_RF_HPF_BYPASS_EN),
	ADMV1355_EXT_INFO("bypass_hpf_val", ADMV1355_RF_HPF_BYPASS_VAL),
	{ }
};

static const struct iio_chan_spec_ext_info admv1355_if_ext_info[] = {
	IIO_ENUM("mode", IIO_SEPARATE, &admv1355_if_mode_enum),
	IIO_ENUM_AVAILABLE("mode", IIO_SEPARATE, &admv1355_if_mode_enum),
	{ }
};

static const struct iio_chan_spec_ext_info admv1355_lo_ext_info[] = {
	IIO_ENUM("sideband", IIO_SEPARATE, &admv1355_mixer_sideband_enum),
	IIO_ENUM_AVAILABLE("sideband", IIO_SEPARATE,
			   &admv1355_mixer_sideband_enum),
	IIO_ENUM("dsai_0p1db", IIO_SEPARATE, &admv1355_dsai_enum),
	IIO_ENUM_AVAILABLE("dsai_0p1db", IIO_SEPARATE, &admv1355_dsai_enum),
	IIO_ENUM("dsaq_0p1db", IIO_SEPARATE, &admv1355_dsaq_enum),
	IIO_ENUM_AVAILABLE("dsaq_0p1db", IIO_SEPARATE, &admv1355_dsaq_enum),
	ADMV1355_EXT_INFO("direct_i_phase_val", ADMV1355_LO_PHASE_I),
	ADMV1355_EXT_INFO("direct_q_phase_val", ADMV1355_LO_PHASE_Q),
	ADMV1355_EXT_INFO("direct_lon_offset_i", ADMV1355_LO_LON_OFFSET_I),
	ADMV1355_EXT_INFO("direct_lon_offset_q", ADMV1355_LO_LON_OFFSET_Q),
	{ }
};

/* Device-level attributes */
struct admv1355_attribute {
	enum admv1355_dev_attr_id id;
	struct device_attribute attr;
};

#define to_admv1355_attribute(x) container_of(x, struct admv1355_attribute, attr)

static ssize_t admv1355_dev_attr_show(struct device *dev,
				      struct device_attribute *dev_attr,
				      char *buf)
{
	const struct admv1355_attribute *attr = to_admv1355_attribute(dev_attr);
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct admv1355_priv *priv = iio_priv(indio_dev);
	unsigned int data;
	int ret;

	guard(mutex)(&priv->lock);

	switch (attr->id) {
	case ADMV1355_DEV_ATTR_FILTER_LUT_EN:
		ret = regmap_read(priv->regmap, ADMV1355_REG_FILTER_LUT_EN,
				  &data);
		if (ret)
			return ret;
		return sysfs_emit(buf, "%s\n",
				  (data & BIT(0)) ? "true" : "false");
	case ADMV1355_DEV_ATTR_FILTER_LOAD_EN:
		ret = regmap_read(priv->regmap, ADMV1355_REG_FILTER_LOAD_EN,
				  &data);
		if (ret)
			return ret;
		return sysfs_emit(buf, "%s\n",
				  (data & BIT(0)) ? "true" : "false");
	case ADMV1355_DEV_ATTR_GAIN_LUT_EN:
		ret = regmap_read(priv->regmap, ADMV1355_REG_GAIN_LUT_EN,
				  &data);
		if (ret)
			return ret;
		return sysfs_emit(buf, "%s\n",
				  (data & BIT(0)) ? "true" : "false");
	case ADMV1355_DEV_ATTR_GAIN_LUT_BYPASS_EN:
		ret = regmap_read(priv->regmap, ADMV1355_REG_GAIN_TBL_BYP,
				  &data);
		if (ret)
			return ret;
		return sysfs_emit(buf, "%s\n",
				  (data & ADMV1355_GAIN_LUT_BYP_EN_MSK) ?
				  "true" : "false");
	case ADMV1355_DEV_ATTR_GAIN_LOAD_EN:
		ret = regmap_read(priv->regmap, ADMV1355_REG_GAIN_LOAD_EN,
				  &data);
		if (ret)
			return ret;
		return sysfs_emit(buf, "%s\n",
				  (data & BIT(0)) ? "true" : "false");
	default:
		return -EINVAL;
	}
}

static ssize_t admv1355_dev_attr_store(struct device *dev,
				       struct device_attribute *dev_attr,
				       const char *buf, size_t len)
{
	const struct admv1355_attribute *attr = to_admv1355_attribute(dev_attr);
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct admv1355_priv *priv = iio_priv(indio_dev);
	bool bval;
	int ret;

	ret = kstrtobool(buf, &bval);
	if (ret)
		return ret;

	guard(mutex)(&priv->lock);

	switch (attr->id) {
	case ADMV1355_DEV_ATTR_FILTER_LUT_EN:
		ret = regmap_update_bits(priv->regmap,
					 ADMV1355_REG_FILTER_LUT_EN,
					 BIT(0), bval ? BIT(0) : 0);
		break;
	case ADMV1355_DEV_ATTR_FILTER_LOAD_EN:
		ret = regmap_update_bits(priv->regmap,
					 ADMV1355_REG_FILTER_LOAD_EN,
					 BIT(0), bval ? BIT(0) : 0);
		break;
	case ADMV1355_DEV_ATTR_GAIN_LUT_EN:
		ret = regmap_update_bits(priv->regmap,
					 ADMV1355_REG_GAIN_LUT_EN,
					 BIT(0), bval ? BIT(0) : 0);
		break;
	case ADMV1355_DEV_ATTR_GAIN_LUT_BYPASS_EN:
		ret = regmap_update_bits(priv->regmap,
					 ADMV1355_REG_GAIN_TBL_BYP,
					 ADMV1355_GAIN_LUT_BYP_EN_MSK,
					 bval ? ADMV1355_GAIN_LUT_BYP_EN_MSK
					       : 0);
		break;
	case ADMV1355_DEV_ATTR_GAIN_LOAD_EN:
		ret = regmap_update_bits(priv->regmap,
					 ADMV1355_REG_GAIN_LOAD_EN,
					 BIT(0), bval ? BIT(0) : 0);
		break;
	default:
		return -EINVAL;
	}

	return ret ? ret : len;
}

#define ADMV1355_ATTR(_name, _id, _mode) \
	struct admv1355_attribute dev_attr_##_name = { \
		.attr = __ATTR(_name, _mode, admv1355_dev_attr_show, \
			       admv1355_dev_attr_store), \
		.id = _id, \
	}

static ADMV1355_ATTR(filter_lut_en, ADMV1355_DEV_ATTR_FILTER_LUT_EN, 0644);
static ADMV1355_ATTR(filter_load_en, ADMV1355_DEV_ATTR_FILTER_LOAD_EN, 0644);
static ADMV1355_ATTR(gain_lut_en, ADMV1355_DEV_ATTR_GAIN_LUT_EN, 0644);
static ADMV1355_ATTR(gain_lut_bypass_en, ADMV1355_DEV_ATTR_GAIN_LUT_BYPASS_EN,
		     0644);
static ADMV1355_ATTR(gain_load_en, ADMV1355_DEV_ATTR_GAIN_LOAD_EN, 0644);

static struct attribute *admv1355_attributes[] = {
	&dev_attr_filter_lut_en.attr.attr,
	&dev_attr_filter_load_en.attr.attr,
	&dev_attr_gain_lut_en.attr.attr,
	&dev_attr_gain_lut_bypass_en.attr.attr,
	&dev_attr_gain_load_en.attr.attr,
	NULL,
};

static const struct attribute_group admv1355_attribute_group = {
	.attrs = admv1355_attributes,
};

static const struct iio_chan_spec admv1355_channels[] = {
	/* IF input channel */
	{
		.type = IIO_ALTVOLTAGE,
		.indexed = 1,
		.output = 0,
		.channel = 0,
		.extend_name = "if",
		.ext_info = admv1355_if_ext_info,
	},
	/* RF output channel */
	{
		.type = IIO_ALTVOLTAGE,
		.indexed = 1,
		.output = 1,
		.channel = 1,
		.extend_name = "rf",
		.ext_info = admv1355_rf_ext_info,
	},
	/* LO channel */
	{
		.type = IIO_ALTVOLTAGE,
		.indexed = 1,
		.output = 0,
		.channel = 2,
		.extend_name = "lo",
		.ext_info = admv1355_lo_ext_info,
	},
};

static const struct iio_info admv1355_info = {
	.debugfs_reg_access = &admv1355_reg_access,
	.attrs = &admv1355_attribute_group,
};

static int admv1355_set_filters(struct admv1355_priv *priv, u64 rate_khz)
{
	struct device *dev = &priv->spi->dev;
	u8 lo_trap_filter_7_0;
	u8 lo_trap_filter_9_8;
	u8 lo_doubler_band;
	u16 lo_trap_filter;
	u8 lo_x4_filter;
	u8 lo_x3_filter;
	u8 lo_x1_filter;
	int ret;

	if (rate_khz < (10000 * HZ_PER_KHZ)) {
		lo_x4_filter = 0x08;
		lo_x3_filter = 0x1C;
		lo_x1_filter = 0x1F;
		lo_trap_filter = 0x318;
		lo_doubler_band = 0x1F;
	} else if (rate_khz < (12000 * HZ_PER_KHZ)) {
		lo_x4_filter = 0x00;
		lo_x3_filter = 0x1C;
		lo_x1_filter = 0x1F;
		lo_trap_filter = 0x318;
		lo_doubler_band = 0x1D;
	} else if (rate_khz < (14000 * HZ_PER_KHZ)) {
		lo_x4_filter = 0x00;
		lo_x3_filter = 0x07;
		lo_x1_filter = 0x0A;
		lo_trap_filter = 0x3AA;
		lo_doubler_band = 0x04;
	} else if (rate_khz < (14500 * HZ_PER_KHZ)) {
		lo_x4_filter = 0x05;
		lo_x3_filter = 0x07;
		lo_x1_filter = 0x0A;
		lo_trap_filter = 0x3AA;
		lo_doubler_band = 0x08;
	} else if (rate_khz < (15000 * HZ_PER_KHZ)) {
		lo_x4_filter = 0x05;
		lo_x3_filter = 0x07;
		lo_x1_filter = 0x0A;
		lo_trap_filter = 0x1A3;
		lo_doubler_band = 0x08;
	} else if (rate_khz < (17000 * HZ_PER_KHZ)) {
		lo_x4_filter = 0x05;
		lo_x3_filter = 0x05;
		lo_x1_filter = 0x05;
		lo_trap_filter = 0x1A3;
		lo_doubler_band = 0x08;
	} else if (rate_khz < (18000 * HZ_PER_KHZ)) {
		lo_x4_filter = 0x05;
		lo_x3_filter = 0x04;
		lo_x1_filter = 0x01;
		lo_trap_filter = 0x1A3;
		lo_doubler_band = 0x08;
	} else if (rate_khz < (18500 * HZ_PER_KHZ)) {
		lo_x4_filter = 0x07;
		lo_x3_filter = 0x04;
		lo_x1_filter = 0x01;
		lo_trap_filter = 0x1A3;
		lo_doubler_band = 0x80;
	} else if (rate_khz < (19000 * HZ_PER_KHZ)) {
		lo_x4_filter = 0x07;
		lo_x3_filter = 0x04;
		lo_x1_filter = 0x01;
		lo_trap_filter = 0x1A7;
		lo_doubler_band = 0x80;
	} else if (rate_khz < (21000 * HZ_PER_KHZ)) {
		lo_x4_filter = 0x07;
		lo_x3_filter = 0x03;
		lo_x1_filter = 0x01;
		lo_trap_filter = 0x1A7;
		lo_doubler_band = 0x80;
	} else if (rate_khz < (24000 * HZ_PER_KHZ)) {
		lo_x4_filter = 0x07;
		lo_x3_filter = 0x02;
		lo_x1_filter = 0x00;
		lo_trap_filter = 0x1E7;
		lo_doubler_band = 0x80;
	} else {
		lo_x4_filter = 0x07;
		lo_x3_filter = 0x00;
		lo_x1_filter = 0x00;
		lo_trap_filter = 0x1E7;
		lo_doubler_band = 0xC0;
	}

	lo_trap_filter_7_0 = lo_trap_filter & 0xff;
	lo_trap_filter_9_8 = (lo_trap_filter >> 8) & 0x03;

	dev_dbg(dev, "%s: LO rate %llu kHz, x4=0x%02X x3=0x%02X x1=0x%02X trap=0x%03X dbl=0x%02X\n",
		__func__, rate_khz, lo_x4_filter, lo_x3_filter, lo_x1_filter,
		lo_trap_filter, lo_doubler_band);

	ret = admv1355_spi_write(priv, ADMV1355_REG_LO_X4_FILTER, lo_x4_filter);
	if (ret)
		return ret;

	ret = admv1355_spi_write(priv, ADMV1355_REG_LO_X3_FILTER, lo_x3_filter);
	if (ret)
		return ret;

	ret = admv1355_spi_write(priv, ADMV1355_REG_LO_X1_FILTER, lo_x1_filter);
	if (ret)
		return ret;

	ret = admv1355_spi_write(priv, ADMV1355_REG_LO_TRAP_7_0, lo_trap_filter_7_0);
	if (ret)
		return ret;

	ret = regmap_update_bits(priv->regmap, ADMV1355_REG_LO_TRAP_9_8,
				 0x03, lo_trap_filter_9_8);
	if (ret)
		return ret;

	return admv1355_spi_write(priv, ADMV1355_REG_LO_DBL_BAND, lo_doubler_band);
}

static int admv1355_freq_change(struct notifier_block *nb, unsigned long action,
				void *data)
{
	struct admv1355_priv *priv = container_of(nb, struct admv1355_priv, nb);
	u64 rate;
	int ret;

	if (action != POST_RATE_CHANGE)
		return NOTIFY_OK;

	guard(mutex)(&priv->lock);

	rate = clk_get_rate_scaled(priv->lo_input, &priv->clkscale);
	ret = admv1355_set_filters(priv, rate);
	if (ret)
		dev_err(&priv->spi->dev, "%s: failed for LO rate %llu (%d)\n",
			__func__, rate, ret);

	return notifier_from_errno(ret);
}

static int admv1355_spi_verify(struct admv1355_priv *priv)
{
	struct device *dev = &priv->spi->dev;
	unsigned int scratch_rd;
	int ret;

	ret = admv1355_spi_write(priv, ADMV1355_REG_SCRATCH_PAD,
				 ADMV1355_SCRATCH_TEST_VAL);
	if (ret)
		return ret;

	ret = admv1355_spi_read(priv, ADMV1355_REG_SCRATCH_PAD, &scratch_rd);
	if (ret)
		return ret;

	if (scratch_rd != ADMV1355_SCRATCH_TEST_VAL) {
		dev_err(dev, "%s: mismatch: wrote 0x%02X, read 0x%02X\n",
			__func__, ADMV1355_SCRATCH_TEST_VAL, scratch_rd);
		return -EIO;
	}

	dev_info(dev, "SPI verify OK (scratchpad 0x%02X)\n",
		 ADMV1355_SCRATCH_TEST_VAL);

	return 0;
}

static int admv1355_nvm_load(struct admv1355_priv *priv)
{
	int ret;

	ret = admv1355_spi_write(priv, ADMV1355_REG_NVM_CTRL, 0x08);
	if (ret)
		return ret;

	usleep_range(1000, 2000);

	ret = admv1355_spi_write(priv, ADMV1355_REG_NVM_LOAD, 0x40);
	if (ret)
		return ret;

	usleep_range(1000, 2000);

	ret = admv1355_spi_write(priv, ADMV1355_REG_NVM_LOAD, 0x54);
	if (ret)
		return ret;

	usleep_range(1000, 2000);

	return admv1355_spi_write(priv, ADMV1355_REG_NVM_CTRL, 0x09);
}

static int admv1355_nvm_init(struct admv1355_priv *priv)
{
	struct device *dev = &priv->spi->dev;
	static const u8 shadow_regs[] = { 0x33, 0x34, 0x37, 0x38 };
	unsigned int addr_rd, data_rd;
	int ret, i;

	ret = admv1355_nvm_load(priv);
	if (ret)
		return ret;

	for (i = 0; i < ARRAY_SIZE(shadow_regs); i++) {
		ret = admv1355_spi_write(priv, ADMV1355_REG_NVM_ADDR,
					 shadow_regs[i]);
		if (ret)
			return ret;

		ret = admv1355_spi_read(priv, ADMV1355_REG_NVM_ADDR, &addr_rd);
		if (ret)
			return ret;

		if (addr_rd != shadow_regs[i]) {
			dev_err(dev, "%s: NVM addr 0x%02X readback mismatch: 0x%02X\n",
				__func__, shadow_regs[i], addr_rd);
			return -EIO;
		}

		ret = admv1355_spi_read(priv, ADMV1355_REG_NVM_DATA, &data_rd);
		if (ret)
			return ret;

		if (data_rd == 0) {
			dev_err(dev, "%s: NVM shadow 0x%02X still zero after load\n",
				__func__, shadow_regs[i]);
			return -EIO;
		}
	}

	dev_info(dev, "NVM loaded and verified\n");

	return 0;
}

static int admv1355_setup(struct admv1355_priv *priv)
{
	struct device *dev = &priv->spi->dev;
	unsigned int product_id;
	int ret;

	ret = admv1355_spi_write(priv, ADMV1355_REG_SDO_CTRL, 0x18);
	if (ret)
		return ret;

	ret = admv1355_spi_read(priv, ADMV1355_REG_PRODUCT_ID_LSB, &product_id);
	if (ret)
		return ret;

	dev_info(dev, "PRODUCT_ID: 0x%02X\n", product_id);

	ret = admv1355_spi_verify(priv);
	if (ret)
		return ret;

	ret = admv1355_nvm_init(priv);
	if (ret)
		return ret;

	ret = admv1355_spi_write(priv, ADMV1355_REG_RF_HS_PD, 0x02);
	if (ret)
		return ret;

	ret = admv1355_spi_write(priv, ADMV1355_REG_FILTER_LUT_EN, 0x00);
	if (ret)
		return ret;

	ret = admv1355_spi_write(priv, ADMV1355_REG_FILTER_LOAD_EN, 0x00);
	if (ret)
		return ret;

	ret = admv1355_spi_write(priv, ADMV1355_REG_GAIN_TBL_BYP, 0x01);
	if (ret)
		return ret;

	ret = admv1355_spi_write(priv, ADMV1355_REG_DSA_BYPASS, 0x00);
	if (ret)
		return ret;

	ret = admv1355_spi_write(priv, ADMV1355_REG_RF_LPF, 0x10);
	if (ret)
		return ret;

	ret = admv1355_spi_write(priv, ADMV1355_REG_RF_HPF, 0xBF);
	if (ret)
		return ret;

	return admv1355_spi_write(priv, ADMV1355_REG_GPO_G_DIRECT, 0x08);
}

static const struct regmap_config admv1355_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.read_flag_mask = BIT(7),
};

static int admv1355_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct iio_dev *indio_dev;
	struct admv1355_priv *priv;
	u64 rate;
	int ret;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*priv));
	if (!indio_dev)
		return -ENOMEM;

	priv = iio_priv(indio_dev);
	priv->spi = spi;

	dev_info(dev, "probe start\n");

	priv->reset_gpio = devm_gpiod_get_optional(dev, "reset",
						    GPIOD_OUT_HIGH);
	if (IS_ERR(priv->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(priv->reset_gpio),
				     "failed to get reset gpio\n");

	priv->cen_gpio = devm_gpiod_get_optional(dev, "chip-enable",
						  GPIOD_OUT_LOW);
	if (IS_ERR(priv->cen_gpio))
		return dev_err_probe(dev, PTR_ERR(priv->cen_gpio),
				     "failed to get chip-enable gpio\n");

	priv->lvl_en_gpio = devm_gpiod_get_optional(dev, "lvl-en",
						     GPIOD_OUT_HIGH);
	if (IS_ERR(priv->lvl_en_gpio))
		return dev_err_probe(dev, PTR_ERR(priv->lvl_en_gpio),
				     "failed to get lvl-en gpio\n");

	priv->pwr_en_gpio = devm_gpiod_get_optional(dev, "pwr-en",
						     GPIOD_OUT_HIGH);
	if (IS_ERR(priv->pwr_en_gpio))
		return dev_err_probe(dev, PTR_ERR(priv->pwr_en_gpio),
				     "failed to get pwr-en gpio\n");
	if (priv->pwr_en_gpio)
		msleep(100);

	if (priv->reset_gpio) {
		gpiod_set_value_cansleep(priv->reset_gpio, 0);
		usleep_range(1000, 2000);
		dev_info(dev, "reset deasserted\n");
	}

	priv->regmap = devm_regmap_init_spi(spi, &admv1355_regmap_config);
	if (IS_ERR(priv->regmap))
		return dev_err_probe(dev, PTR_ERR(priv->regmap),
				     "failed to init regmap\n");

	priv->lo_input = devm_clk_get_enabled(dev, "lo-input");
	if (IS_ERR(priv->lo_input))
		return dev_err_probe(dev, PTR_ERR(priv->lo_input),
				     "failed to get the LO input clock\n");

	ret = of_clk_get_scale(spi->dev.of_node, NULL, &priv->clkscale);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get clock scale\n");

	priv->nb.notifier_call = admv1355_freq_change;
	ret = devm_clk_notifier_register(dev, priv->lo_input, &priv->nb);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to register clock notifier\n");

	ret = devm_mutex_init(dev, &priv->lock);
	if (ret)
		return ret;

	ret = admv1355_setup(priv);
	if (ret)
		return ret;

	rate = clk_get_rate_scaled(priv->lo_input, &priv->clkscale);
	ret = admv1355_set_filters(priv, rate);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to set filters for LO rate %llu\n",
				     rate);

	if (priv->cen_gpio) {
		gpiod_set_value_cansleep(priv->cen_gpio, 1);
		dev_info(dev, "chip-enable asserted\n");
	}

	indio_dev->name = spi->dev.of_node->name;
	indio_dev->info = &admv1355_info;
	indio_dev->channels = admv1355_channels;
	indio_dev->num_channels = ARRAY_SIZE(admv1355_channels);

	dev_info(dev, "successfully initialized, LO rate %llu Hz\n", rate);

	return devm_iio_device_register(dev, indio_dev);
}

static const struct spi_device_id admv1355_id[] = {
	{ "admv1355", 0 },
	{}
};
MODULE_DEVICE_TABLE(spi, admv1355_id);

static const struct of_device_id admv1355_of_match[] = {
	{ .compatible = "adi,admv1355" },
	{}
};
MODULE_DEVICE_TABLE(of, admv1355_of_match);

static struct spi_driver admv1355_driver = {
	.driver = {
		.name = "admv1355",
		.of_match_table = admv1355_of_match,
	},
	.probe = admv1355_probe,
	.id_table = admv1355_id,
};
module_spi_driver(admv1355_driver);

MODULE_AUTHOR("Dragos Bogdan <dragos.bogdan@analog.com>");
MODULE_DESCRIPTION("ADMV1355 Microwave Upconverter Driver");
MODULE_LICENSE("GPL v2");
