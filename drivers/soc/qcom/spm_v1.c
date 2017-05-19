/* Copyright (c) 2010-2011, The Linux Foundation. All rights reserved.
 * Copyright (c) 2014,2015, Linaro Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/cpuidle.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/of_regulator.h>

#include <asm/cpuidle.h>
#include <asm/suspend.h>

#include <soc/qcom/spm_v1.h>

#define MSM_SPM_PMIC_STATE_IDLE 0

#define MV_TO_UV(mv)		((mv)*1000)
#define UV_TO_MV(uv)		((uv)/1000)

#define VREF_SEL	1 /* 0: 0.625V (50mV step), 1: 0.3125V (25mV step). */
#define V_STEP		(25 * (2 - VREF_SEL)) /* Minimum voltage step size. */
#define VREG_DATA	(VREG_CONFIG | (VREF_SEL << 5))
#define VREG_CONFIG	(BIT(7) | BIT(6)) /* Enable VREG, pull-down if disabled. */
/* Cause a compile error if the voltage is not a multiple of the step size. */
#define MV(mv)		((mv) / (!((mv) % V_STEP)))
/* mv = (750mV + (raw * 25mV)) * (2 - VREF_SEL) */
#define VDD_RAW(mv)	(((MV(mv) / V_STEP) - 30) | VREG_DATA)

#define VDD_RAW_TO_MV(raw)	((((raw & ~VREG_DATA)) + 30) * V_STEP)

enum spm_reg {
	MSM_SPM_REG_SAW_AVS_CTL,
	MSM_SPM_REG_SAW_CFG,
	MSM_SPM_REG_SAW_SPM_CTL,
	MSM_SPM_REG_SAW_SPM_SLP_TMR_DLY,
	MSM_SPM_REG_SAW_SPM_WAKE_TMR_DLY,
	MSM_SPM_REG_SAW_SLP_CLK_EN,
	MSM_SPM_REG_SAW_SLP_HSFS_PRECLMP_EN,
	MSM_SPM_REG_SAW_SLP_HSFS_POSTCLMP_EN,
	MSM_SPM_REG_SAW_SLP_CLMP_EN,
	MSM_SPM_REG_SAW_SLP_RST_EN,
	MSM_SPM_REG_SAW_SPM_MPM_CFG,
	MSM_SPM_REG_NR_INITIALIZE,

	MSM_SPM_REG_SAW_VCTL = MSM_SPM_REG_NR_INITIALIZE,
	MSM_SPM_REG_SAW_STS,
	MSM_SPM_REG_SAW_SPM_PMIC_CTL,
	MSM_SPM_REG_NR
};

struct msm_spm_platform_data {
	void __iomem *reg_base_addr;
	uint32_t reg_data[MSM_SPM_REG_NR];

	uint8_t awake_vlevel;
	uint8_t retention_vlevel;
	uint8_t collapse_vlevel;
	uint8_t retention_mid_vlevel;
	uint8_t collapse_mid_vlevel;

	uint32_t vctl_timeout_us;
};

static const u8 msm_spm_reg_offsets[MSM_SPM_REG_NR] = {
	[MSM_SPM_REG_SAW_AVS_CTL] = 0x04,

	[MSM_SPM_REG_SAW_VCTL] = 0x08,
	[MSM_SPM_REG_SAW_STS] = 0x0C,
	[MSM_SPM_REG_SAW_CFG] = 0x10,

	[MSM_SPM_REG_SAW_SPM_CTL] = 0x14,
	[MSM_SPM_REG_SAW_SPM_SLP_TMR_DLY] = 0x18,
	[MSM_SPM_REG_SAW_SPM_WAKE_TMR_DLY] = 0x1C,

	[MSM_SPM_REG_SAW_SPM_PMIC_CTL] = 0x20,
	[MSM_SPM_REG_SAW_SLP_CLK_EN] = 0x24,
	[MSM_SPM_REG_SAW_SLP_HSFS_PRECLMP_EN] = 0x28,
	[MSM_SPM_REG_SAW_SLP_HSFS_POSTCLMP_EN] = 0x2C,

	[MSM_SPM_REG_SAW_SLP_CLMP_EN] = 0x30,
	[MSM_SPM_REG_SAW_SLP_RST_EN] = 0x34,
	[MSM_SPM_REG_SAW_SPM_MPM_CFG] = 0x38,
};

static const struct msm_spm_platform_data spm_reg_7x30_cpu = {
	.reg_data[MSM_SPM_REG_SAW_CFG] = 0x05,
	.reg_data[MSM_SPM_REG_SAW_SPM_CTL] = 0x18,
	.reg_data[MSM_SPM_REG_SAW_SPM_SLP_TMR_DLY] = 0x00006666,
	.reg_data[MSM_SPM_REG_SAW_SPM_WAKE_TMR_DLY] = 0xFF000666,

	.reg_data[MSM_SPM_REG_SAW_SLP_CLK_EN] = 0x01,
	.reg_data[MSM_SPM_REG_SAW_SLP_HSFS_PRECLMP_EN] = 0x03,
	.reg_data[MSM_SPM_REG_SAW_SLP_HSFS_POSTCLMP_EN] = 0x00,

	.reg_data[MSM_SPM_REG_SAW_SLP_CLMP_EN] = 0x01,
	.reg_data[MSM_SPM_REG_SAW_SLP_RST_EN] = 0x00,
	.reg_data[MSM_SPM_REG_SAW_SPM_MPM_CFG] = 0x00,

	.awake_vlevel = 0xF2,
	.retention_vlevel = 0xE0,
	.collapse_vlevel = 0x72,
	.retention_mid_vlevel = 0xE0,
	.collapse_mid_vlevel = 0xE0,

	.vctl_timeout_us = 50,
};

struct spm_driver_data {
	struct regulator_desc rdesc;
	void __iomem *reg_base;
	struct msm_spm_platform_data *cpu_data;

	unsigned int low_power_mode;
	bool notify_rpm;
	bool dirty;

	uint32_t *rst_vector;
	uint32_t saved_vector[2];
};

static DEFINE_PER_CPU(struct spm_driver_data *, cpu_spm_drv);

typedef int (*idle_fn)(int);
static DEFINE_PER_CPU(idle_fn*, qcom_idle_ops);

/******************************************************************************
 * Internal helper functions
 *****************************************************************************/

static inline void msm_spm_set_vctl(struct spm_driver_data *drv,
				    uint32_t vlevel)
{
	drv->cpu_data->reg_data[MSM_SPM_REG_SAW_VCTL] &= ~0xFF;
	drv->cpu_data->reg_data[MSM_SPM_REG_SAW_VCTL] |= vlevel;
}

static inline void msm_spm_set_spm_ctl(struct spm_driver_data *drv,
				       uint32_t rpm_bypass,
				       uint32_t mode_encoding)
{
	drv->cpu_data->reg_data[MSM_SPM_REG_SAW_SPM_CTL] &= ~0x0F;
	drv->cpu_data->reg_data[MSM_SPM_REG_SAW_SPM_CTL] |= rpm_bypass << 3;
	drv->cpu_data->reg_data[MSM_SPM_REG_SAW_SPM_CTL] |= mode_encoding;
}

static inline void msm_spm_set_pmic_ctl(struct spm_driver_data *drv,
					uint32_t awake_vlevel,
					uint32_t mid_vlevel,
					uint32_t sleep_vlevel)
{
	drv->cpu_data->reg_data[MSM_SPM_REG_SAW_SPM_PMIC_CTL] =
		(mid_vlevel << 16) | (awake_vlevel << 8) | (sleep_vlevel);
}

static inline void msm_spm_set_slp_rst_en(struct spm_driver_data *drv,
					  uint32_t slp_rst_en)
{
	drv->cpu_data->reg_data[MSM_SPM_REG_SAW_SLP_RST_EN] = slp_rst_en;
}

static inline void msm_spm_flush_shadow(struct spm_driver_data *drv,
					unsigned int reg_index)
{
	__raw_writel(drv->cpu_data->reg_data[reg_index],
		     drv->reg_base + msm_spm_reg_offsets[reg_index]);
}

static inline void msm_spm_load_shadow(struct spm_driver_data *drv,
				       unsigned int reg_index)
{
	drv->cpu_data->reg_data[reg_index] = __raw_readl(drv->reg_base +
		msm_spm_reg_offsets[reg_index]);
}

static inline uint32_t msm_spm_get_sts_pmic_state(struct spm_driver_data *drv)
{
	return (drv->cpu_data->reg_data[MSM_SPM_REG_SAW_STS] >> 20) & 0x03;
}

static inline uint32_t msm_spm_get_sts_curr_pmic_data(
	struct spm_driver_data *drv)
{
	return (drv->cpu_data->reg_data[MSM_SPM_REG_SAW_STS] >> 10) & 0xFF;
}

/******************************************************************************
 * Public functions
 *****************************************************************************/
int msm_spm_set_low_power_mode(unsigned int mode, bool notify_rpm)
{
	int i;
	struct spm_driver_data *drv = this_cpu_ptr(cpu_spm_drv);
	uint32_t rpm_bypass = notify_rpm ? 0x00 : 0x01;

	if (drv->low_power_mode == mode && drv->notify_rpm == notify_rpm &&
		!drv->dirty)
		return 0;

	switch (mode) {
	case MSM_SPM_MODE_CLOCK_GATING:
		msm_spm_set_spm_ctl(drv, rpm_bypass, 0x00);
		msm_spm_set_slp_rst_en(drv, 0x00);
		break;

	case MSM_SPM_MODE_POWER_RETENTION:
		msm_spm_set_spm_ctl(drv, rpm_bypass, 0x02);
		msm_spm_set_pmic_ctl(drv, drv->cpu_data->awake_vlevel,
				     drv->cpu_data->retention_mid_vlevel,
				     drv->cpu_data->retention_vlevel);
		msm_spm_set_slp_rst_en(drv, 0x00);
		break;

	case MSM_SPM_MODE_POWER_COLLAPSE:
		msm_spm_set_spm_ctl(drv, rpm_bypass, 0x02);
		msm_spm_set_pmic_ctl(drv, drv->cpu_data->awake_vlevel,
				     drv->cpu_data->collapse_mid_vlevel,
				     drv->cpu_data->collapse_vlevel);
		msm_spm_set_slp_rst_en(drv, 0x01);
		break;

	default:
		BUG();
	}

	msm_spm_flush_shadow(drv, MSM_SPM_REG_SAW_SPM_CTL);
	msm_spm_flush_shadow(drv, MSM_SPM_REG_SAW_SPM_PMIC_CTL);
	msm_spm_flush_shadow(drv, MSM_SPM_REG_SAW_SLP_RST_EN);
	/* Ensure that the registers are written before returning */
	mb();

	drv->low_power_mode = mode;
	drv->notify_rpm = notify_rpm;
	drv->dirty = false;

	for (i = 0; i < MSM_SPM_REG_NR; i++)
		pr_debug("%s: reg %02x = 0x%08x\n", __func__,
			 msm_spm_reg_offsets[i], drv->cpu_data->reg_data[i]);

	return 0;
}

int msm_spm_set_vdd(struct spm_driver_data *drv, unsigned int vlevel)
{
	uint32_t timeout_us;

	pr_debug("%s: requesting cpu vlevel 0x%x\n", __func__, vlevel);

	msm_spm_set_vctl(drv, vlevel);
	msm_spm_flush_shadow(drv, MSM_SPM_REG_SAW_VCTL);

	/* Wait for PMIC state to return to idle or until timeout */
	timeout_us = drv->cpu_data->vctl_timeout_us;
	msm_spm_load_shadow(drv, MSM_SPM_REG_SAW_STS);
	while (msm_spm_get_sts_pmic_state(drv) != MSM_SPM_PMIC_STATE_IDLE) {
		if (!timeout_us)
			goto set_vdd_bail;

		if (timeout_us > 10) {
			udelay(10);
			timeout_us -= 10;
		} else {
			udelay(timeout_us);
			timeout_us = 0;
		}
		msm_spm_load_shadow(drv, MSM_SPM_REG_SAW_STS);
	}

	if (msm_spm_get_sts_curr_pmic_data(drv) != vlevel)
		goto set_vdd_bail;

	drv->cpu_data->awake_vlevel = vlevel;
	drv->dirty = true;

	pr_debug("%s: cpu done, remaining timeout %uus\n",
		 __func__, timeout_us);

	/* Wait for voltage to stabilize. */
	udelay(62);

	return 0;

set_vdd_bail:
	pr_err("%s: cpu failed, remaining timeout %uus, vlevel 0x%x\n",
	       __func__, timeout_us, msm_spm_get_sts_curr_pmic_data(drv));

	return -EIO;
}

void msm_spm_reinit(void)
{
	struct spm_driver_data *drv = this_cpu_ptr(cpu_spm_drv);
	int i;

	for (i = 0; i < MSM_SPM_REG_NR_INITIALIZE; i++)
		msm_spm_flush_shadow(drv, i);

	/* Ensure that the registers are written before returning */
	mb();
}

static int qcom_pm_collapse(unsigned long int unused)
{
	/*
	 * WFI triggers CPU suspend when SPM is configured for SPC.
	 */
	cpu_do_idle();

	/*
	 * Returns here only if there was a pending interrupt and we did not
	 * power down as a result.
	 */
	return -1;
}

static void msm_pm_config_rst_vector_before_pc(struct spm_driver_data *drv,
		unsigned long entry)
{
	drv->saved_vector[0] = drv->rst_vector[0];
	drv->saved_vector[1] = drv->rst_vector[1];
	drv->rst_vector[0] = 0xE51FF004;	/* ldr pc, [rc, #-4] */
	drv->rst_vector[1] = entry;		/* pc jump phy address */
}

static void msm_pm_config_rst_vector_after_pc(struct spm_driver_data *drv)
{
	drv->rst_vector[0] = drv->saved_vector[0];
	drv->rst_vector[1] = drv->saved_vector[1];
}

static int qcom_cpu_spc(int cpu)
{
	int ret;
	struct spm_driver_data *drv = per_cpu(cpu_spm_drv, cpu);

	msm_spm_set_low_power_mode(MSM_SPM_MODE_POWER_COLLAPSE, false);
	msm_pm_config_rst_vector_before_pc(drv, virt_to_phys(cpu_resume_arm));

	ret = cpu_suspend(0, qcom_pm_collapse);

	/*
	 * ARM common code executes WFI without calling into our driver and
	 * if the SPM mode is not reset, then we may accidentaly power down the
	 * cpu when we intended only to gate the cpu clock.
	 * Ensure the state is set to standby before returning.
	 */

	msm_pm_config_rst_vector_after_pc(drv);
	msm_spm_set_low_power_mode(MSM_SPM_MODE_CLOCK_GATING, false);

	return ret;
}

static int qcom_idle_enter(int cpu, unsigned long index)
{
	return per_cpu(qcom_idle_ops, cpu)[index](cpu);
}

static const struct of_device_id qcom_idle_state_match[] __initconst = {
	{ .compatible = "qcom,idle-state-spc", .data = qcom_cpu_spc },
	{ },
};

static int __init qcom_cpuidle_init(struct device_node *cpu_node, int cpu)
{
	const struct of_device_id *match_id;
	struct device_node *state_node;
	int i;
	int state_count = 1;
	idle_fn idle_fns[CPUIDLE_STATE_MAX];
	idle_fn *fns;

	for (i = 0; ; i++) {
		state_node = of_parse_phandle(cpu_node, "cpu-idle-states", i);
		if (!state_node)
			break;

		if (!of_device_is_available(state_node))
			continue;

		if (i == CPUIDLE_STATE_MAX) {
			pr_warn("%s: cpuidle states reached max possible\n",
					__func__);
			break;
		}

		match_id = of_match_node(qcom_idle_state_match, state_node);
		if (!match_id)
			return -ENODEV;

		idle_fns[state_count] = match_id->data;
		state_count++;
	}

	if (state_count == 1)
		goto check_spm;

	fns = devm_kcalloc(get_cpu_device(cpu), state_count, sizeof(*fns),
			GFP_KERNEL);
	if (!fns)
		return -ENOMEM;

	for (i = 1; i < state_count; i++)
		fns[i] = idle_fns[i];

	per_cpu(qcom_idle_ops, cpu) = fns;

	/*
	 * SPM probe for the cpu should have happened by now, if the
	 * SPM device does not exist, return -ENXIO to indicate that the
	 * cpu does not support idle states.
	 */
check_spm:
	return per_cpu(cpu_spm_drv, cpu) ? 0 : -ENXIO;
}

static struct cpuidle_ops qcom_cpuidle_ops __initdata = {
	.suspend = qcom_idle_enter,
	.init = qcom_cpuidle_init,
};

CPUIDLE_METHOD_OF_DECLARE(qcom_idle_v1, "qcom,scss", &qcom_cpuidle_ops);

static int spm_vreg_get_voltage(struct regulator_dev *rdev)
{
	struct spm_driver_data *drv = rdev_get_drvdata(rdev);
	unsigned int vlevel;

	msm_spm_load_shadow(drv, MSM_SPM_REG_SAW_STS);
	vlevel = msm_spm_get_sts_curr_pmic_data(drv);

	return MV_TO_UV(VDD_RAW_TO_MV(vlevel));
}

static int spm_vreg_set_voltage(struct regulator_dev *rdev,
				int min_uV, int max_uV, unsigned *sel)
{
	struct spm_driver_data *drv = rdev_get_drvdata(rdev);
	return msm_spm_set_vdd(drv, VDD_RAW(UV_TO_MV(min_uV)));
}

static struct regulator_ops spm_vreg_ops = {
	.list_voltage	= regulator_list_voltage_linear,
	.get_voltage	= spm_vreg_get_voltage,
	.set_voltage	= spm_vreg_set_voltage,
};

static struct spm_driver_data *spm_get_drv(struct platform_device *pdev,
		int *spm_cpu)
{
	struct spm_driver_data *drv = NULL;
	struct device_node *cpu_node, *saw_node;
	int cpu;
	bool found = false;

	for_each_possible_cpu(cpu) {
		cpu_node = of_cpu_device_node_get(cpu);
		if (!cpu_node)
			continue;
		saw_node = of_parse_phandle(cpu_node, "qcom,saw", 0);
		found = (saw_node == pdev->dev.of_node);
		of_node_put(saw_node);
		of_node_put(cpu_node);
		if (found)
			break;
	}

	if (found) {
		drv = devm_kzalloc(&pdev->dev, sizeof(*drv), GFP_KERNEL);
		if (drv)
			*spm_cpu = cpu;
	}

	return drv;
}

static const struct of_device_id spm_match_table[] = {
	{ .compatible = "qcom,msm7x30-saw-cpu", .data = &spm_reg_7x30_cpu },
	{ },
};

static int spm_dev_probe(struct platform_device *pdev)
{
	struct spm_driver_data *drv;
	struct resource *res;
	const struct of_device_id *match_id;
	int cpu;
	int i;

	uint32_t rst_phys = 0;

	struct regulator_init_data *initdata;
	struct regulator_config config = { };
	struct regulator_dev *rdev;

	drv = spm_get_drv(pdev, &cpu);
	if (!drv)
		return -EINVAL;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	drv->reg_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(drv->reg_base))
		return PTR_ERR(drv->reg_base);

	of_property_read_u32(pdev->dev.of_node, "reset-vector", &rst_phys);
	if (rst_phys)
		drv->rst_vector = phys_to_virt(rst_phys);

	match_id = of_match_node(spm_match_table, pdev->dev.of_node);
	if (!match_id)
		return -ENODEV;

	drv->cpu_data = devm_kzalloc(&pdev->dev, sizeof(*drv->cpu_data),
				     GFP_KERNEL);
	if (!drv->cpu_data)
		return -ENOMEM;

	memcpy(drv->cpu_data, match_id->data, sizeof(*drv->cpu_data));

	for (i = 0; i < MSM_SPM_REG_NR_INITIALIZE; i++)
		msm_spm_flush_shadow(drv, i);

	/* Ensure that the registers are written before returning */
	mb();

	drv->low_power_mode = MSM_SPM_MODE_CLOCK_GATING;
	drv->notify_rpm = false;
	drv->dirty = true;

	initdata = of_get_regulator_init_data(&pdev->dev,
					      pdev->dev.of_node,
					      &drv->rdesc);
	if (!initdata)
		return -ENODEV;

	drv->rdesc.min_uV = initdata->constraints.min_uV;
	drv->rdesc.uV_step = MV_TO_UV(V_STEP);
	drv->rdesc.n_voltages =
		((initdata->constraints.max_uV - initdata->constraints.min_uV) /
		drv->rdesc.uV_step) + 1;

	drv->rdesc.name = initdata->constraints.name;
	drv->rdesc.ops = &spm_vreg_ops;
	drv->rdesc.type = REGULATOR_VOLTAGE;
	drv->rdesc.owner = THIS_MODULE;

	config.dev = &pdev->dev;
	config.init_data = initdata;
	config.driver_data = drv;
	config.of_node = pdev->dev.of_node;

	rdev = devm_regulator_register(&pdev->dev, &drv->rdesc, &config);
	if (IS_ERR(rdev))
		return PTR_ERR(rdev);

	per_cpu(cpu_spm_drv, cpu) = drv;

	return 0;
}

static struct platform_driver spm_driver = {
	.probe = spm_dev_probe,
	.driver = {
		.name = "saw",
		.of_match_table = spm_match_table,
	},
};
module_platform_driver(spm_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("SAW power controller driver");
MODULE_ALIAS("platform:saw");
