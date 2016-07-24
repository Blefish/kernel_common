/* arch/arm/mach-msm/proc_comm.c
 *
 * Copyright (C) 2007-2008 Google, Inc.
 * Copyright (c) 2009-2012, The Linux Foundation. All rights reserved.
 * Copyright (c) 2016-2017 Rudolf Tammekivi
 * Author: Brian Swetland <swetland@google.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/delay.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/mfd/syscon.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <soc/qcom/proc_comm.h>
#ifdef CONFIG_MSM_SMD
#include <soc/qcom/smsm.h>
#endif

struct proccomm_data {
	struct regmap *ipc_regmap;
	struct regmap *smem_regmap;
	int ipc_offset;
	int ipc_bit;
	void __iomem *shared_ram;

	spinlock_t lock;
	bool pcom_disabled;
};

static struct proccomm_data *pcom_data;

static inline void notify_other_proc_comm(struct proccomm_data *data)
{
	regmap_write(data->ipc_regmap, data->ipc_offset, BIT(data->ipc_bit));
}

#define APP_COMMAND	0x00
#define APP_STATUS	0x04
#define APP_DATA1	0x08
#define APP_DATA2	0x0C

#define MDM_COMMAND	0x10
#define MDM_STATUS	0x14
#define MDM_DATA1	0x18
#define MDM_DATA2	0x1C

/* Poll for a state change, checking for possible
 * modem crashes along the way (so we don't wait
 * forever while the ARM9 is blowing up.
 *
 * Return an error in the event of a modem crash and
 * restart so the msm_proc_comm() routine can restart
 * the operation from the beginning.
 */
static int proc_comm_wait_for(struct proccomm_data *data,
			      unsigned int reg, unsigned int val)
{
	unsigned int value;
	int ret;

	while (1) {
		/* Barrier here prevents excessive spinning */
		mb();

		ret = regmap_read(data->smem_regmap, reg, &value);
		if (!ret && val == value)
			return 0;

#ifdef CONFIG_MSM_SMD
		if (smsm_check_for_modem_crash())
			return -EAGAIN;
#endif

		udelay(5);
	}
}

void msm_proc_comm_reset_modem_now(void)
{
	struct proccomm_data *data = pcom_data;
	unsigned long flags;

	if (!data)
		return;

	spin_lock_irqsave(&data->lock, flags);

again:
	if (proc_comm_wait_for(data, MDM_STATUS, PCOM_READY))
		goto again;

	regmap_write(data->smem_regmap, APP_COMMAND, PCOM_RESET_MODEM);
	regmap_write(data->smem_regmap, APP_DATA1, 0);
	regmap_write(data->smem_regmap, APP_DATA2, 0);

	spin_unlock_irqrestore(&data->lock, flags);

	/* Make sure the writes complete before notifying the other side */
	wmb();
	notify_other_proc_comm(data);

	return;
}
EXPORT_SYMBOL(msm_proc_comm_reset_modem_now);

int msm_proc_comm(unsigned cmd, unsigned *data1, unsigned *data2)
{
	struct proccomm_data *data = pcom_data;
	unsigned long flags;
	unsigned int val;
	int ret;

	if (!data)
		return -EPROBE_DEFER;

	if (data->pcom_disabled)
		return -EIO;

	spin_lock_irqsave(&data->lock, flags);

again:
	if (proc_comm_wait_for(data, MDM_STATUS, PCOM_READY))
		goto again;

	regmap_write(data->smem_regmap, APP_COMMAND, cmd);
	regmap_write(data->smem_regmap, APP_DATA1, data1 ? *data1 : 0);
	regmap_write(data->smem_regmap, APP_DATA2, data2 ? *data2 : 0);

	/* Make sure the writes complete before notifying the other side */
	wmb();
	notify_other_proc_comm(data);

	if (proc_comm_wait_for(data, APP_COMMAND, PCOM_CMD_DONE))
		goto again;

	ret = regmap_read(data->smem_regmap, APP_STATUS, &val);
	if (!ret && val == PCOM_CMD_SUCCESS) {
		if (data1)
			regmap_read(data->smem_regmap, APP_DATA1, data1);
		if (data2)
			regmap_read(data->smem_regmap, APP_DATA2, data2);
	}

	regmap_write(data->smem_regmap, APP_COMMAND, PCOM_CMD_IDLE);

	switch (cmd) {
	case PCOM_RESET_CHIP:
	case PCOM_RESET_CHIP_IMM:
	case PCOM_RESET_APPS:
		data->pcom_disabled = true;
		pr_err("%s: proc comm disabled\n", __func__);
		break;
	}

	/* Make sure the writes complete before returning */
	wmb();
	spin_unlock_irqrestore(&data->lock, flags);
	return ret;
}
EXPORT_SYMBOL(msm_proc_comm);

static int proccomm_probe(struct platform_device *pdev)
{
	struct device_node *syscon_np;
	const char *key;
	struct device *dev = &pdev->dev;
	struct proccomm_data *data;
	int ret;

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data) {
		dev_err(dev, "failed to allocate memory\n");
		return -ENOMEM;
	}

	key = "qcom,ipc";
	syscon_np = of_parse_phandle(pdev->dev.of_node, key, 0);
	if (!syscon_np) {
		dev_err(dev, "no qcom,ipc node\n");
		return -ENODEV;
	}
	data->ipc_regmap = syscon_node_to_regmap(syscon_np);
	if (IS_ERR(data->ipc_regmap)) {
		ret = PTR_ERR(data->ipc_regmap);
		dev_err(dev, "failed to get ipc regmap\n");
		return ret;
	}

	ret = of_property_read_u32_index(dev->of_node, key, 1,
					 &data->ipc_offset);
	if (ret < 0) {
		dev_err(dev, "no offset in %s ret=%d\n", key, ret);
		return ret;
	}

	ret = of_property_read_u32_index(dev->of_node, key, 2, &data->ipc_bit);
	if (ret < 0) {
		dev_err(dev, "no bit in %s ret=%d\n", key, ret);
		return ret;
	}

	key = "qcom,smem";
	syscon_np = of_parse_phandle(pdev->dev.of_node, key, 0);
	if (!syscon_np) {
		dev_err(dev, "no qcom,smem node\n");
		return -ENODEV;
	}
	data->smem_regmap = syscon_node_to_regmap(syscon_np);
	if (IS_ERR(data->ipc_regmap)) {
		ret = PTR_ERR(data->ipc_regmap);
		dev_err(dev, "failed to get smem regmap\n");
		return ret;
	}

	spin_lock_init(&data->lock);

	platform_set_drvdata(pdev, data);
	pcom_data = data;

	ret = of_platform_populate(dev->of_node, NULL, NULL, dev);
	if (ret)
		dev_err(dev, "some devices failed to populate ret=%d\n", ret);

	return 0;
}

static int proccomm_remove(struct platform_device *pdev)
{
	of_platform_depopulate(&pdev->dev);
	pcom_data = NULL;
	return 0;
}

static const struct of_device_id proccomm_of_match[] = {
	{ .compatible = "qcom,proccomm", },
	{ },
};
MODULE_DEVICE_TABLE(of, proc_comm_of_match);

static struct platform_driver proccomm_driver = {
	.probe = proccomm_probe,
	.remove = proccomm_remove,
	.driver = {
		.name = "proccomm",
		.of_match_table = proccomm_of_match,
	},
};

static int __init proccomm_init(void)
{
	return platform_driver_register(&proccomm_driver);
}
core_initcall(proccomm_init);

static void __exit proccomm_exit(void)
{
	platform_driver_unregister(&proccomm_driver);
}
module_exit(proccomm_exit);

MODULE_AUTHOR("Brian Swetland <swetland@google.com>");
MODULE_DESCRIPTION("Qualcomm ProcComm driver");
MODULE_LICENSE("GPL v2");
