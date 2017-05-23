/*
 * Copyright (C) 2007 Google, Inc.
 * Copyright (c) 2009, 2011 The Linux Foundation. All rights reserved.
 * Copyright (c) 2016 Rudolf Tammekivi
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

#include <asm/exception.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/irqchip.h>
#include <linux/irqdomain.h>
#include <linux/of_address.h>

#define VIC_REG(off) (vic_data.base + (off))
#define VIC_INT_TO_REG_ADDR(base, irq) (base + (irq / 32) * 4)
#define VIC_INT_TO_REG_INDEX(irq) ((irq >> 5) & 3)

#define VIC_INT_SELECT0		VIC_REG(0x0000) /* 1: FIQ, 0: IRQ */
#define VIC_INT_SELECT1		VIC_REG(0x0004) /* 1: FIQ, 0: IRQ */
#define VIC_INT_SELECT2		VIC_REG(0x0008) /* 1: FIQ, 0: IRQ */
#define VIC_INT_SELECT3		VIC_REG(0x000C) /* 1: FIQ, 0: IRQ */
#define VIC_INT_EN0		VIC_REG(0x0010)
#define VIC_INT_EN1		VIC_REG(0x0014)
#define VIC_INT_EN2		VIC_REG(0x0018)
#define VIC_INT_EN3		VIC_REG(0x001C)
#define VIC_INT_ENCLEAR0	VIC_REG(0x0020)
#define VIC_INT_ENCLEAR1	VIC_REG(0x0024)
#define VIC_INT_ENCLEAR2	VIC_REG(0x0028)
#define VIC_INT_ENCLEAR3	VIC_REG(0x002C)
#define VIC_INT_ENSET0		VIC_REG(0x0030)
#define VIC_INT_ENSET1		VIC_REG(0x0034)
#define VIC_INT_ENSET2		VIC_REG(0x0038)
#define VIC_INT_ENSET3		VIC_REG(0x003C)
#define VIC_INT_TYPE0		VIC_REG(0x0040) /* 1: EDGE, 0: LEVEL */
#define VIC_INT_TYPE1		VIC_REG(0x0044) /* 1: EDGE, 0: LEVEL */
#define VIC_INT_TYPE2		VIC_REG(0x0048) /* 1: EDGE, 0: LEVEL */
#define VIC_INT_TYPE3		VIC_REG(0x004C) /* 1: EDGE, 0: LEVEL */
#define VIC_INT_POLARITY0	VIC_REG(0x0050) /* 1: NEG, 0: POS */
#define VIC_INT_POLARITY1	VIC_REG(0x0054) /* 1: NEG, 0: POS */
#define VIC_INT_POLARITY2	VIC_REG(0x0058) /* 1: NEG, 0: POS */
#define VIC_INT_POLARITY3	VIC_REG(0x005C) /* 1: NEG, 0: POS */
#define VIC_NO_PEND_VAL		VIC_REG(0x0060)

#define VIC_NO_PEND_VAL_FIQ	VIC_REG(0x0064)
#define VIC_INT_MASTEREN	VIC_REG(0x0068) /* 1: IRQ, 2: FIQ */
#define VIC_CONFIG		VIC_REG(0x006C) /* 1: USE SC VIC */

#define VIC_IRQ_STATUS0		VIC_REG(0x0080)
#define VIC_IRQ_STATUS1		VIC_REG(0x0084)
#define VIC_IRQ_STATUS2		VIC_REG(0x0088)
#define VIC_IRQ_STATUS3		VIC_REG(0x008C)
#define VIC_FIQ_STATUS0		VIC_REG(0x0090)
#define VIC_FIQ_STATUS1		VIC_REG(0x0094)
#define VIC_FIQ_STATUS2		VIC_REG(0x0098)
#define VIC_FIQ_STATUS3		VIC_REG(0x009C)
#define VIC_RAW_STATUS0		VIC_REG(0x00A0)
#define VIC_RAW_STATUS1		VIC_REG(0x00A4)
#define VIC_RAW_STATUS2		VIC_REG(0x00A8)
#define VIC_RAW_STATUS3		VIC_REG(0x00AC)
#define VIC_INT_CLEAR0		VIC_REG(0x00B0)
#define VIC_INT_CLEAR1		VIC_REG(0x00B4)
#define VIC_INT_CLEAR2		VIC_REG(0x00B8)
#define VIC_INT_CLEAR3		VIC_REG(0x00BC)
#define VIC_SOFTINT0		VIC_REG(0x00C0)
#define VIC_SOFTINT1		VIC_REG(0x00C4)
#define VIC_SOFTINT2		VIC_REG(0x00C8)
#define VIC_SOFTINT3		VIC_REG(0x00CC)
#define VIC_IRQ_VEC_RD		VIC_REG(0x00D0) /* pending int # */
#define VIC_IRQ_VEC_PEND_RD	VIC_REG(0x00D4) /* pending vector addr */
#define VIC_IRQ_VEC_WR		VIC_REG(0x00D8)

#define VIC_FIQ_VEC_RD		VIC_REG(0x00DC)
#define VIC_FIQ_VEC_PEND_RD	VIC_REG(0x00E0)
#define VIC_FIQ_VEC_WR		VIC_REG(0x00E4)
#define VIC_IRQ_IN_SERVICE	VIC_REG(0x00E8)
#define VIC_IRQ_IN_STACK	VIC_REG(0x00EC)
#define VIC_FIQ_IN_SERVICE	VIC_REG(0x00F0)
#define VIC_FIQ_IN_STACK	VIC_REG(0x00F4)
#define VIC_TEST_BUS_SEL	VIC_REG(0x00F8)
#define VIC_IRQ_CTRL_CONFIG	VIC_REG(0x00FC)

#define VIC_VECTPRIORITY(n)	VIC_REG(0x0200+((n) * 4))
#define VIC_VECTADDR(n)		VIC_REG(0x0400+((n) * 4))

#define VIC_NUM_REGS		4

struct vic_device {
	void __iomem *base;
	struct irq_domain *domain;
};
static struct vic_device vic_data;

static inline void msm_irq_write_all_regs(void __iomem *base, unsigned int val)
{
	int i;

	for (i = 0; i < VIC_NUM_REGS; i++)
		writel(val, base + (i * 4));
}

static void msm_irq_ack(struct irq_data *d)
{
	irq_hw_number_t hwirq = irqd_to_hwirq(d);
	void __iomem *reg = VIC_INT_TO_REG_ADDR(VIC_INT_CLEAR0, hwirq);
	uint32_t mask = 1UL << (hwirq & 31);

	writel(mask, reg);
	mb();
}

static void msm_irq_mask(struct irq_data *d)
{
	irq_hw_number_t hwirq = irqd_to_hwirq(d);
	void __iomem *reg = VIC_INT_TO_REG_ADDR(VIC_INT_ENCLEAR0, hwirq);
	uint32_t mask = 1UL << (hwirq & 31);

	writel(mask, reg);
	mb();
}

static void msm_irq_unmask(struct irq_data *d)
{
	irq_hw_number_t hwirq = irqd_to_hwirq(d);
	void __iomem *reg = VIC_INT_TO_REG_ADDR(VIC_INT_ENSET0, hwirq);
	uint32_t mask = 1UL << (hwirq & 31);

	writel(mask, reg);
	mb();
}

static int msm_irq_set_type(struct irq_data *d, unsigned int flow_type)
{
	irq_hw_number_t hwirq = irqd_to_hwirq(d);
	void __iomem *treg = VIC_INT_TO_REG_ADDR(VIC_INT_TYPE0, hwirq);
	void __iomem *preg = VIC_INT_TO_REG_ADDR(VIC_INT_POLARITY0, hwirq);
	uint32_t mask = 1UL << (hwirq & 31);

	uint32_t polarity = readl(preg);
	uint32_t type = readl(treg);

	if (flow_type & (IRQF_TRIGGER_FALLING | IRQF_TRIGGER_LOW))
		polarity |= mask;
	if (flow_type & (IRQF_TRIGGER_RISING | IRQF_TRIGGER_HIGH))
		polarity &= ~mask;

	if (flow_type & (IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING)) {
		type |= mask;
		irq_set_handler_locked(d, handle_edge_irq);
	}
	if (flow_type & (IRQF_TRIGGER_HIGH | IRQF_TRIGGER_LOW)) {
		type &= ~mask;
		irq_set_handler_locked(d, handle_level_irq);
	}

	writel(polarity, preg);
	writel(type, treg);
	mb();
	return 0;
}

static void __exception_irq_entry vic_handle_irq(struct pt_regs *regs)
{
	struct vic_device *vic = &vic_data;
	u32 irqnr;

	do {
		/* 0xD0 has irq# or old irq# if the irq has been handled
		 * 0xD4 has irq# or -1 if none pending *but* if you just
		 * read 0xD4 you never get the first irq for some reason
		 */
		irqnr = readl_relaxed(vic->base + 0xD0);
		irqnr = readl_relaxed(vic->base + 0xD4);
		if (irqnr == -1)
			break;
		handle_domain_irq(vic->domain, irqnr, regs);
	} while (1);
}

static struct irq_chip msm_irq_chip = {
	.name		= "msm-vic",
	.irq_ack	= msm_irq_ack,
	.irq_mask	= msm_irq_mask,
	.irq_unmask	= msm_irq_unmask,
	.irq_set_type	= msm_irq_set_type,
	.flags		= IRQCHIP_SKIP_SET_WAKE |
			  IRQCHIP_MASK_ON_SUSPEND,
};

static int msm_vic_irqdomain_map(struct irq_domain *d, unsigned int irq,
				 irq_hw_number_t hwirq)
{
	struct vic_device *v = d->host_data;

	irq_set_chip_and_handler(irq, &msm_irq_chip, handle_level_irq);
	irq_set_chip_data(irq, v->base);
	irq_set_probe(irq);

	return 0;
}

static const struct irq_domain_ops msm_vic_irqdomain_ops = {
	.map = msm_vic_irqdomain_map,
	.xlate = irq_domain_xlate_onetwocell,
};

static int __init msm_init_irq(struct device_node *node,
			       struct device_node *parent)
{
	int ret;
	void __iomem *regs;
	uint32_t num_irqs;

	regs = of_iomap(node, 0);
	if (WARN_ON(!regs))
		return -EIO;
	vic_data.base = regs;

	ret = of_property_read_u32(node, "num-irqs", &num_irqs);
	if (ret) {
		pr_err("%s: failed to read num-irqs ret=%d\n", __func__, ret);
		return ret;
	}

	/* select level interrupts */
	msm_irq_write_all_regs(VIC_INT_TYPE0, 0);

	/* select highlevel interrupts */
	msm_irq_write_all_regs(VIC_INT_POLARITY0, 0);

	/* select IRQ for all INTs */
	msm_irq_write_all_regs(VIC_INT_SELECT0, 0);

	/* disable all INTs */
	msm_irq_write_all_regs(VIC_INT_EN0, 0);

	/* don't use vic */
	writel(0, VIC_CONFIG);

	vic_data.domain = irq_domain_add_linear(node, num_irqs,
						&msm_vic_irqdomain_ops,
						&vic_data);
	if (!vic_data.domain) {
		pr_err("%s: failed to register domain\n", __func__);
		return -ENODEV;
	}

	set_handle_irq(vic_handle_irq);

	/* enable interrupt controller */
	writel(1, VIC_INT_MASTEREN);
	mb();

	return 0;
}
IRQCHIP_DECLARE(msm_vic, "qcom,msm-vic", msm_init_irq);
