/*
 * Copyright (C) 2015 Hisilicon Limited, All Rights Reserved.
 * Author: Jun Ma <majun258@huawei.com>
 * Author: Yun Wu <wuyun.wu@huawei.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/msi.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include "irqchip.h"


/* Interrupt numbers per mbigen node supported */
#define IRQS_PER_MBIGEN_NODE	(128)

/* Pin0-pin15 total 16 irqs are reserved for each mbigen chip*/
#define RESERVED_IRQ_PER_MBIGEN_CHIP (16)

#define MBIGEN_INDEX_SHIFT	(12)

/*
 * To calculate the register addr of interrupt, the private index value
 * also should be included except the hardware pin offset value.
 *
 * hwirq[23:12]: index. private index value of interrupt.
		Start from 0 for a device.
 * hwirq[11:0]: pin. hardware pin offset of this interrupt
 */
#define	COMPOSE_MBIGEN_HWIRQ(index, pin)	\
		(((index) << MBIGEN_INDEX_SHIFT) | (pin))

/* get the interrupt pin offset from mbigen hwirq */
#define	GET_IRQ_PIN_OFFSET(hwirq)	((hwirq) & 0xfff)
/* get the private index value from mbigen hwirq */
#define GET_IRQ_INDEX(hwirq)	(((hwirq) >> MBIGEN_INDEX_SHIFT) & 0xfff)

/*
 * In mbigen vector register
 * bit[21:12]: event id value
 * bit[11:0]: device id
 */
#define IRQ_EVENT_ID_SHIFT	(12)
#define IRQ_EVENT_ID_MASK	(0x3ff)

/* register range of mbigen node  */
#define MBIGEN_NODE_OFFSET	0x1000

/* offset of vector register in mbigen node */
#define REG_MBIGEN_VEC_OFFSET	0x200

/* offset of clear register in mbigen node.
 * This register is used to clear the status
 * of interrupt.
 */
#define REG_MBIGEN_CLEAR_OFFSET	0xa00

/*
 * get the base address of mbigen node
 * nid: mbigen node number
 */
#define MBIGEN_NODE_ADDR_BASE(nid)	((nid) * MBIGEN_NODE_OFFSET)

/*
 * struct mbigen_device--Holds the  information of devices connected
 * to mbigen chip
 * @domain: irq domain of this mbigen device.
 * @global_entry: node in a global mbigen device list.
 * @node: represents the mbigen device node defined in device tree.
 * @mgn_data: pointer to mbigen_irq_data
 * @nr_irqs: the total interrupt lines of this device
 * @base: mapped address of mbigen chip which this mbigen device connected.
*/
struct mbigen_device {
	struct irq_domain	*domain;
	struct list_head	global_entry;
	struct device_node	*node;
	struct mbigen_irq_data	*mgn_data;
	unsigned int		nr_irqs;
	void __iomem		*base;
};

/*
 * struct irq_priv_info--structure of irq corresponding information.
 *
 * @global_pin_offset: global pin offset of this irq.
 * @index: private index value of interrupt.(start from 0 for a device)
 * @nid: id of mbigen node this irq connected.
 * @local_pin_offset: local pin offset of interrupt within mbigen node.
 * @reg_offset: Interrupt corresponding register addr offset.
 */
struct irq_priv_info {
	unsigned int	global_pin_offset;
	unsigned int	index;
	unsigned int	nid;
	unsigned int	local_pin_offset;
	unsigned int	reg_offset;
};

/*
 * struct mbigen_irq_data -- private data of each irq
 *
 * @info: structure of irq private information.
 * @dev: mbigen device this irq belong to.
 * @dev_irq: virq number of this interrupt.
 * @msi_irq: Corresponding msi irq number of this interrupt.
 */
struct mbigen_irq_data {
	struct irq_priv_info	info;
	struct mbigen_device	*dev;
	unsigned int	dev_irq;
	unsigned int	msi_irq;
};

/*
 * global mbigen device list including all of the mbigen
 * devices in this system
 */
static LIST_HEAD(mbigen_device_list);
static DEFINE_SPINLOCK(mbigen_device_lock);

static inline int get_mbigen_vec_reg_addr(u32 nid, u32 offset)
{
	return MBIGEN_NODE_ADDR_BASE(nid) + REG_MBIGEN_VEC_OFFSET
	    + (offset * 4);
}

static struct mbigen_irq_data *get_mbigen_irq_data(struct mbigen_device *mgn_dev,
						   struct irq_data *d)
{
	struct irq_priv_info *info;
	u32 index;

	index = GET_IRQ_INDEX(d->hwirq);
	if (index < 0)
		return NULL;

	info = &mgn_dev->mgn_data[index].info;
	info->index = index;
	info->global_pin_offset = GET_IRQ_PIN_OFFSET(d->hwirq);
	info->nid = info->global_pin_offset / IRQS_PER_MBIGEN_NODE;

	info->local_pin_offset = (info->global_pin_offset % IRQS_PER_MBIGEN_NODE)
						- RESERVED_IRQ_PER_MBIGEN_CHIP;

	info->reg_offset = get_mbigen_vec_reg_addr(info->nid, info->local_pin_offset);

	return &mgn_dev->mgn_data[index];
}

static int mbigen_set_affinity(struct irq_data *data,
				const struct cpumask *mask_val,
				bool force)
{
	struct mbigen_irq_data *mgn_irq_data = irq_data_get_irq_chip_data(data);
	struct irq_chip *chip = irq_get_chip(mgn_irq_data->msi_irq);
	struct irq_data *parent_d = irq_get_irq_data(mgn_irq_data->msi_irq);

	if (chip && chip->irq_set_affinity)
		return chip->irq_set_affinity(parent_d, mask_val, force);
	else
		return -EINVAL;
}

static void mbigen_mask_irq(struct irq_data *data)
{
	struct mbigen_irq_data *mgn_irq_data = irq_data_get_irq_chip_data(data);
	struct irq_chip *chip = irq_get_chip(mgn_irq_data->msi_irq);
	struct irq_data *parent_d = irq_get_irq_data(mgn_irq_data->msi_irq);

	if (chip && chip->irq_mask)
		return chip->irq_mask(parent_d);
}

static void mbigen_unmask_irq(struct irq_data *data)
{
	struct mbigen_irq_data *mgn_irq_data = irq_data_get_irq_chip_data(data);
	struct irq_chip *chip = irq_get_chip(mgn_irq_data->msi_irq);
	struct irq_data *parent_d = irq_get_irq_data(mgn_irq_data->msi_irq);

	if (chip && chip->irq_unmask)
		chip->irq_unmask(parent_d);
}

static void mbigen_eoi_irq(struct irq_data *data)
{

	struct mbigen_irq_data *mgn_irq_data = irq_data_get_irq_chip_data(data);
	struct mbigen_device *mgn_dev = mgn_irq_data->dev;
	struct irq_chip *chip = irq_get_chip(mgn_irq_data->msi_irq);
	struct irq_data *parent_d = irq_get_irq_data(mgn_irq_data->msi_irq);
	u32 pin_offset, ofst, mask;

	pin_offset = mgn_irq_data->info.local_pin_offset;

	ofst = pin_offset / 32 * 4;
	mask = 1 << (pin_offset % 32);

	writel_relaxed(mask, mgn_dev->base + ofst
			+ REG_MBIGEN_CLEAR_OFFSET);

	if (chip && chip->irq_eoi)
		chip->irq_eoi(parent_d);
}

static struct irq_chip mbigen_irq_chip = {
	.name = "mbigen-intc-v2",
	.irq_mask = mbigen_mask_irq,
	.irq_unmask = mbigen_unmask_irq,
	.irq_eoi = mbigen_eoi_irq,
	.irq_set_affinity = mbigen_set_affinity,
};

static int mbigen_domain_xlate(struct irq_domain *d,
			       struct device_node *controller,
			       const u32 *intspec, unsigned int intsize,
			       unsigned long *out_hwirq,
			       unsigned int *out_type)
{

	if (d->of_node != controller)
		return -EINVAL;

	if (intsize < 2)
		return -EINVAL;

	/* Compose the hwirq local to mbigen domain
	 * intspec[0]: interrut pin offset
	 * intspec[1]: index(start from 0)
	 */
	*out_hwirq = COMPOSE_MBIGEN_HWIRQ(intspec[1], intspec[0]);
	*out_type = 0;

	return 0;
}

static int mbigen_domain_map(struct irq_domain *d, unsigned int irq,
				irq_hw_number_t hw)
{
	struct mbigen_device *mgn_dev = d->host_data;
	struct mbigen_irq_data *mgn_irq_data;
	struct irq_data *data = irq_get_irq_data(irq);

	mgn_irq_data = get_mbigen_irq_data(mgn_dev, data);
	if (!mgn_irq_data)
		return -EINVAL;

	mgn_irq_data->dev_irq = irq;
	irq_set_chip_data(irq, mgn_irq_data);
	irq_set_chip_and_handler(irq, &mbigen_irq_chip, handle_fasteoi_irq);

	set_irq_flags(irq, IRQF_VALID);

	return 0;
}

static struct irq_domain_ops mbigen_domain_ops = {
	.xlate = mbigen_domain_xlate,
	.map = mbigen_domain_map,
};

/*
 * mbigen_device_init()- initial mbigen devices connected to
 * mbigen chip as a interrupt controller
 */
static int __init mbigen_intc_of_init(struct device_node *node,
				 struct device_node *parent)
{
	struct mbigen_device *mgn_dev;
	struct irq_domain *domain;
	struct mbigen_irq_data *mgn_irq_data;
	u32 nvec;
	int ret;

	mgn_dev = kzalloc(sizeof(*mgn_dev), GFP_KERNEL);
	if (!mgn_dev)
		return -ENOMEM;

	mgn_dev->node = node;

	of_property_read_u32(node, "nr-interrupts", &nvec);
	if (!nvec) {
		ret = -EINVAL;
		goto out_free_dev;
	}

	mgn_dev->nr_irqs = nvec;

	mgn_irq_data = kcalloc(nvec, sizeof(*mgn_irq_data), GFP_KERNEL);
	if (!mgn_irq_data) {
		ret = -ENOMEM;
		goto out_free_dev;
	}

	mgn_dev->mgn_data = mgn_irq_data;

	domain = irq_domain_add_tree(node, &mbigen_domain_ops, mgn_dev);
	if (!domain) {
			ret = -ENOMEM;
			goto out_free_data;
	}
	mgn_dev->domain = domain;

	INIT_LIST_HEAD(&mgn_dev->global_entry);

	/* add this mbigen device into a global list*/
	spin_lock(&mbigen_device_lock);
	list_add(&mgn_dev->global_entry, &mbigen_device_list);
	spin_unlock(&mbigen_device_lock);

	return 0;

out_free_data:
	kfree(mgn_dev->mgn_data);
out_free_dev:
	kfree(mgn_dev);
	pr_err("mbigen-v2:failed to initialize mbigen device:%s (%d)\n",
						node->full_name, ret);
	return ret;
}
IRQCHIP_DECLARE(hisi_mbigen, "hisilicon,mbigen-intc-v2", mbigen_intc_of_init);

MODULE_AUTHOR("Jun Ma <majun258@huawei.com>");
MODULE_AUTHOR("Yun Wu <wuyun.wu@huawei.com>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Hisilicon MBI Generator driver");
