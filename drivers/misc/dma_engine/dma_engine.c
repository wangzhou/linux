/* SPDX-License-Identifier: GPL-2.0-or-later */
#include <linux/bitmap.h>
#include <linux/dma-mapping.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/pci.h>
#include <linux/printk.h>
#include <linux/slab.h>
#include <asm/page.h>

#define DMA_ENGINE		"dma_engine"
#define PCI_VENDOR_ID_QEMU	0x1234
#define IO_OFFSET_CHAN_SRC	0x1000
#define IO_OFFSET_CHAN_DTS	0x1008
#define IO_OFFSET_CHAN_SIZE	0x1014
#define IO_OFFSET_CHAN_START	0x1018
#define IO_OFFSET_CHAN_DONE	0x101c

static const struct pci_device_id dma_pci_tbl[] = {
	{PCI_VDEVICE(QEMU, 0x3456), 0},
	{0, }
};
MODULE_DEVICE_TABLE(pci, dma_pic_tbl);

struct hw_state {
	struct pci_dev *pdev;
	void *io_va;

	/* dma test parameter */
	unsigned long copy_size;
};

static void do_dma_copy(struct hw_state *hw, dma_addr_t src, dma_addr_t dts, u32 size)
{
	void *base = hw->io_va;
	u32 val = 0;

	writeq(src, base + IO_OFFSET_CHAN_SRC);
	writeq(dts, base + IO_OFFSET_CHAN_DTS);
	writel(size, base + IO_OFFSET_CHAN_SIZE);
	writel(1, base + IO_OFFSET_CHAN_START);

	readl_relaxed_poll_timeout(base + IO_OFFSET_CHAN_DONE, val, (val == 1),
				   10, 1000);
}

static void do_dma_copy_test(struct hw_state *hw)
{
	struct device *dev = &hw->pdev->dev;
	u32 buf_size = hw->copy_size;
	dma_addr_t dma_src;
	void *src;

	src = kmalloc(buf_size * 2, GFP_KERNEL);
	if (!src)
		BUG_ON(1);
	memset(src, 5, buf_size);

	dma_src = dma_map_single(dev, src, buf_size * 2, DMA_BIDIRECTIONAL);
	if (dma_mapping_error(dev, dma_src))
		BUG_ON(1);

	do_dma_copy(hw, dma_src, dma_src + buf_size, buf_size);

	dma_unmap_single(dev, dma_src, buf_size * 2, DMA_BIDIRECTIONAL);

	if (memcmp(src, src + buf_size, buf_size)) {
		dev_info(dev, "dma engine test failed!\n");
		BUG_ON(1);
	}
	dev_info(dev, "dma engine test sucessed!\n");

	kfree(src);
}

static ssize_t copy_size_store(struct device *dev, struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct hw_state *hw = dev_get_drvdata(dev);
	unsigned long size;

	if (kstrtoul(buf, 0, &size) < 0)
		return -EINVAL;

	dev_info(dev, "input size: %lx\n", size);
	hw->copy_size = size;

	do_dma_copy_test(hw);

	return count;
}

static ssize_t copy_size_show(struct device *dev, struct device_attribute *attr,
			      char *buf)
{
	struct hw_state *hw = dev_get_drvdata(dev);
	return sysfs_emit(buf, "0x%lx\n", hw->copy_size);
}
static DEVICE_ATTR_RW(copy_size);

static irqreturn_t dma_irq_handler(int irq, void *opaqu)
{
	struct hw_state *hw = opaqu;

	dev_info(&hw->pdev->dev, "dma interrupt!\n");

	/* todo: report an irq after a dma task is finished */

	return IRQ_HANDLED;
}

static struct hw_state *dma_hw_init(struct pci_dev *pdev, unsigned int irq)
{
	struct device *dev = &pdev->dev;
	struct hw_state *hw;
	int ret;

	hw = devm_kzalloc(dev, sizeof(*hw), GFP_KERNEL);
	if (!hw)
		BUG_ON(1);

	hw->io_va = devm_ioremap_resource(dev, &pdev->resource[0]);
	if (IS_ERR(hw->io_va))
		BUG_ON(1);

	ret = devm_request_irq(dev, irq, dma_irq_handler, 0, DMA_ENGINE, hw);
	if (ret)
		BUG_ON(1);

	ret = sysfs_create_file(&dev->kobj, &dev_attr_copy_size.attr);
	if (ret)
		BUG_ON(1);

	dev_info(dev, "dma engine demo start!\n");

	return hw;
}

static void dma_hw_uninit(struct device *dev, struct hw_state *hw)
{
	sysfs_remove_file(&dev->kobj, &dev_attr_copy_size.attr);
}

static int dma_pci_probe(struct pci_dev *pdev, const struct pci_device_id *ent)
{
	struct hw_state *hw;
	unsigned int irq;
	int ret;

	if (pci_enable_device(pdev))
		BUG_ON(1);

	if (dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(64)))
		BUG_ON(1);

	ret = pci_alloc_irq_vectors(pdev, 1, 1, PCI_IRQ_LEGACY | PCI_IRQ_MSI);
	if (ret < 0)
		BUG_ON(1);
	irq = pci_irq_vector(pdev, 0);
	pci_set_master(pdev);

	hw = dma_hw_init(pdev, irq);
	hw->pdev = pdev;
	pci_set_drvdata(pdev, hw);

	return 0;
}

static void dma_pci_remove(struct pci_dev *pdev)
{
	pci_free_irq_vectors(pdev);
	dma_hw_uninit(&pdev->dev, pci_get_drvdata(pdev));
}

static void dma_pci_shutdown(struct pci_dev *pdev)
{
	dev_info(&pdev->dev, "dma shutdown\n");
}

static struct pci_driver dma_pci_drv = {
	.name     = DMA_ENGINE,
	.id_table = dma_pci_tbl,
	.probe    = dma_pci_probe,
	.remove   = dma_pci_remove,
	.shutdown = dma_pci_shutdown,
};

static int __init dma_init(void)
{
	return pci_register_driver(&dma_pci_drv);
}

static void __exit dma_exit(void)
{
	pci_unregister_driver(&dma_pci_drv);
}

module_init(dma_init);
module_exit(dma_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("DMA engine demo Module");
MODULE_AUTHOR("Sherlock Wang <wangzhou89@126.com>");
