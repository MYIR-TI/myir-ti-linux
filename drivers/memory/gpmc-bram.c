// SPDX-License-Identifier: GPL-2.0

#include <linux/err.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/mtd/map.h>
#include <linux/mtd/cfi.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/omap-gpmc.h>

struct gpmc_fpga_bram {
	struct map_info *map;
	struct resource *res;
	int				gpmc_cs;
};

#define DRIVER_NAME "gpmc-bram"

static int gpmc_fpga_bram_parse_dt(struct platform_device *dev)
{
	struct gpmc_fpga_bram *gpmc_fpga_bram = platform_get_drvdata(dev);
	struct device_node *dp = dev->dev.of_node;
	int err;
	u32 bankwidth;
	u32 cs;
	int swap = CFI_LITTLE_ENDIAN;

	if (!dp)
		return -EINVAL;

	err = of_property_read_u32(dp, "reg", &cs);
	if (err) {
		dev_err(&dev->dev, "reg not found in DT\n");
		return -EINVAL;
	}
	gpmc_fpga_bram->gpmc_cs = cs;

	err = of_property_read_u32(dp, "bank-width", &bankwidth);
	if (err) {
		dev_err(&dev->dev, "Can't get bank width from device tree\n");
		return err;
	}

	gpmc_fpga_bram->map->bankwidth = bankwidth;
	gpmc_fpga_bram->map->swap = swap;
	gpmc_fpga_bram->map->device_node = dp;
	gpmc_fpga_bram->map->phys = 0;

	return 0;
}

static int gpmc_fpga_bram_probe(struct platform_device *pdev)
{
	struct gpmc_fpga_bram *gpmc_fpga_bram;
	int err;

	if (! pdev->dev.of_node) {
		dev_err(&pdev->dev, "failed to find of node\n");
		return -ENOMEM;
	}

	gpmc_fpga_bram = devm_kzalloc(&pdev->dev, sizeof(struct gpmc_fpga_bram), GFP_KERNEL);
	if (!gpmc_fpga_bram)
		return -ENOMEM;

	platform_set_drvdata(pdev, gpmc_fpga_bram);

	gpmc_fpga_bram->map = devm_kzalloc(&pdev->dev, sizeof(struct map_info), GFP_KERNEL);
	if (!gpmc_fpga_bram->map)
		return -ENOMEM;

	err = gpmc_fpga_bram_parse_dt(pdev);
	if (err) {
		dev_err(&pdev->dev, "failed to parse dt\n");
		return -ENOMEM;
	}

	gpmc_fpga_bram->res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!gpmc_fpga_bram->res) {
		dev_err(&pdev->dev, "failed to get memory resource\n");
		return -ENOENT;
	}

	gpmc_fpga_bram->map->phys = gpmc_fpga_bram->res->start;
	gpmc_fpga_bram->map->size = resource_size(gpmc_fpga_bram->res);
	gpmc_fpga_bram->map->virt = devm_ioremap_resource(&pdev->dev, gpmc_fpga_bram->res);
	if (IS_ERR(gpmc_fpga_bram->map->virt))
		return PTR_ERR(gpmc_fpga_bram->map->virt);

	simple_map_init(gpmc_fpga_bram->map);

	gpmc_cs_write_reg(0, 0x00,(2 << 23)|(0 << 22)|(0 << 21)|(0 << 18)|(1 << 12)|(2 << 8));

	return 0;
}

static const struct of_device_id gpmc_fpga_bram_ids[] = {
	{ .compatible = "am62l,gpmc-bram" },
	{},
};
MODULE_DEVICE_TABLE(of, gpmc_fpga_bram_ids);

static struct platform_driver gpmc_fpga_bram_driver = {
	.probe = gpmc_fpga_bram_probe,
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = gpmc_fpga_bram_ids,
	},
};

module_platform_driver(gpmc_fpga_bram_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("gpmc fpga bram");