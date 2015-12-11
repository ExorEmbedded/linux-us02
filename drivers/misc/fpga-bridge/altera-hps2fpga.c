/*
 * FPGA to/from HPS Bridge Driver for Altera SoCFPGA Devices
 *
 *  Copyright (C) 2013 Altera Corporation, All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/of_address.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/err.h>
#include <linux/regmap.h>
#include <linux/reset.h>
#include "fpga-bridge.h"

#define SOCFPGA_RSTMGR_BRGMODRST		0x1c
#define ALT_RSTMGR_BRGMODRST_H2F_MSK		0x00000001
#define ALT_RSTMGR_BRGMODRST_LWH2F_MSK		0x00000002
#define ALT_RSTMGR_BRGMODRST_F2H_MSK		0x00000004

#define ALT_L3_REMAP_OFST			0x0
#define ALT_L3_REMAP_MPUZERO_MSK		0x00000001
#define ALT_L3_REMAP_H2F_MSK			0x00000008
#define ALT_L3_REMAP_LWH2F_MSK			0x00000010

#define HPS2FPGA_BRIDGE_NAME			"hps2fpga"
#define LWHPS2FPGA_BRIDGE_NAME			"lwhps2fpga"
#define FPGA2HPS_BRIDGE_NAME			"fpga2hps"
static struct of_device_id altera_fpga_of_match[];

/* The L3 REMAP register is write only, so keep a cached value. */
static unsigned int l3_remap_value;

struct altera_hps2fpga_data {
	char	name[48];
	struct platform_device *pdev;
	struct device_node *np;
	struct reset_control *bridge_reset;
	struct regmap *l3reg;
	unsigned int reset_mask;
	unsigned int remap_mask;
};

static int alt_hps2fpga_enable_show(struct fpga_bridge *bridge)
{
	struct altera_hps2fpga_data *priv = bridge->priv;

	return reset_control_status(priv->bridge_reset);
}

static inline void _alt_hps2fpga_enable_set(struct altera_hps2fpga_data *priv,
	 bool enable)
{
	/* bring bridge out of reset */
	if (enable)
		reset_control_deassert(priv->bridge_reset);
	else
		reset_control_assert(priv->bridge_reset);

	/* Allow bridge to be visible to L3 masters or not */
	if (priv->remap_mask) {
		l3_remap_value |= ALT_L3_REMAP_MPUZERO_MSK;

		if (enable)
			l3_remap_value |= priv->remap_mask;
		else
			l3_remap_value &= ~priv->remap_mask;

		regmap_write(priv->l3reg, ALT_L3_REMAP_OFST, l3_remap_value);
	}
}

static void alt_hps2fpga_enable_set(struct fpga_bridge *bridge, bool enable)
{
	_alt_hps2fpga_enable_set(bridge->priv, enable);
}

struct fpga_bridge_ops altera_hps2fpga_br_ops = {
	.enable_set = alt_hps2fpga_enable_set,
	.enable_show = alt_hps2fpga_enable_show,
};

static struct altera_hps2fpga_data hps2fpga_data  = {
	.name = HPS2FPGA_BRIDGE_NAME,
	.reset_mask = ALT_RSTMGR_BRGMODRST_H2F_MSK,
	.remap_mask = ALT_L3_REMAP_H2F_MSK,
};

static struct altera_hps2fpga_data lwhps2fpga_data  = {
	.name = LWHPS2FPGA_BRIDGE_NAME,
	.reset_mask = ALT_RSTMGR_BRGMODRST_LWH2F_MSK,
	.remap_mask = ALT_L3_REMAP_LWH2F_MSK,
};

static struct altera_hps2fpga_data fpga2hps_data  = {
	.name = FPGA2HPS_BRIDGE_NAME,
	.reset_mask = ALT_RSTMGR_BRGMODRST_F2H_MSK,
};

static int alt_fpga_bridge_probe(struct platform_device *pdev)
{
	struct altera_hps2fpga_data *priv;
	const struct of_device_id *of_id;
	struct device *dev = &pdev->dev;
	uint32_t init_val;
	int rc;
	struct clk *clk;

	of_id = of_match_device(altera_fpga_of_match, dev);
	priv = (struct altera_hps2fpga_data *)of_id->data;
	WARN_ON(!priv);

	priv->np = dev->of_node;
	priv->pdev = pdev;

	priv->bridge_reset = devm_reset_control_get(dev, priv->name);
	if (IS_ERR(priv->bridge_reset)) {
		dev_err(dev, "Could not get %s reset control!\n", priv->name);
		return PTR_ERR(priv->bridge_reset);
	}

	priv->l3reg = syscon_regmap_lookup_by_compatible("altr,l3regs");
	if (IS_ERR(priv->l3reg)) {
		dev_err(dev, "regmap for altr,l3regs lookup failed.\n");
		return PTR_ERR(priv->l3reg);
	}

	clk = of_clk_get(pdev->dev.of_node, 0);
	if (IS_ERR(clk)) {
		dev_err(dev, "no clock specified\n");
		return PTR_ERR(clk);
	}

	rc = clk_prepare_enable(clk);
	if (rc) {
		dev_err(dev, "could not enable clock\n");
		return -EBUSY;
	}

	rc = register_fpga_bridge(pdev, &altera_hps2fpga_br_ops,
				    priv->name, priv);
	if (rc)
		return rc;

	if (of_property_read_u32(priv->np, "init-val", &init_val))
		dev_info(&priv->pdev->dev, "init-val not specified\n");
	else if (init_val > 1)
		dev_warn(&priv->pdev->dev, "invalid init-val %u > 1\n",
			init_val);
	else {
		dev_info(&priv->pdev->dev, "%s bridge\n",
			(init_val ? "enabling" : "disabling"));

		_alt_hps2fpga_enable_set(priv, init_val);
	}

	return rc;
}

static int alt_fpga_bridge_remove(struct platform_device *pdev)
{
	remove_fpga_bridge(pdev);
	return 0;
}

static struct of_device_id altera_fpga_of_match[] = {
	{ .compatible = "altr,socfpga-hps2fpga-bridge", .data = &hps2fpga_data },
	{ .compatible = "altr,socfpga-lwhps2fpga-bridge", .data = &lwhps2fpga_data },
	{ .compatible = "altr,socfpga-fpga2hps-bridge", .data = &fpga2hps_data },
	{},
};

MODULE_DEVICE_TABLE(of, altera_fpga_of_match);

static struct platform_driver altera_fpga_driver = {
	.remove = alt_fpga_bridge_remove,
	.driver = {
		.name	= "altera_hps2fpga_bridge",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(altera_fpga_of_match),
	},
};

static int __init alt_fpga_bridge_init(void)
{
	return platform_driver_probe(&altera_fpga_driver,
				     alt_fpga_bridge_probe);
}

static void __exit alt_fpga_bridge_exit(void)
{
	platform_driver_unregister(&altera_fpga_driver);
}

arch_initcall(alt_fpga_bridge_init);
module_exit(alt_fpga_bridge_exit);

MODULE_DESCRIPTION("Altera SoCFPGA HPS to FPGA Bridge");
MODULE_AUTHOR("Alan Tull <atull@altera.com>");
MODULE_LICENSE("GPL v2");
