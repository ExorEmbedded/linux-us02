/*
 *  Copyright (C) 2012-2013 Altera Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <linux/dw_apb_timer.h>
#include <linux/clk-provider.h>
#include <linux/irqchip.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/of_net.h>
#include <linux/stmmac.h>
#include <linux/phy.h>
#include <linux/micrel_phy.h>
#include <linux/sys_soc.h>

#include <asm/hardware/cache-l2x0.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/pmu.h>

#include "core.h"
#include "socfpga_cti.h"
#include "dma.h"
#include "l2_cache.h"
#include "ocram.h"
#include "cmdline.h"

#if defined(CONFIG_FB_ULTIEVC)
extern int ultievc_init_fb(u32 id);
#endif

void __iomem *socfpga_scu_base_addr = ((void __iomem *)(SOCFPGA_SCU_VIRT_BASE));
void __iomem *sys_manager_base_addr;
void __iomem *rst_manager_base_addr;
void __iomem *sdr_ctl_base_addr;
void __iomem *l3regs_base_addr;

void __iomem *clk_mgr_base_addr;
unsigned long cpu1start_addr;

bool fiq_fix_enable = true; 

static int socfpga_phy_reset_mii(struct mii_bus *bus, int phyaddr);
static int stmmac_plat_init(struct platform_device *pdev);
static void stmmac_fix_mac_speed(void *priv, unsigned int speed);

static struct stmmac_mdio_bus_data stmmacenet_mdio_bus_data = {
	.phy_reset_mii = socfpga_phy_reset_mii,
};

static struct plat_stmmacenet_data stmmacenet0_data = {
	.mdio_bus_data = &stmmacenet_mdio_bus_data,
	.init = &stmmac_plat_init,
	.bus_id = 0,
	.fix_mac_speed = stmmac_fix_mac_speed,
};

static struct plat_stmmacenet_data stmmacenet1_data = {
	.mdio_bus_data = &stmmacenet_mdio_bus_data,
	.init = &stmmac_plat_init,
	.bus_id = 0,
	.fix_mac_speed = stmmac_fix_mac_speed,
};

#ifdef CONFIG_HW_PERF_EVENTS
static struct arm_pmu_platdata socfpga_pmu_platdata = {
	.handle_irq = socfpga_pmu_handler,
	.init = socfpga_init_cti,
	.start = socfpga_start_cti,
	.stop = socfpga_stop_cti,
};
#endif

static const struct of_dev_auxdata socfpga_auxdata_lookup[] __initconst = {
	OF_DEV_AUXDATA("snps,dwmac-3.70a", 0xff700000, NULL, &stmmacenet0_data),
	OF_DEV_AUXDATA("snps,dwmac-3.70a", 0xff702000, NULL, &stmmacenet1_data),
	OF_DEV_AUXDATA("arm,pl330", 0xffe00000, "dma-pl330",
		&dma_platform_data),
	OF_DEV_AUXDATA("arm,pl330", 0xffe01000, "dma-pl330",
		&dma_platform_data),
#ifdef CONFIG_HW_PERF_EVENTS
	OF_DEV_AUXDATA("arm,cortex-a9-pmu", 0, "arm-pmu", &socfpga_pmu_platdata),
#endif
	{ /* sentinel */ }
};

static struct map_desc scu_io_desc __initdata = {
	.virtual	= SOCFPGA_SCU_VIRT_BASE,
	.pfn		= 0, /* run-time */
	.length		= SZ_8K,
	.type		= MT_DEVICE,
};

static struct map_desc uart_io_desc __initdata = {
	.virtual        = 0xfec02000,
	.pfn            = __phys_to_pfn(0xffc02000),
	.length         = SZ_8K,
	.type           = MT_DEVICE,
};

static void __init socfpga_soc_device_init(void)
{
	struct device_node *root;
	struct soc_device *soc_dev;
	struct soc_device_attribute *soc_dev_attr;
	const char *machine;
	u32 id = SOCFPGA_ID_DEFAULT;
	u32 rev = SOCFPGA_REVISION_DEFAULT;
	int display_id;	
	int err;

	root = of_find_node_by_path("/");
	if (!root)
		return;

	err = of_property_read_string(root, "model", &machine);
	if (err)
		return;

	of_node_put(root);

	soc_dev_attr = kzalloc(sizeof(*soc_dev_attr), GFP_KERNEL);
	if (!soc_dev_attr)
		return;

	/* Read Silicon ID from System manager */
	if (sys_manager_base_addr) {
		id =  __raw_readl(sys_manager_base_addr +
			SYSMGR_SILICON_ID1_OFFSET);
		rev = (id & SYSMGR_SILICON_ID1_REV_MASK)
				>> SYSMGR_SILICON_ID1_REV_SHIFT;
		id = (id & SYSMGR_SILICON_ID1_ID_MASK)
				>> SYSMGR_SILICON_ID1_ID_SHIFT;
	}

	soc_dev_attr->soc_id = kasprintf(GFP_KERNEL, "%u", id);
	soc_dev_attr->revision = kasprintf(GFP_KERNEL, "%d", rev);
	soc_dev_attr->machine = kasprintf(GFP_KERNEL, "%s", machine);
	soc_dev_attr->family = "SOCFPGA";

	soc_dev = soc_device_register(soc_dev_attr);
	if (IS_ERR_OR_NULL(soc_dev)) {
		kfree(soc_dev_attr->soc_id);
		kfree(soc_dev_attr->machine);
		kfree(soc_dev_attr->revision);
		kfree(soc_dev_attr);
		return;
	}
#if defined(CONFIG_FB_ULTIEVC)
 	/*
	*  Initialize the UltiEVC framebuffer driver
	*/   
    	display_id = 47; // assume default value;
    	cmdline_getint("hw_dispid", &display_id);
    	printk(KERN_INFO "Initializing ultiEVC with display ID %d\n", display_id);

    	ultievc_init_fb(display_id);
#endif	
	return;
}

static void __init socfpga_scu_map_io(void)
{
	unsigned long base;

	/* Get SCU base */
	asm("mrc p15, 4, %0, c15, c0, 0" : "=r" (base));

	scu_io_desc.pfn = __phys_to_pfn(base);
	iotable_init(&scu_io_desc, 1);
}

static void __init enable_periphs(void)
{
	/* Release all peripherals, except for emacs, from reset.*/
	u32 rstval;
	rstval = RSTMGR_PERMODRST_EMAC0 | RSTMGR_PERMODRST_EMAC1;
	writel(rstval, rst_manager_base_addr + SOCFPGA_RSTMGR_MODPERRST);
}

static int stmmac_mdio_write_null(struct mii_bus *bus, int phyaddr, int phyreg,
			     u16 phydata)
{
	return 0;
}

#define MICREL_KSZ9021_EXTREG_CTRL 11
#define MICREL_KSZ9021_EXTREG_DATA_WRITE 12
#define MICREL_KSZ9021_RGMII_CLK_CTRL_PAD_SCEW 260
#define MICREL_KSZ9021_RGMII_RX_DATA_PAD_SCEW 261

static int ksz9021rlrn_phy_fixup(struct phy_device *phydev)
{
	if (IS_BUILTIN(CONFIG_PHYLIB)) {
		/* min rx data delay */
		phy_write(phydev, MICREL_KSZ9021_EXTREG_CTRL,
			MICREL_KSZ9021_RGMII_RX_DATA_PAD_SCEW | 0x8000);
		phy_write(phydev, MICREL_KSZ9021_EXTREG_DATA_WRITE, 0x0000);

		/* max rx/tx clock delay, min rx/tx control delay */
		phy_write(phydev, MICREL_KSZ9021_EXTREG_CTRL,
			MICREL_KSZ9021_RGMII_CLK_CTRL_PAD_SCEW | 0x8000);
		phy_write(phydev, MICREL_KSZ9021_EXTREG_DATA_WRITE, 0xa0d0);
		phy_write(phydev, MICREL_KSZ9021_EXTREG_CTRL, 0x104);
	}

	return 0;
}

#define EMAC_SPLITTER_CTRL_REG			0x0
#define EMAC_SPLITTER_CTRL_SPEED_MASK		0x3
#define EMAC_SPLITTER_CTRL_SPEED_10		0x2
#define EMAC_SPLITTER_CTRL_SPEED_100		0x3
#define EMAC_SPLITTER_CTRL_SPEED_1000		0x0

static void stmmac_fix_mac_speed(void *priv, unsigned int speed)
{
	void __iomem *base = (void __iomem *)priv;
	u32 val;

	if (!base)
		return;

	val = readl(base + EMAC_SPLITTER_CTRL_REG);
	val &= ~EMAC_SPLITTER_CTRL_SPEED_MASK;

	switch (speed) {
	case 1000:
		val |= EMAC_SPLITTER_CTRL_SPEED_1000;
		break;
	case 100:
		val |= EMAC_SPLITTER_CTRL_SPEED_100;
		break;
	case 10:
		val |= EMAC_SPLITTER_CTRL_SPEED_10;
		break;
	default:
		return;
	}

	writel(val, base + EMAC_SPLITTER_CTRL_REG);
}

static int stmmac_emdio_write(struct mii_bus *bus, int phyaddr, int phyreg,
			     u16 phydata)
{
	int ret = (bus->write)(bus, phyaddr,
		MICREL_KSZ9021_EXTREG_CTRL, 0x8000|phyreg);
	if (ret) {
		pr_warn("stmmac_emdio_write write1 failed %d\n", ret);
		return ret;
	}

	ret = (bus->write)(bus, phyaddr,
		MICREL_KSZ9021_EXTREG_DATA_WRITE, phydata);
	if (ret) {
		pr_warn("stmmac_emdio_write write2 failed %d\n", ret);
		return ret;
	}

	return ret;
}

static int socfpga_phy_reset_mii(struct mii_bus *bus, int phyaddr)
{
	struct phy_device *phydev;

	//AG
	pr_info("%s resetting bus\n", bus->id);
	
	if (of_machine_is_compatible("altr,socfpga-vt"))
		return 0;

	phydev = bus->phy_map[phyaddr];

	if (NULL == phydev) {
		pr_err("%s no phydev found\n", __func__);
		return -EINVAL;
	}

	//AG
	pr_info("%s physical id is \n", phydev->phy_id);
	switch(phydev->phy_id)
	{
		case PHY_ID_KSZ9021RLRN: 
			pr_info("%s writing extended registers to phyaddr %d\n",
				__func__, phyaddr);
				/* add 2 ns of RXC PAD Skew and 2.6 ns of TXC PAD Skew */
			stmmac_emdio_write(bus, phyaddr,
				MICREL_KSZ9021_RGMII_CLK_CTRL_PAD_SCEW, 0xa0d0);

			/* set no PAD skew for data */
			stmmac_emdio_write(bus, phyaddr,
				MICREL_KSZ9021_RGMII_RX_DATA_PAD_SCEW, 0x0000);
			break;
		case 0xb8242824:
			pr_info("Renesas Industrial Ethernet uPD60620 PHY %d attached\n", phyaddr);
			pr_info("%s writing extended registers to phyaddr %d\n",
				__func__, phyaddr);
			//nothing to do so far
			break;
		default:
			//!!!pr_err("%s unexpected PHY ID %08x\n", __func__, phydev->phy_id);
			//!!!return -EINVAL;
			pr_info("PHY %d attached\n", phyaddr);
			pr_info("%s writing extended registers to phyaddr %d\n",
				__func__, phyaddr);
			//nothing to do so far
			break;
			
	}

	bus->write = &stmmac_mdio_write_null;
	return 0;
}

static int stmmac_plat_init(struct platform_device *pdev)
{
	u32 ctrl, val, shift;
	u32 rstmask;
	int phymode;
	struct device_node *np_splitter;
	struct resource res_splitter;
	struct plat_stmmacenet_data *plat;

	if (of_machine_is_compatible("altr,socfpga-vt"))
		return 0;

	phymode = of_get_phy_mode(pdev->dev.of_node);

    	phymode = PHY_INTERFACE_MODE_MII; //!!!
	switch (phymode) {
	case PHY_INTERFACE_MODE_RGMII:
	case PHY_INTERFACE_MODE_RGMII_ID:
		val = SYSMGR_EMACGRP_CTRL_PHYSEL_ENUM_RGMII;
		break;
	case PHY_INTERFACE_MODE_MII:
	case PHY_INTERFACE_MODE_GMII:
		val = SYSMGR_EMACGRP_CTRL_PHYSEL_ENUM_GMII_MII;
		break;
	default:
		pr_err("%s bad phy mode %d", __func__, phymode);
		return -EINVAL;
	}

	np_splitter = of_parse_phandle(pdev->dev.of_node,
			"altr,emac-splitter", 0);
	if (np_splitter) {
		plat = dev_get_platdata(&pdev->dev);

		if (of_address_to_resource(np_splitter, 0, &res_splitter)) {
			pr_err("%s: ERROR: missing emac splitter address\n",
				   __func__);
			return -EINVAL;
		}

		plat->bsp_priv = (void *)devm_ioremap_resource(&pdev->dev,
			&res_splitter);

		if (!plat->bsp_priv) {
			pr_err("%s: ERROR: failed to mapping emac splitter\n",
				   __func__);
			return -EINVAL;
		}

		/* Overwrite val to GMII if splitter core is enabled. The
		 * phymode here is the actual phy mode on phy hardware,
		 * but phy interface from EMAC core is GMII.
		 */
		val = SYSMGR_EMACGRP_CTRL_PHYSEL_ENUM_GMII_MII;
	}

	if (&stmmacenet1_data == pdev->dev.platform_data) {
		shift = SYSMGR_EMACGRP_CTRL_PHYSEL_WIDTH;
		rstmask = RSTMGR_PERMODRST_EMAC1;
	} else if (&stmmacenet0_data == pdev->dev.platform_data) {
		shift = 0;
		rstmask = RSTMGR_PERMODRST_EMAC0;
	} else {
		pr_err("%s unexpected platform data pointer\n", __func__);
		return -EINVAL;
	}

	ctrl = readl(sys_manager_base_addr + SYSMGR_EMACGRP_CTRL_OFFSET);
	ctrl &= ~(SYSMGR_EMACGRP_CTRL_PHYSEL_MASK << shift);
	ctrl |= (val << shift);

	writel(ctrl, (sys_manager_base_addr + SYSMGR_EMACGRP_CTRL_OFFSET));

	ctrl = readl(rst_manager_base_addr + SOCFPGA_RSTMGR_MODPERRST);
	ctrl &= ~(rstmask);
	writel(ctrl, rst_manager_base_addr + SOCFPGA_RSTMGR_MODPERRST);

	return 0;
}

static void __init socfpga_map_io(void)
{
	socfpga_scu_map_io();
	iotable_init(&uart_io_desc, 1);
	early_printk("Early printk initialized\n");
}

static void __init socfpga_sysmgr_init(void)
{
	struct device_node *np;

	np = of_find_compatible_node(NULL, NULL, "altr,sys-mgr");
	if (!np) {
		pr_err("SOCFPGA: Unable to find sys-magr in dtb\n");
		return;
	}

	if (of_property_read_u32(np, "cpu1-start-addr",
			(u32 *) &cpu1start_addr))
		pr_err("SMP: Need cpu1-start-addr in device tree.\n");

	sys_manager_base_addr = of_iomap(np, 0);
	WARN_ON(!sys_manager_base_addr);

	np = of_find_compatible_node(NULL, NULL, "altr,rst-mgr");
	if (!np) {
		pr_err("SOCFPGA: Unable to find rst-mgr in dtb\n");
		return;
	}

	rst_manager_base_addr = of_iomap(np, 0);
	WARN_ON(!rst_manager_base_addr);

	np = of_find_compatible_node(NULL, NULL, "altr,clk-mgr");
	if (!np) {
		pr_err("SOCFPGA: Unable to find clk-mgr\n");
		return;
	}

	clk_mgr_base_addr = of_iomap(np, 0);
	WARN_ON(!clk_mgr_base_addr);

	np = of_find_compatible_node(NULL, NULL, "altr,sdr-ctl");
	if (!np) {
		pr_err("SOCFPGA: Unable to find sdr-ctl\n");
		return;
	}

	sdr_ctl_base_addr = of_iomap(np, 0);
	WARN_ON(!sdr_ctl_base_addr);

	np = of_find_compatible_node(NULL, NULL, "altr,l3regs");
	if (!np) {
		pr_err("SOCFPGA: Unable to find l3regs\n");
		return;
	}

	l3regs_base_addr = of_iomap(np, 0);
	WARN_ON(!l3regs_base_addr);
}

static void __init socfpga_init_irq(void)
{
	irqchip_init();
	socfpga_sysmgr_init();

	of_clk_init(NULL);
	clocksource_of_init();
}

static void socfpga_cyclone5_restart(char mode, const char *cmd)
{
	u32 temp;

	/* Turn on all periph PLL clocks */
	writel(0xffff, clk_mgr_base_addr + SOCFPGA_ENABLE_PLL_REG);

	temp = readl(rst_manager_base_addr + SOCFPGA_RSTMGR_CTRL);

	if (mode == 'h')
		temp |= RSTMGR_CTRL_SWCOLDRSTREQ;
	else
		temp |= RSTMGR_CTRL_SWWARMRSTREQ;
	writel(temp, rst_manager_base_addr + SOCFPGA_RSTMGR_CTRL);
}

static void __init socfpga_cyclone5_init(void)
{
#ifdef CONFIG_CACHE_L2X0
	u32 aux_ctrl = 0;
	socfpga_init_l2_ecc();
	aux_ctrl |= (1 << L2X0_AUX_CTRL_SHARE_OVERRIDE_SHIFT) |
			(1 << L2X0_AUX_CTRL_DATA_PREFETCH_SHIFT) |
			(1 << L2X0_AUX_CTRL_INSTR_PREFETCH_SHIFT);
	l2x0_of_init(aux_ctrl, ~0UL);
#endif
	of_platform_populate(NULL, of_default_bus_match_table,
		socfpga_auxdata_lookup, NULL);

	socfpga_init_ocram_ecc();

	enable_periphs();

	socfpga_soc_device_init();
	if (IS_BUILTIN(CONFIG_PHYLIB))
		phy_register_fixup_for_uid(PHY_ID_KSZ9021RLRN,
			MICREL_PHY_ID_MASK, ksz9021rlrn_phy_fixup);
}

static const char *altera_dt_match[] = {
	"altr,socfpga",
	NULL
};

DT_MACHINE_START(SOCFPGA, "Altera SOCFPGA")
	.smp		= smp_ops(socfpga_smp_ops),
	.map_io		= socfpga_map_io,
	.init_irq	= socfpga_init_irq,
	.init_time	= dw_apb_timer_init,
	.init_machine	= socfpga_cyclone5_init,
	.restart	= socfpga_cyclone5_restart,
	.dt_compat	= altera_dt_match,
MACHINE_END
