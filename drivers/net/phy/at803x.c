/*
 * drivers/net/phy/at803x.c
 *
 * Driver for Atheros 803x PHY
 *
 * Author: Matus Ujhelyi <ujhelyi.m@gmail.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/phy.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/of_gpio.h>

#define AT803X_INTR_ENABLE			0x12
#define AT803X_INTR_STATUS			0x13
#define AT803X_SMART_SPEED			0x14
#define AT803X_LED_CONTROL			0x18
#define AT803X_WOL_ENABLE			0x01
#define AT803X_DEVICE_ADDR			0x03
#define AT803X_LOC_MAC_ADDR_0_15_OFFSET		0x804C
#define AT803X_LOC_MAC_ADDR_16_31_OFFSET	0x804B
#define AT803X_LOC_MAC_ADDR_32_47_OFFSET	0x804A
#define AT803X_MMD_ACCESS_CONTROL		0x0D
#define AT803X_MMD_ACCESS_CONTROL_DATA		0x0E
#define AT803X_FUNC_DATA			0x4003
#define AT803X_INER				0x0012
#define AT803X_INER_INIT			0xec00
#define AT803X_INSR				0x0013
#define AT803X_DEBUG_ADDR			0x1D
#define AT803X_DEBUG_DATA			0x1E
#define AT803X_DEBUG_SYSTEM_MODE_CTRL		0x05
#define AT803X_DEBUG_RGMII_TX_CLK_DLY		BIT(8)

#define ATH8030_PHY_ID 0x004dd076
#define ATH8031_PHY_ID 0x004dd074
#define ATH8035_PHY_ID 0x004dd072

MODULE_DESCRIPTION("Atheros 803x PHY driver");
MODULE_AUTHOR("Matus Ujhelyi");
MODULE_LICENSE("GPL");

struct at803x_priv {
	bool phy_reset:1;
	int  gpiod_reset;
};

struct at803x_context {
	u16 bmcr;
	u16 advertise;
	u16 control1000;
	u16 int_enable;
	u16 smart_speed;
	u16 led_control;
};

/* save relevant PHY registers to private copy */
static void at803x_context_save(struct phy_device *phydev,
				struct at803x_context *context)
{
	context->bmcr = phy_read(phydev, MII_BMCR);
	context->advertise = phy_read(phydev, MII_ADVERTISE);
	context->control1000 = phy_read(phydev, MII_CTRL1000);
	context->int_enable = phy_read(phydev, AT803X_INTR_ENABLE);
	context->smart_speed = phy_read(phydev, AT803X_SMART_SPEED);
	context->led_control = phy_read(phydev, AT803X_LED_CONTROL);
}

/* restore relevant PHY registers from private copy */
static void at803x_context_restore(struct phy_device *phydev,
				   const struct at803x_context *context)
{
	phy_write(phydev, MII_BMCR, context->bmcr);
	phy_write(phydev, MII_ADVERTISE, context->advertise);
	phy_write(phydev, MII_CTRL1000, context->control1000);
	phy_write(phydev, AT803X_INTR_ENABLE, context->int_enable);
	phy_write(phydev, AT803X_SMART_SPEED, context->smart_speed);
	phy_write(phydev, AT803X_LED_CONTROL, context->led_control);
}
static void at803x_set_wol_mac_addr(struct phy_device *phydev)
{
	struct net_device *ndev = phydev->attached_dev;
	const u8 *mac;
	unsigned int i, offsets[] = {
		AT803X_LOC_MAC_ADDR_32_47_OFFSET,
		AT803X_LOC_MAC_ADDR_16_31_OFFSET,
		AT803X_LOC_MAC_ADDR_0_15_OFFSET,
	};

	if (!ndev)
		return;

	mac = (const u8 *) ndev->dev_addr;

	if (!is_valid_ether_addr(mac))
		return;

	for (i = 0; i < 3; i++) {
		phy_write(phydev, AT803X_MMD_ACCESS_CONTROL,
				  AT803X_DEVICE_ADDR);
		phy_write(phydev, AT803X_MMD_ACCESS_CONTROL_DATA,
				  offsets[i]);
		phy_write(phydev, AT803X_MMD_ACCESS_CONTROL,
				  AT803X_FUNC_DATA);
		phy_write(phydev, AT803X_MMD_ACCESS_CONTROL_DATA,
				  mac[(i * 2) + 1] | (mac[(i * 2)] << 8));
	}
}

static int at803x_probe(struct phy_device *phydev)
{
	struct device *dev = &phydev->dev;
	struct at803x_priv *priv;
	struct device_node *of_node = dev->of_node;
	int ret = 0;
	char reset_gpio_name[20]="reset";

	if (!of_node && dev->parent->of_node)
	{
		of_node = dev->parent->of_node;
		sprintf(reset_gpio_name,"reset%d",phydev->addr);
	}
	
	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;
	
  	priv->gpiod_reset = -EINVAL;
	if(!of_node)
	{
		return 0;
	}
	
	printk("at803x_probe of_node-name=%s reset_gpio_name=%s\n",of_node->name,reset_gpio_name);
	
	priv->gpiod_reset = of_get_named_gpio(of_node, reset_gpio_name, 0);
	if (gpio_is_valid(priv->gpiod_reset)) 
	{
	  printk("at803x_probe RESET PIN = %d\n",priv->gpiod_reset);
	  ret = gpio_request(priv->gpiod_reset, "eth-phy");
	  if (ret < 0)
	    return ret;
	  ret = gpio_direction_output(priv->gpiod_reset,1);
	  if (ret < 0)
	    return ret;
	  
	  gpio_set_value(priv->gpiod_reset, 1);
	  msleep(1);
	} 
	else
	  priv->gpiod_reset = -EINVAL;

	phydev->priv = priv;
	return 0;
}
static int at803x_config_init(struct phy_device *phydev)
{
	int val;
	u32 features;
	int status;

	features = SUPPORTED_TP | SUPPORTED_MII | SUPPORTED_AUI |
		   SUPPORTED_FIBRE | SUPPORTED_BNC;

	val = phy_read(phydev, MII_BMSR);
	if (val < 0)
		return val;

	if (val & BMSR_ANEGCAPABLE)
		features |= SUPPORTED_Autoneg;
	if (val & BMSR_100FULL)
		features |= SUPPORTED_100baseT_Full;
	if (val & BMSR_100HALF)
		features |= SUPPORTED_100baseT_Half;
	if (val & BMSR_10FULL)
		features |= SUPPORTED_10baseT_Full;
	if (val & BMSR_10HALF)
		features |= SUPPORTED_10baseT_Half;

	if (val & BMSR_ESTATEN) {
		val = phy_read(phydev, MII_ESTATUS);
		if (val < 0)
			return val;

		if (val & ESTATUS_1000_TFULL)
			features |= SUPPORTED_1000baseT_Full;
		if (val & ESTATUS_1000_THALF)
			features |= SUPPORTED_1000baseT_Half;
	}

	phydev->supported = features;
	phydev->advertising = features;

	/* enable WOL */
	at803x_set_wol_mac_addr(phydev);
	status = phy_write(phydev, AT803X_INTR_ENABLE, AT803X_WOL_ENABLE);
	status = phy_read(phydev, AT803X_INTR_STATUS);

	return 0;
}


static void at803x_link_change_notify(struct phy_device *phydev)
{
	struct at803x_priv *priv = phydev->priv;

	/*
	 * Conduct a hardware reset for AT8030 every time a link loss is
	 * signalled. This is necessary to circumvent a hardware bug that
	 * occurs when the cable is unplugged while TX packets are pending
	 * in the FIFO. In such cases, the FIFO enters an error mode it
	 * cannot recover from by software. (gpio_is_valid(priv->gpiod_reset))
	 */
	if (phydev->drv->phy_id == ATH8030_PHY_ID) {
		if (phydev->state == PHY_NOLINK) {
			if ((gpio_is_valid(priv->gpiod_reset)) && !priv->phy_reset) {
				struct at803x_context context;

				at803x_context_save(phydev, &context);

				gpio_set_value(priv->gpiod_reset, 0);
				msleep(2);
				gpio_set_value(priv->gpiod_reset, 1);
				msleep(1);
				printk("at8030 phy was reset. RST PIN = %d\n",priv->gpiod_reset);

				at803x_context_restore(phydev, &context);

				dev_dbg(&phydev->dev, "%s(): phy was reset\n",	__func__);
				priv->phy_reset = true;
			}
		} else {
			priv->phy_reset = false;
		}
	}
}
/* ATHEROS 8035 */
static struct phy_driver at8035_driver = {
	.phy_id		= ATH8035_PHY_ID,
	.name		= "Atheros 8035 ethernet",
	.phy_id_mask	= 0xffffffef,
	.probe		= at803x_probe,
	.config_init	= at803x_config_init,
	.link_change_notify	= at803x_link_change_notify,
	.features	= PHY_GBIT_FEATURES,
	.flags		= PHY_HAS_INTERRUPT,
	.config_aneg	= &genphy_config_aneg,
	.read_status	= &genphy_read_status,
	.driver		= {
		.owner = THIS_MODULE,
	},
};

/* ATHEROS 8030 */
static struct phy_driver at8030_driver = {
	.phy_id			= ATH8030_PHY_ID,
	.name		= "Atheros 8030 ethernet",
	.phy_id_mask	= 0xffffffef,
	.probe		= at803x_probe,
	.config_init	= at803x_config_init,
	.link_change_notify	= at803x_link_change_notify,
	.features	= PHY_GBIT_FEATURES,
	.flags		= PHY_HAS_INTERRUPT,
	.config_aneg	= &genphy_config_aneg,
	.read_status	= &genphy_read_status,
	.driver		= {
		.owner = THIS_MODULE,
	},
};

static int __init atheros_init(void)
{
	int ret;

	ret = phy_driver_register(&at8035_driver);
	if (ret)
		goto err1;

	ret = phy_driver_register(&at8030_driver);
	if (ret)
		goto err2;

	return 0;

err2:
	phy_driver_unregister(&at8035_driver);
err1:
	return ret;
}

static void __exit atheros_exit(void)
{
	phy_driver_unregister(&at8035_driver);
	phy_driver_unregister(&at8030_driver);
}

module_init(atheros_init);
module_exit(atheros_exit);

static struct mdio_device_id __maybe_unused atheros_tbl[] = {
	{ ATH8030_PHY_ID, 0xffffffef },
	{ ATH8031_PHY_ID, 0xffffffef },
	{ }
};

MODULE_DEVICE_TABLE(mdio, atheros_tbl);
