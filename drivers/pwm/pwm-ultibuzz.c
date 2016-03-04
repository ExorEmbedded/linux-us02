/*
 * Driver for the ultibuzz squarewave generator.
 * 
 * Copyright (C) 2016 Exor International S.p.a.
 * Author: Giovanni Pavoni (Exor Int. S.p.a.)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/pwm.h>
#include <linux/of.h>
#include <linux/of_device.h>

#define ULTIBUZZ_IDVAL 0xaa010100

/*
 * Register map and register fields
 */
#define ULTIBUZZ_ID_REG   0x00    /* ID   register */
#define ULTIBUZZ_CTRL_REG 0x10    /* CTRL register */

#define ID_REG_IDFIELD  0xffffff00 /* ID + version  */
#define ID_REG_REVFIELD 0x000000ff /* Revision mask */

#define CTRL_REG_FREQFIELD 0x00ffffff /* Frequency field */
#define CTRL_REG_ENAFIELD  (1 << 31)  /* Enable */

struct ultibuzz_chip {
  void __iomem*   mmio_base;
  struct pwm_chip chip;
  int             ref_freq;
};

#define to_ultibuzz_chip(chip) container_of(chip, struct ultibuzz_chip, chip)

static int ultibuzz_pwm_config(struct pwm_chip *chip, struct pwm_device *pwm, int duty_ns, int period_ns)
{
  struct ultibuzz_chip* ultibuzz = to_ultibuzz_chip(chip);
  
  int period_us = period_ns / 1000 + 1;
  int freq_hz = (1000000 / (period_us)) + 1;
  int divider = (ultibuzz->ref_freq / freq_hz) & CTRL_REG_FREQFIELD; 
  
  u32 ctrlval = readl(ultibuzz->mmio_base + ULTIBUZZ_CTRL_REG);
  ctrlval &= CTRL_REG_ENAFIELD;
  ctrlval |= divider;
  writel(ctrlval, ultibuzz->mmio_base + ULTIBUZZ_CTRL_REG);
  
  printk("ultibuzz_pwm_config freq_hz=%d ctrlval=0x%x\n",freq_hz,ctrlval);
  return 0;
}

static int ultibuzz_pwm_enable(struct pwm_chip *chip, struct pwm_device *pwm)
{
  struct ultibuzz_chip* ultibuzz = to_ultibuzz_chip(chip);
  
  u32 ctrlval = readl(ultibuzz->mmio_base + ULTIBUZZ_CTRL_REG);
  ctrlval |= CTRL_REG_ENAFIELD;
  writel(ctrlval, ultibuzz->mmio_base + ULTIBUZZ_CTRL_REG);
  
  printk("ultibuzz_pwm_enable ctrlval=0x%x\n",ctrlval);
  return 0;
}

static void ultibuzz_pwm_disable(struct pwm_chip *chip, struct pwm_device *pwm)
{
  struct ultibuzz_chip* ultibuzz = to_ultibuzz_chip(chip);
  
  u32 ctrlval = readl(ultibuzz->mmio_base + ULTIBUZZ_CTRL_REG);
  ctrlval &= ~CTRL_REG_ENAFIELD;
  writel(ctrlval, ultibuzz->mmio_base + ULTIBUZZ_CTRL_REG);
  
  printk("ultibuzz_pwm_disable ctrlval=0x%x\n",ctrlval);
}

static struct pwm_ops ultibuzz_pwm_ops = {
  .enable = ultibuzz_pwm_enable,
  .disable = ultibuzz_pwm_disable,
  .config = ultibuzz_pwm_config,
  .owner = THIS_MODULE,
};

static const struct of_device_id ultibuzz_pwm_dt_ids[] = {
  { .compatible = "exor,ultibuzz",},
  { /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, ultibuzz_pwm_dt_ids);


static int ultibuzz_pwm_probe(struct platform_device *pdev)
{
  const struct of_device_id *of_id = of_match_device(ultibuzz_pwm_dt_ids, &pdev->dev);
  struct ultibuzz_chip *ultibuzz;
  struct resource *r;
  int ret = 0;
  u32 idval;
  struct device_node *np = NULL; 
  
  if (!of_id)
    return -ENODEV;
  
  np = pdev->dev.of_node;
  
  ultibuzz = devm_kzalloc(&pdev->dev, sizeof(*ultibuzz), GFP_KERNEL);
  if (ultibuzz == NULL)
    return -ENOMEM;
  
  ultibuzz->chip.ops = &ultibuzz_pwm_ops;
  ultibuzz->chip.dev = &pdev->dev;
  ultibuzz->chip.base = -1;
  ultibuzz->chip.npwm = 1;
  ultibuzz->chip.can_sleep = true;
  
  ret = of_property_read_u32(np, "ref-freq", &ultibuzz->ref_freq);
  if (ret < 0) 
  {
    dev_err(&pdev->dev, "failed to get ref-freq value: %d\n", ret);
    return ret;
  }
  
  r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
  ultibuzz->mmio_base = devm_ioremap_resource(&pdev->dev, r);
  if (IS_ERR(ultibuzz->mmio_base))
    return PTR_ERR(ultibuzz->mmio_base);
  
  /* Check for ID core match */
  idval = readl(ultibuzz->mmio_base + ULTIBUZZ_ID_REG);
  printk("ultibuzz_pwm_probe: detected ID: 0x%x\n",idval);
  if((idval & ID_REG_IDFIELD) != ULTIBUZZ_IDVAL)
  {
    dev_err(&pdev->dev, "ultibuzz_pwm_probe: wrong ID: 0x%x\n",idval);
    return -1;
  }
  
  ret = pwmchip_add(&ultibuzz->chip);
  if (ret < 0)
    return ret;
  
  platform_set_drvdata(pdev, ultibuzz);
  return 0;
}

static int ultibuzz_pwm_remove(struct platform_device *pdev)
{
  struct ultibuzz_chip *ultibuzz;
  
  ultibuzz = platform_get_drvdata(pdev);
  if (ultibuzz == NULL)
    return -ENODEV;
  
  return pwmchip_remove(&ultibuzz->chip);
}

static struct platform_driver ultibuzz_pwm_driver = {
  .driver = {
    .name = "ultibuzz-pwm",
    .of_match_table = ultibuzz_pwm_dt_ids,
  },
  
  .probe = ultibuzz_pwm_probe,
  .remove = ultibuzz_pwm_remove,
};

module_platform_driver(ultibuzz_pwm_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Giovanni Pavoni, Exor International S.p.a.");
