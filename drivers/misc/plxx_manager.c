/*
 *  plxx_manager.c - Linux plugins manager driver
 *
 *  Written by: Giovanni Pavoni, Exor S.p.a.
 *  Copyright (c) 2016 Exor S.p.a.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/kthread.h>
#include <linux/i2c/eeprom.h>
#include <linux/i2c/I2CSeeprom.h>

#include <linux/of.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <dt-bindings/gpio/gpio.h>

static DEFINE_MUTEX(plxx_lock);

struct plxx_data 
{
  struct i2c_client*       seeprom_client;
  struct memory_accessor*  seeprom_macc;            // I2C SEEPROM accessor
  u32                      index;                   // Plugin index
  u32                      sel_gpio;                // Gpio index to select the plugin
  u8                       eeprom[SEE_FACTORYSIZE]; // Image of the I2C SEEPROM contents of the plugin
};

/*
 * static helper function for parsing the DTB tree
 */
#ifdef CONFIG_OF
static int plxx_parse_dt(struct device *dev, struct plxx_data *data)
{
  struct device_node* node = dev->of_node;
  struct device_node* eeprom_node;
  u32                 eeprom_handle;
  int                 ret;

  /* Parse the DT to find the I2C SEEPROM bindings*/
  ret = of_property_read_u32(node, "eeprom", &eeprom_handle);
  if (ret != 0) 
  {
    dev_err(dev, "Failed to locate eeprom\n");
    return -ENODEV;
  }

  eeprom_node = of_find_node_by_phandle(eeprom_handle);
  if (eeprom_node == NULL) 
  {
    dev_err(dev, "Failed to find eeprom node\n");
    return -ENODEV;
  }

  data->seeprom_client = of_find_i2c_device_by_node(eeprom_node);
  if (data->seeprom_client == NULL) 
  {
    dev_err(dev, "Failed to find i2c client\n");
    of_node_put(eeprom_node);
    return -EPROBE_DEFER;
  }
 
  /* release ref to the node and inc reference to the I2C client used */
  of_node_put(eeprom_node);
  eeprom_node = NULL;
  i2c_use_client(data->seeprom_client);
  
  /* And now get the I2C SEEPROM memory accessor */
  data->seeprom_macc = i2c_eeprom_get_memory_accessor(data->seeprom_client);
  if (IS_ERR_OR_NULL(data->seeprom_macc)) 
  {
    dev_err(dev, "Failed to get memory accessor\n");
    return -ENODEV;
  }
  
  /* Get the plugin index */
  ret = of_property_read_u32(node, "index", &data->index);
  if (ret)
    return ret;
  
  /* Get the sel_gpio */
  data->sel_gpio = of_get_named_gpio(node, "sel-gpio", 0);
  if (gpio_is_valid(data->sel_gpio)) 
  {
    ret = gpio_request(data->sel_gpio, "plxx_sel-gpio");
    if (ret < 0)
      return -EPROBE_DEFER;
    ret = gpio_direction_output(data->sel_gpio,0);
    if (ret < 0)
      return ret;
    
    gpio_set_value(data->sel_gpio, 0);
  }
  else
    return -EPROBE_DEFER;
  
  return 0;
}
#else
static int hrs_parse_dt(struct device *dev, struct hrs_data *data)
{
  return -ENODEV;
}
#endif

/*
 * sysfs interface 
 */
static ssize_t show_hwcode(struct device *dev, struct device_attribute *attr, char *buf)
{
  u8 tmp;
  struct plxx_data *data = dev_get_drvdata(dev);
  mutex_lock(&plxx_lock);
  tmp = data->eeprom[SEE_CODE_OFF];
  mutex_unlock(&plxx_lock);
  return sprintf(buf, "%d\n",(u32)tmp);
}

static ssize_t show_hwsubcode(struct device *dev, struct device_attribute *attr, char *buf)
{
  u8 tmp;
  struct plxx_data *data = dev_get_drvdata(dev);
  mutex_lock(&plxx_lock);
  tmp = data->eeprom[SEE_SUBCODE_OFF];
  mutex_unlock(&plxx_lock);
  return sprintf(buf, "%d\n",(u32)tmp);
}

static ssize_t show_fpgacode(struct device *dev, struct device_attribute *attr, char *buf)
{
  u8 tmp;
  struct plxx_data *data = dev_get_drvdata(dev);
  mutex_lock(&plxx_lock);
  tmp = data->eeprom[SEE_XILCODE_OFF];
  mutex_unlock(&plxx_lock);
  return sprintf(buf, "%d\n",(u32)tmp);
}

static ssize_t show_fpgasubcode(struct device *dev, struct device_attribute *attr, char *buf)
{
  u8 tmp;
  struct plxx_data *data = dev_get_drvdata(dev);
  mutex_lock(&plxx_lock);
  tmp = data->eeprom[SEE_XILSUBCODE_OFF];
  mutex_unlock(&plxx_lock);
  return sprintf(buf, "%d\n",(u32)tmp);
}

static ssize_t show_name(struct device *dev, struct device_attribute *attr, char *buf)
{
  struct plxx_data *data = dev_get_drvdata(dev);
  mutex_lock(&plxx_lock);
  memcpy(buf, &data->eeprom[SEE_NAME_OFF], SEE_MODULENAMELEN);
  buf[SEE_MODULENAMELEN-1]=0;
  mutex_unlock(&plxx_lock);
  return SEE_MODULENAMELEN;
}

static DEVICE_ATTR(hwcode, S_IRUGO, show_hwcode, NULL);
static DEVICE_ATTR(hwsubcode, S_IRUGO, show_hwsubcode, NULL);
static DEVICE_ATTR(fpgacode, S_IRUGO, show_fpgacode, NULL);
static DEVICE_ATTR(fpgasubcode, S_IRUGO, show_fpgasubcode, NULL);
static DEVICE_ATTR(name, S_IRUGO, show_name, NULL);

static struct attribute *plxx_sysfs_attributes[] = {
  &dev_attr_name.attr,
  &dev_attr_hwcode.attr,
  &dev_attr_hwsubcode.attr,
  &dev_attr_fpgacode.attr,
  &dev_attr_fpgasubcode.attr,
  NULL
};

static const struct attribute_group plxx_attr_group = {
  .attrs = plxx_sysfs_attributes,
};

/*
 * Probe and remove functions
 */
static int plxx_probe(struct platform_device *pdev)
{
  int res = 0;
  struct plxx_data *data;
  
  data = kzalloc(sizeof(struct plxx_data), GFP_KERNEL);
  if (data == NULL) 
  {
    dev_err(&pdev->dev, "Memory allocation failed\n");
    return -ENOMEM;
  }
  memset(data, 0, sizeof(struct plxx_data));
  dev_set_drvdata(&pdev->dev, data);

  res = plxx_parse_dt(&pdev->dev, data);
  if (res) {
    dev_err(&pdev->dev, "Could not find valid DT data.\n");
    goto plxx_error1;
  }

  //TODO: Read plugin seeprom. Probe fails if no seeprom found
  
  // Create sysfs entry
  res = sysfs_create_group(&pdev->dev.kobj, &plxx_attr_group);
  if (res) 
  {
    dev_err(&pdev->dev, "device create file failed\n");
    goto plxx_error1;
  }

  return res;
plxx_error1:
  kfree(data);
  return res;
}
			     
static int plxx_remove(struct platform_device *pdev)
{
  struct plxx_data *data = dev_get_drvdata(&pdev->dev);
  
  sysfs_remove_group(&pdev->dev.kobj, &plxx_attr_group);
  kfree(data);
  return 0;
}

/*
 * Driver instantiation
 */
#ifdef CONFIG_OF
static struct of_device_id plxx_of_match[] = {
  { .compatible = "exor,plxx_manager" },
  { }
};

MODULE_DEVICE_TABLE(of, plxx_of_match);
#endif

static struct platform_driver plxx_driver = {
  .driver		= {
    .name		= "plxx_manager",
    .owner		= THIS_MODULE,
    .of_match_table	= of_match_ptr(plxx_of_match),
  },
  .probe		= plxx_probe,
  .remove		= plxx_remove,
};

module_platform_driver(plxx_driver);

MODULE_DESCRIPTION("plxx_manager");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:plxx_manager");
