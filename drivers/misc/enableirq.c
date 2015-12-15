#include <linux/module.h>    // included for all kernel modules
#include <linux/kernel.h>    // included for KERN_INFO
#include <linux/init.h>      // included for __init and __exit macros
#include <linux/interrupt.h>    // included for all kernel modules


MODULE_LICENSE("GPL");
MODULE_AUTHOR("GP");
MODULE_DESCRIPTION("Calls enable_irq()");

static int __init enableirq_init(void)
{
    printk(KERN_INFO "Calling enable_irq()\n");
    enable_irq(160);
    printk(KERN_INFO "Calling enable_irq(): DONE\n");
    return 0;    // Non-zero return means that the module couldn't be loaded.
}

static void __exit enableirq_cleanup(void)
{
    printk(KERN_INFO "Cleaning up enableirq module.\n");
}

module_init(enableirq_init);
module_exit(enableirq_cleanup);
