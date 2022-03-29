#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/of.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <asm/io.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/poll.h>
#include "wbpf_device.h"
#include "wbpf_uapi.h"
#include "wbpf_chrdev.h"

#define DEVICE_NAME "wbpf"
#define MAX_NUM_DEVICES 16

static uint8_t minor_alloc_state[MAX_NUM_DEVICES];
DEFINE_MUTEX(minor_alloc_lock);

static int dev_major;
static struct class *dev_cls;

static const unsigned long HW_FREQ = 100000000;

// TODO: DT
static const unsigned long LVL_SHFTR_EN = 0xF8000900;

static struct of_device_id wbpf_match_table[] = {
    {
        .compatible = "bluelogic,wbpf1",
    },
    {0}};
MODULE_DEVICE_TABLE(of, wbpf_match_table);

static irqreturn_t handle_wbpf_intr(int irq, void *pdev_v)
{
  struct platform_device *pdev = pdev_v;
  struct wbpf_device *wdev = platform_get_drvdata(pdev);
  int i;
  uint32_t __iomem *code_reg;
  uint32_t code;

  printk(KERN_INFO "wbpf: interrupt received: %s\n", pdev->name);
  for (i = 0; i < wdev->num_pe; i++)
  {
    code_reg = mmio_base_for_core(wdev, i) + 0x20;
    code = readl(code_reg);
    if (code)
    {
      printk(KERN_INFO "wbpf: code %d from processing element %d\n", code, i);
      writel(0x1, code_reg);
    }
  }

  return IRQ_HANDLED;
}

static int alloc_minor(void)
{
  int i;

  mutex_lock(&minor_alloc_lock);
  for (i = 0; i < MAX_NUM_DEVICES; i++)
  {
    if (minor_alloc_state[i] == 0)
    {
      minor_alloc_state[i] = 1;
      mutex_unlock(&minor_alloc_lock);
      return i;
    }
  }
  mutex_unlock(&minor_alloc_lock);
  return -ENODEV;
}

static int release_minor(int minor)
{
  mutex_lock(&minor_alloc_lock);
  minor_alloc_state[minor] = 0;
  mutex_unlock(&minor_alloc_lock);
  return 0;
}

static int load_wbpf_device_region(struct wbpf_device_region *region, struct device *dev, struct resource *res)
{
  region->phys = res->start;
  region->virt = devm_ioremap_resource(dev, res);
  if (IS_ERR(region->virt))
  {
    return PTR_ERR(region->virt);
  }
  region->size = resource_size(res);
  return 0;
}

// https://stackoverflow.com/a/37255743
static ssize_t wdev_num_pe_show(struct device *dev,
                                struct device_attribute *attr, char *buf)
{
  struct wbpf_device *wdev = dev_get_drvdata(dev);

  // https://www.kernel.org/doc/Documentation/filesystems/sysfs.txt
  // "sysfs allocates a buffer of size (PAGE_SIZE) and passes it to the method."
  return sprintf(buf, "%d\n", wdev->num_pe);
}

static DEVICE_ATTR(num_pe, S_IRUGO, wdev_num_pe_show, NULL);

static struct attribute *wdev_attrs[] = {
    &dev_attr_num_pe.attr,
    NULL};
ATTRIBUTE_GROUPS(wdev);

int wbpf_pd_probe(struct platform_device *pdev)
{
  int ret, irq;
  unsigned long old_rate;
  struct resource *memres_mmio, *memres_dm;
  struct wbpf_device *wdev;

  irq = platform_get_irq(pdev, 0);
  if (irq < 0)
    return irq;

  memres_mmio = platform_get_resource(pdev, IORESOURCE_MEM, 0);
  if (!memres_mmio)
  {
    dev_err(&pdev->dev, "failed to get mmio region\n");
    return -EINVAL;
  }

  memres_dm = platform_get_resource(pdev, IORESOURCE_MEM, 1);
  if (!memres_dm)
  {
    dev_err(&pdev->dev, "failed to get dm region\n");
    return -EINVAL;
  }

  wdev = devm_kzalloc(&pdev->dev, sizeof(*wdev), GFP_KERNEL);
  if (!wdev)
  {
    return -ENOMEM;
  }

  wdev->pdev = pdev;

  init_waitqueue_head(&wdev->intr_wq);

  ret = load_wbpf_device_region(&wdev->mmio, &pdev->dev, memres_mmio);
  if (ret)
    return ret;
  ret = load_wbpf_device_region(&wdev->dm, &pdev->dev, memres_dm);
  if (ret)
    return ret;

  wdev->clk = devm_clk_get(&pdev->dev, NULL);
  if (IS_ERR(wdev->clk))
  {
    dev_err(&pdev->dev, "failed to get clock\n");
    return PTR_ERR(wdev->clk);
  }

  old_rate = clk_get_rate(wdev->clk);
  ret = clk_set_rate(wdev->clk, HW_FREQ);
  if (ret)
  {
    dev_err(&pdev->dev, "failed to set clock rate\n");
    return ret;
  }
  dev_info(&pdev->dev, "clk rate: %lu -> %lu\n", old_rate, clk_get_rate(wdev->clk));

  wdev->irq = irq;

  ret = wbpf_device_init_dma(wdev);
  if (ret)
    goto fail_init_dma;

  ret = wbpf_device_probe(wdev);
  if (ret)
    goto fail_device_probe;
  platform_set_drvdata(pdev, wdev);

  wdev->minor = alloc_minor();
  if (wdev->minor < 0)
  {
    dev_err(&pdev->dev, "failed to allocate minor number\n");
    ret = wdev->minor;
    goto fail_minor_alloc;
  }

  ret = devm_request_irq(&pdev->dev, irq, handle_wbpf_intr, 0, dev_name(&pdev->dev), pdev);
  if (ret)
    goto fail_request_irq;

  cdev_init(&wdev->cdev, &wbpf_chrdev_fops);
  wdev->cdev.owner = THIS_MODULE;

  // TODO: Review ownership issue
  ret = cdev_add(&wdev->cdev, MKDEV(dev_major, wdev->minor), 1);
  if (ret)
    goto fail_cdev_add;

  wdev->chrdev = device_create(dev_cls, NULL, MKDEV(dev_major, wdev->minor),
                               NULL, pdev->name);
  if (IS_ERR(wdev->chrdev))
  {
    ret = PTR_ERR(wdev->chrdev);
    dev_err(&pdev->dev, "failed to create chrdev - error %d\n", ret);
    goto fail_device_create;
  }

  dev_info(&pdev->dev, "device registered. irq %d, mmio %08x-%08x, dm %08x-%08x\n",
           irq,
           memres_mmio->start, memres_mmio->end,
           memres_dm->start, memres_dm->end);

  return 0;

fail_device_create:
  cdev_del(&wdev->cdev);
fail_cdev_add:
fail_request_irq:
  release_minor(wdev->minor);
fail_minor_alloc:
fail_device_probe:
  wbpf_device_release_dma(wdev);
fail_init_dma:
  return ret;
}

int wbpf_pd_remove(struct platform_device *pdev)
{
  struct wbpf_device *wdev = dev_get_drvdata(&pdev->dev);

  device_destroy(dev_cls, MKDEV(dev_major, wdev->minor));
  cdev_del(&wdev->cdev);
  release_minor(wdev->minor);
  wbpf_device_release_dma(wdev);
  dev_info(&pdev->dev, "platform device removed\n");
  return 0;
}

static struct platform_driver wbpf_platform_driver = {
    .probe = wbpf_pd_probe,
    .remove = wbpf_pd_remove,
    .driver = {
        .name = "wbpf_driver",
        .owner = THIS_MODULE,

        // https://lwn.net/Articles/794974/
        .dev_groups = wdev_groups,
        .of_match_table = of_match_ptr(wbpf_match_table),
    },
};


static int global_hw_init(void)
{
  void __iomem *lvl_shftr_en;
  uint32_t old_lvl_shifter;

  lvl_shftr_en = ioremap(LVL_SHFTR_EN, 4);
  if (!lvl_shftr_en)
  {
    return -ENOMEM;
  }

  old_lvl_shifter = readl(lvl_shftr_en);
  writel(0xf, lvl_shftr_en);
  pr_info("wbpf: lvl shifter: %x -> %x\n", old_lvl_shifter, readl(lvl_shftr_en));
  iounmap(lvl_shftr_en);
  return 0;
}

int init_module(void)
{
  int ret;
  dev_t allocated_region;

  ret = global_hw_init();
  if (ret)
    goto fail_global_hw_init;

  ret = alloc_chrdev_region(&allocated_region, 0, MAX_NUM_DEVICES, "wbpf");
  if (ret)
    goto fail_alloc_chrdev;
  dev_major = MAJOR(allocated_region);

  dev_cls = class_create(THIS_MODULE, DEVICE_NAME);
  if (IS_ERR(dev_cls))
  {
    ret = PTR_ERR(dev_cls);
    goto fail_class_create;
  }

  ret = platform_driver_register(&wbpf_platform_driver);
  if (ret)
    goto fail_platform_driver_register;

  pr_info("wbpf: driver loaded\n");
  return 0;

fail_platform_driver_register:
  class_destroy(dev_cls);

fail_class_create:
  unregister_chrdev_region(allocated_region, MAX_NUM_DEVICES);

fail_alloc_chrdev:
fail_global_hw_init:
  return ret;
}

void cleanup_module(void)
{
  platform_driver_unregister(&wbpf_platform_driver);
  class_destroy(dev_cls);
  unregister_chrdev_region(MKDEV(dev_major, 0), MAX_NUM_DEVICES);
  pr_info("wbpf: driver cleanup\n");
}

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Heyang Zhou <zhy20000919@hotmail.com>");
MODULE_DESCRIPTION("wBPF driver");
