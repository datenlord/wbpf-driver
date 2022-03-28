#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <asm/io.h>
#include <linux/device.h>
#include "wbpf_device.h"

const unsigned long HW_FREQ = 50000000;

// TODO: DT
unsigned long LVL_SHFTR_EN = 0xF8000900;

static struct of_device_id wbpf_match_table[] = {
    {
        .compatible = "bluelogic,wbpf1",
    },
    {0}};
MODULE_DEVICE_TABLE(of, wbpf_match_table);

static irqreturn_t handle_wbpf_intr(int irq, void *pdev_v)
{
  struct platform_device *pdev = pdev_v;
  printk(KERN_INFO "wbpf: interrupt received: %s\n", pdev->name);
  return IRQ_HANDLED;
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
  uint32_t old_lvl_shifter;
  struct resource *memres_mmio, *memres_dm;
  struct wbpf_device *wdev;
  void __iomem *lvl_shftr_en;

  irq = platform_get_irq(pdev, 0);
  if (irq < 0)
    return irq;

  memres_mmio = platform_get_resource(pdev, IORESOURCE_MEM, 0);
  if (!memres_mmio)
  {
    pr_err("wbpf: failed to get mmio region\n");
    return -EINVAL;
  }

  memres_dm = platform_get_resource(pdev, IORESOURCE_MEM, 1);
  if (!memres_dm)
  {
    pr_err("wbpf: failed to get dm region\n");
    return -EINVAL;
  }

  wdev = devm_kzalloc(&pdev->dev, sizeof(*wdev), GFP_KERNEL);
  if (!wdev)
  {
    return -ENOMEM;
  }

  ret = load_wbpf_device_region(&wdev->mmio, &pdev->dev, memres_mmio);
  if (ret)
    return ret;
  ret = load_wbpf_device_region(&wdev->dm, &pdev->dev, memres_dm);
  if (ret)
    return ret;

  wdev->clk = devm_clk_get(&pdev->dev, NULL);
  if (IS_ERR(wdev->clk))
  {
    pr_err("wbpf: failed to get clock\n");
    return PTR_ERR(wdev->clk);
  }

  old_rate = clk_get_rate(wdev->clk);
  ret = clk_set_rate(wdev->clk, HW_FREQ);
  if (ret)
  {
    pr_err("wbpf: failed to set clock rate\n");
    return ret;
  }
  pr_info("wbpf: clk rate: %lu -> %lu\n", old_rate, clk_get_rate(wdev->clk));

  lvl_shftr_en = ioremap(LVL_SHFTR_EN, 4);
  if (!lvl_shftr_en)
  {
    return -ENOMEM;
  }

  old_lvl_shifter = readl(lvl_shftr_en);
  writel(0xf, lvl_shftr_en);
  pr_info("wbpf: lvl shifter: %x -> %x\n", old_lvl_shifter, readl(lvl_shftr_en));
  iounmap(lvl_shftr_en);

  ret = wbpf_device_probe(wdev);
  if (ret)
    return ret;
  platform_set_drvdata(pdev, wdev);

  ret = devm_request_irq(&pdev->dev, irq, handle_wbpf_intr, 0, dev_name(&pdev->dev), pdev);
  if (ret)
    return ret;

  pr_info("wbpf: device '%s' registered. irq %d, mmio %08x-%08x, dm %08x-%08x\n",
          pdev->name,
          irq,
          memres_mmio->start, memres_mmio->end,
          memres_dm->start, memres_dm->end);

  return 0;
}

int wbpf_pd_remove(struct platform_device *dev)
{
  pr_info("wbpf: platform device removed\n");
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

int init_module(void)
{
  int err;

  err = platform_driver_register(&wbpf_platform_driver);
  if (err)
    return err;

  pr_info("wbpf: driver loaded\n");
  return 0;
}

void cleanup_module(void)
{
  platform_driver_unregister(&wbpf_platform_driver);
  pr_info("wbpf: driver cleanup\n");
}

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Heyang Zhou <zhy20000919@hotmail.com>");
MODULE_DESCRIPTION("wBPF driver");
