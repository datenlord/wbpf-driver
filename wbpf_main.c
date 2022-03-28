#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <asm/io.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/cdev.h>
#include <linux/slab.h>
#include "wbpf_device.h"
#include "wbpf_uapi.h"

#define DEVICE_NAME "wbpf"

// TODO: Multiple devices
static dev_t dev_singleton;
static struct class *dev_cls;

const unsigned long HW_FREQ = 100000000;

// TODO: DT
unsigned long LVL_SHFTR_EN = 0xF8000900;

static struct of_device_id wbpf_match_table[] = {
    {
        .compatible = "bluelogic,wbpf1",
    },
    {0}};
MODULE_DEVICE_TABLE(of, wbpf_match_table);

static int fop_open(struct inode *inode, struct file *filp);
static int fop_release(struct inode *inode, struct file *filp);
static int fop_mmap(struct file *filp, struct vm_area_struct *vma);
static long fop_ioctl(struct file *filp, unsigned int cmd, unsigned long arg);

static const struct file_operations wbpf_fops = {
    .owner = THIS_MODULE,
    .open = fop_open,
    .release = fop_release,
    .mmap = fop_mmap,
    .unlocked_ioctl = fop_ioctl,
};

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

  wdev->irq = irq;

  ret = wbpf_device_probe(wdev);
  if (ret)
    return ret;
  platform_set_drvdata(pdev, wdev);

  ret = devm_request_irq(&pdev->dev, irq, handle_wbpf_intr, 0, dev_name(&pdev->dev), pdev);
  if (ret)
    return ret;

  cdev_init(&wdev->cdev, &wbpf_fops);
  wdev->cdev.owner = THIS_MODULE;

  // TODO: Review ownership issue
  ret = cdev_add(&wdev->cdev, dev_singleton, 1);
  if (ret)
    return ret;

  wdev->chrdev = device_create(dev_cls, NULL, dev_singleton,
                               NULL, pdev->name);
  if (IS_ERR(wdev->chrdev))
  {
    pr_err("wbpf: failed to create chrdev - error %ld\n", PTR_ERR(wdev->chrdev));
    cdev_del(&wdev->cdev);
    return PTR_ERR(wdev->chrdev);
  }

  pr_info("wbpf: device '%s' registered. irq %d, mmio %08x-%08x, dm %08x-%08x\n",
          pdev->name,
          irq,
          memres_mmio->start, memres_mmio->end,
          memres_dm->start, memres_dm->end);

  return 0;
}

int wbpf_pd_remove(struct platform_device *pdev)
{
  struct wbpf_device *wdev = dev_get_drvdata(&pdev->dev);

  device_destroy(dev_cls, dev_singleton);
  cdev_del(&wdev->cdev);
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

static int fop_open(struct inode *inode, struct file *filp)
{
  struct wbpf_device *box = container_of(inode->i_cdev, struct wbpf_device, cdev);
  filp->private_data = box;
  try_module_get(THIS_MODULE);
  return 0;
}

static int fop_release(struct inode *inode, struct file *filp)
{
  module_put(THIS_MODULE);
  return 0;
}

static int fop_mmap(struct file *filp, struct vm_area_struct *vma)
{
  unsigned long map_len = PAGE_ALIGN(vma->vm_end - vma->vm_start);
  struct wbpf_device *wdev = filp->private_data;
  int ret;

  if (map_len > wdev->dm.size)
  {
    return -EINVAL;
  }

  ret = remap_pfn_range(vma, vma->vm_start, wdev->dm.phys >> PAGE_SHIFT, map_len, vma->vm_page_prot);
  if (ret)
    return ret;

  return 0;
}

static long fop_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
  int i;
  struct wbpf_device *wdev = filp->private_data;
  union wbpf_uapi_arg user_arg;
  uint8_t *buffer;
  uint32_t *code_pointer;
  void __iomem *io_addr;

  switch (cmd)
  {
  case WBPF_IOCTL_LOAD_CODE:
    if (copy_from_user(&user_arg.load_code, (void __user *)arg, sizeof(user_arg.load_code)))
    {
      return -EFAULT;
    }
    if (user_arg.load_code.pe_index >= wdev->num_pe)
    {
      return -EINVAL;
    }
    if (
        user_arg.load_code.offset + user_arg.load_code.code_len < user_arg.load_code.offset ||
        user_arg.load_code.offset + user_arg.load_code.code_len > MAX_LOAD_CODE_SIZE ||
        user_arg.load_code.code_len == 0 ||
        (user_arg.load_code.code_len & 0b111) != 0 ||
        (user_arg.load_code.offset & 0b111) != 0)
    {
      return -EINVAL;
    }
    buffer = kmalloc(user_arg.load_code.code_len, GFP_KERNEL);
    if (!buffer)
    {
      return -ENOMEM;
    }
    if (copy_from_user(buffer, (void __user *)user_arg.load_code.code, user_arg.load_code.code_len))
    {
      kfree(buffer);
      return -EFAULT;
    }
    code_pointer = (uint32_t *)buffer;
    i = user_arg.load_code.code_len >> 3;
    io_addr = mmio_base_for_core(wdev, user_arg.load_code.pe_index);

    writel(user_arg.load_code.offset, io_addr + 0x0); // refill counter
    while (i--)
    {
      writel(*(code_pointer++), io_addr + 0x08);
      writel(*(code_pointer++), io_addr + 0x10);
    }

    pr_info("wbpf: code of %u bytes loaded to processing element %u offset %u\n",
            user_arg.load_code.code_len,
            user_arg.load_code.pe_index,
            user_arg.load_code.offset);

    kfree(buffer);
    return 0;
  case WBPF_IOCTL_STOP:
    if (copy_from_user(&user_arg.stop, (void __user *)arg, sizeof(user_arg.stop)))
    {
      return -EFAULT;
    }
    if (user_arg.stop.pe_index >= wdev->num_pe)
    {
      return -EINVAL;
    }
    io_addr = mmio_base_for_core(wdev, user_arg.stop.pe_index);
    writel(0x1, io_addr + 0x04);
    return 0;
  case WBPF_IOCTL_START:
    if (copy_from_user(&user_arg.start, (void __user *)arg, sizeof(user_arg.start)))
    {
      return -EFAULT;
    }
    if (user_arg.start.pe_index >= wdev->num_pe)
    {
      return -EINVAL;
    }
    io_addr = mmio_base_for_core(wdev, user_arg.start.pe_index);
    writel(user_arg.start.pc, io_addr + 0x18);
    return 0;
  default:
    return -EINVAL;
  }
}

int init_module(void)
{
  int ret;

  ret = alloc_chrdev_region(&dev_singleton, 0, 1, "wbpf");
  if (ret)
    goto fail_alloc_chrdev;

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
  unregister_chrdev_region(dev_singleton, 1);

fail_alloc_chrdev:
  return ret;
}

void cleanup_module(void)
{
  platform_driver_unregister(&wbpf_platform_driver);
  class_destroy(dev_cls);
  unregister_chrdev_region(dev_singleton, 1);
  pr_info("wbpf: driver cleanup\n");
}

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Heyang Zhou <zhy20000919@hotmail.com>");
MODULE_DESCRIPTION("wBPF driver");
