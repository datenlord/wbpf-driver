#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/poll.h>
#include "wbpf_device.h"
#include "wbpf_uapi.h"
#include "wbpf_chrdev.h"

static int fop_open(struct inode *inode, struct file *filp)
{
  struct wbpf_device *box = container_of(inode->i_cdev, struct wbpf_device, cdev);

  if (atomic_cmpxchg(&box->in_use, 0, 1) != 0)
    return -EBUSY;

  filp->private_data = box;
  return 0;
}

static int fop_release(struct inode *inode, struct file *filp)
{
  struct wbpf_device *wdev = filp->private_data;
  atomic_set(&wdev->in_use, 0);
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
  case WBPF_IOC_LOAD_CODE:
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

    dev_info(&wdev->pdev->dev, "code of %u bytes loaded to processing element %u offset %u\n",
             user_arg.load_code.code_len,
             user_arg.load_code.pe_index,
             user_arg.load_code.offset);

    kfree(buffer);
    return 0;
  case WBPF_IOC_STOP:
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
  case WBPF_IOC_START:
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
    return -ENOTTY;
  }
}

static unsigned int fop_poll(struct file *filp, struct poll_table_struct *wait)
{
  // https://stackoverflow.com/a/30038432
  struct wbpf_device *wdev = filp->private_data;
  poll_wait(filp, &wdev->intr_wq, wait);
  return 0;
}

struct file_operations wbpf_chrdev_fops = {
    .owner = THIS_MODULE,
    .open = fop_open,
    .release = fop_release,
    .mmap = fop_mmap,
    .unlocked_ioctl = fop_ioctl,
    .poll = fop_poll,
};