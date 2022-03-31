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

struct chrdev_context
{
  struct wbpf_device *wdev;
  uint64_t pe_exc_generation;
};

static int has_new_exc_generation(struct chrdev_context *cctx, struct wbpf_uapi_pe_exception_state out[MAX_NUM_PE])
{
  unsigned long flags;
  struct wbpf_device *wdev = cctx->wdev;
  int ret = 0;

  spin_lock_irqsave(&wdev->pe_exc_lock, flags);
  if (wdev->pe_exc_generation != cctx->pe_exc_generation)
  {
    if (out)
    {
      memcpy(out, wdev->pe_exc, sizeof(wdev->pe_exc));
      cctx->pe_exc_generation = wdev->pe_exc_generation;
    }
    ret = 1;
  }
  spin_unlock_irqrestore(&wdev->pe_exc_lock, flags);
  return ret;
}

static int fop_open(struct inode *inode, struct file *filp)
{
  struct chrdev_context *cctx;
  struct wbpf_device *box = container_of(inode->i_cdev, struct wbpf_device, cdev);

  cctx = kzalloc(sizeof(struct chrdev_context), GFP_KERNEL);
  if (!cctx)
    return -ENOMEM;

  if (atomic_cmpxchg(&box->in_use, 0, 1) != 0)
  {
    kfree(cctx);
    return -EBUSY;
  }

  cctx->wdev = box;
  filp->private_data = cctx;
  return 0;
}

static int fop_release(struct inode *inode, struct file *filp)
{
  struct chrdev_context *cctx = filp->private_data;
  atomic_set(&cctx->wdev->in_use, 0);
  kfree(cctx);
  return 0;
}

static int fop_mmap(struct file *filp, struct vm_area_struct *vma)
{
  unsigned long map_len = PAGE_ALIGN(vma->vm_end - vma->vm_start);
  struct chrdev_context *cctx = filp->private_data;
  struct wbpf_device *wdev = cctx->wdev;
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
  int i, ret;
  struct chrdev_context *cctx = filp->private_data;
  struct wbpf_device *wdev = cctx->wdev;
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
        ((user_arg.load_code.offset + user_arg.load_code.code_len) >> 3) > wdev->insn_buffer_size ||
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

    writel(user_arg.load_code.offset >> 3, io_addr + 0x0); // refill counter
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
  case WBPF_IOC_WRITE_DM:
    if (copy_from_user(&user_arg.write_dm, (void __user *)arg, sizeof(user_arg.write_dm)))
    {
      return -EFAULT;
    }
    if (user_arg.write_dm.offset + user_arg.write_dm.data_len < user_arg.write_dm.offset ||
        user_arg.write_dm.offset + user_arg.write_dm.data_len > wdev->dm.size)
    {
      return -EINVAL;
    }
    ret = mutex_lock_interruptible(&wdev->dmem_dma_buffer_lock);
    if (ret)
      return ret;
    if (copy_from_user(wdev->dmem_dma_buffer, (void __user *)user_arg.write_dm.data, user_arg.write_dm.data_len))
    {
      mutex_unlock(&wdev->dmem_dma_buffer_lock);
      return -EFAULT;
    }
    ret = wbpf_device_xmit_data_memory_dma(wdev, user_arg.write_dm.offset, wdev->dmem_dma_buffer_phys_addr, user_arg.write_dm.data_len);
    mutex_unlock(&wdev->dmem_dma_buffer_lock);
    return ret;
  case WBPF_IOC_READ_DM:
    if (copy_from_user(&user_arg.read_dm, (void __user *)arg, sizeof(user_arg.read_dm)))
    {
      return -EFAULT;
    }
    if (user_arg.read_dm.offset + user_arg.read_dm.data_len < user_arg.read_dm.offset ||
        user_arg.read_dm.offset + user_arg.read_dm.data_len > wdev->dm.size)
    {
      return -EINVAL;
    }
    ret = mutex_lock_interruptible(&wdev->dmem_dma_buffer_lock);
    if (ret)
      return ret;

    ret = wbpf_device_recv_data_memory_dma(wdev, user_arg.read_dm.offset, wdev->dmem_dma_buffer_phys_addr, user_arg.read_dm.data_len);
    if (!ret)
    {
      unsigned long copy_ret = copy_to_user((void __user *)user_arg.read_dm.data, wdev->dmem_dma_buffer, user_arg.read_dm.data_len);
      if (copy_ret)
        ret = -EFAULT;
    }

    mutex_unlock(&wdev->dmem_dma_buffer_lock);
    return ret;
  case WBPF_IOC_GET_HW_REVISION:
  {
    struct wbpf_uapi_hw_revision hw_revision = {
        .major = wdev->hw_revision_major,
        .minor = wdev->hw_revision_minor,
    };
    if (copy_to_user((void __user *)arg, &hw_revision, sizeof(hw_revision)))
      return -EFAULT;
    return 0;
  }
  case WBPF_IOC_GET_NUM_PE:
  {
    struct wbpf_uapi_num_pe num_pe = {
        .num_pe = wdev->num_pe,
    };
    if (copy_to_user((void __user *)arg, &num_pe, sizeof(num_pe)))
      return -EFAULT;
    return 0;
  }
  case WBPF_IOC_GET_PERFORMANCE_COUNTERS:
  {
    struct wbpf_uapi_performance_counters performance_counters = {0};

    if (copy_from_user(&user_arg.read_performance_counters, (void __user *)arg, sizeof(user_arg.read_performance_counters)))
    {
      return -EFAULT;
    }
    if (user_arg.load_code.pe_index >= wdev->num_pe)
    {
      return -EINVAL;
    }

    performance_counters.cycles =
        (((uint64_t)readl(mmio_base_for_core(wdev, user_arg.read_performance_counters.pe_index) + 0x44)) << 32) |
        (uint64_t)readl(mmio_base_for_core(wdev, user_arg.read_performance_counters.pe_index) + 0x40);

    performance_counters.commits =
        (((uint64_t)readl(mmio_base_for_core(wdev, user_arg.read_performance_counters.pe_index) + 0x4c)) << 32) |
        (uint64_t)readl(mmio_base_for_core(wdev, user_arg.read_performance_counters.pe_index) + 0x48);

    if (copy_to_user(
            (void *)user_arg.read_performance_counters.out,
            &performance_counters,
            min(user_arg.read_performance_counters.size, sizeof(performance_counters))))
      return -EFAULT;
    return 0;
  }
  default:
    return -ENOTTY;
  }
}

static unsigned int fop_poll(struct file *filp, struct poll_table_struct *wait)
{
  // https://stackoverflow.com/a/30038432
  struct chrdev_context *cctx = filp->private_data;
  struct wbpf_device *wdev = cctx->wdev;
  poll_wait(filp, &wdev->intr_wq, wait);
  if (has_new_exc_generation(cctx, NULL))
  {
    return POLLIN | POLLRDNORM;
  }
  return 0;
}

static ssize_t fop_read(struct file *filp, char __user *out, size_t len, loff_t *loff)
{
  // Read exception state.
  int ret;
  struct chrdev_context *cctx = filp->private_data;
  struct wbpf_device *wdev = cctx->wdev;
  struct wbpf_uapi_pe_exception_state exc_state[MAX_NUM_PE];
  size_t copy_len = min(sizeof(exc_state), len);

  memset(exc_state, 0, sizeof(exc_state));

  if (filp->f_flags & O_NONBLOCK)
  {
    if (!has_new_exc_generation(cctx, exc_state))
    {
      return -EAGAIN;
    }
  }
  else
  {
    ret = wait_event_interruptible(wdev->intr_wq, has_new_exc_generation(cctx, exc_state));
    if (ret)
      return ret;
  }

  if (copy_to_user(out, exc_state, copy_len))
    return -EFAULT;
  return copy_len;
}

struct file_operations wbpf_chrdev_fops = {
    .owner = THIS_MODULE,
    .open = fop_open,
    .release = fop_release,
    .mmap = fop_mmap,
    .unlocked_ioctl = fop_ioctl,
    .poll = fop_poll,
    .read = fop_read,
};