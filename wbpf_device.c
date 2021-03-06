#include "wbpf_device.h"
#include <asm/io.h>
#include <linux/dma-mapping.h>
#include <linux/slab.h>

// in 8-byte words
#define MAX_INSN_BUFFER_SIZE 16384

struct dmacopy_completion_context
{
  struct wait_queue_head *wq;
  uint8_t done;
};

static void dmacopy_completion(void *arg);

static int try_polyfill(struct wbpf_device *wdev, int pe_index, uint32_t code, uint32_t pc, uint64_t data)
{
  if (code == WBPF_EXC_BAD_INSTRUCTION)
  {
    uint64_t insn = data;
    uint8_t opc = insn & 0xff;
    uint8_t dst_reg_index = (insn >> 8) & 0xf;
    uint8_t src_reg_index = (insn >> 12) & 0xf;
    uint64_t imm = (int64_t)(int32_t)(insn >> 32);
    switch (opc)
    {
    case 0x27:
    {
      // mul dst, imm
      uint64_t dst = wbpf_device_read_register(wdev, pe_index, dst_reg_index);
      wbpf_device_write_register(wdev, pe_index, dst_reg_index, dst * imm);
      goto resume;
    }
    case 0x2f:
    {
      // mul dst, src
      uint64_t dst = wbpf_device_read_register(wdev, pe_index, dst_reg_index);
      uint64_t src = wbpf_device_read_register(wdev, pe_index, src_reg_index);
      wbpf_device_write_register(wdev, pe_index, dst_reg_index, dst * src);
      goto resume;
    }
    case 0x24:
    {
      // mul32 dst, imm
      uint32_t dst = wbpf_device_read_register(wdev, pe_index, dst_reg_index);
      wbpf_device_write_register(wdev, pe_index, dst_reg_index, dst * imm);
      goto resume;
    }
    case 0x2c:
    {
      // mul32 dst, src
      uint32_t dst = wbpf_device_read_register(wdev, pe_index, dst_reg_index);
      uint32_t src = wbpf_device_read_register(wdev, pe_index, src_reg_index);
      wbpf_device_write_register(wdev, pe_index, dst_reg_index, dst * src);
      goto resume;
    }
    case 0x37:
    {
      // div dst, imm
      uint64_t dst = wbpf_device_read_register(wdev, pe_index, dst_reg_index);
      if (imm == 0)
        return 0;

      wbpf_device_write_register(wdev, pe_index, dst_reg_index, div64_u64(dst, imm));
      goto resume;
    }
    case 0x3f:
    {
      // div dst, src
      uint64_t dst = wbpf_device_read_register(wdev, pe_index, dst_reg_index);
      uint64_t src = wbpf_device_read_register(wdev, pe_index, src_reg_index);
      if (src == 0)
        return 0;
      wbpf_device_write_register(wdev, pe_index, dst_reg_index, div64_u64(dst, src));
      goto resume;
    }
    case 0x34:
    {
      // div32 dst, imm
      uint32_t imm32 = imm;
      uint64_t dst = wbpf_device_read_register(wdev, pe_index, dst_reg_index) & 0xffffffff;
      if (imm32 == 0)
        return 0;

      do_div(dst, imm32);
      wbpf_device_write_register(wdev, pe_index, dst_reg_index, dst);
      goto resume;
    }
    case 0x3c:
    {
      // div32 dst, src
      uint64_t dst = wbpf_device_read_register(wdev, pe_index, dst_reg_index) & 0xffffffff;
      uint32_t src = wbpf_device_read_register(wdev, pe_index, src_reg_index);
      if (src == 0)
        return 0;
      do_div(dst, src);
      wbpf_device_write_register(wdev, pe_index, dst_reg_index, dst);
      goto resume;
    }
    case 0x97:
    {
      // mod dst, imm
      uint64_t dst = wbpf_device_read_register(wdev, pe_index, dst_reg_index);
      uint64_t rem;
      if (imm == 0)
        return 0;

      div64_u64_rem(dst, imm, &rem);
      wbpf_device_write_register(wdev, pe_index, dst_reg_index, rem);
      goto resume;
    }
    case 0x9f:
    {
      // mod dst, src
      uint64_t rem;
      uint64_t dst = wbpf_device_read_register(wdev, pe_index, dst_reg_index);
      uint64_t src = wbpf_device_read_register(wdev, pe_index, src_reg_index);
      if (src == 0)
        return 0;

      div64_u64_rem(dst, src, &rem);
      wbpf_device_write_register(wdev, pe_index, dst_reg_index, rem);
      goto resume;
    }
    case 0x94:
    {
      // mod32 dst, imm
      uint32_t imm32 = imm;
      uint64_t dst = wbpf_device_read_register(wdev, pe_index, dst_reg_index) & 0xffffffff;
      if (imm32 == 0)
        return 0;

      wbpf_device_write_register(wdev, pe_index, dst_reg_index, do_div(dst, imm32));
      goto resume;
    }
    case 0x9c:
    {
      // mod32 dst, src
      uint64_t dst = wbpf_device_read_register(wdev, pe_index, dst_reg_index) & 0xffffffff;
      uint32_t src = wbpf_device_read_register(wdev, pe_index, src_reg_index);
      if (src == 0)
        return 0;
      wbpf_device_write_register(wdev, pe_index, dst_reg_index, do_div(dst, src));
      goto resume;
    }
    default:
      break;
    }
  }
  return 0;

resume:
  writel(pc + 8, mmio_base_for_core(wdev, pe_index) + 0x18);
  return 1;
}

static int lock_and_update_pe_exc(struct wbpf_device *wdev)
{
  long i;
  unsigned long irq_flags;
  uint8_t new_generation = 0;

  BUG_ON(wdev->num_pe > MAX_NUM_PE);

  spin_lock_irqsave(&wdev->pe_exc_lock, irq_flags);

  for (i = 0; i < wdev->num_pe; i++)
  {
    void *base = mmio_base_for_core(wdev, i);
    void *code_reg = base + 0x20;
    void *pc_reg = base + 0x18;
    void *data_lower_reg = base + 0x28;
    void *data_upper_reg = base + 0x2c;
    uint32_t new_code;
    uint32_t pc;
    uint64_t data;

    // XXX: Ordering: `code` must be read first.
    // Exceptions do not disappear automatically but new exceptions may appear when we are reading
    // the registers.
    new_code = readl(code_reg);
    if (new_code & (1U << 31))
    {
      writel(0x1, code_reg); // ack
      pc = readl(pc_reg);
      data = (((uint64_t)readl(data_upper_reg)) << 32) | (uint64_t)readl(data_lower_reg);

      // Attempt to polyfill the exception.
      if (!try_polyfill(wdev, i, new_code & 0x7FFFFFFF, pc, data))
      {
        wdev->pe_exc[i].code = new_code;
        wdev->pe_exc[i].pc = pc;
        wdev->pe_exc[i].data = data;
        new_generation = 1;
      }
    }
  }

  if (new_generation)
  {
    wdev->pe_exc_generation++;
  }

  spin_unlock_irqrestore(&wdev->pe_exc_lock, irq_flags);
  return new_generation;
}

int wbpf_device_probe(struct wbpf_device *wdev)
{
  unsigned long i;
  int ret;
  uint32_t pe_info;
  uint32_t hw_revision;
  uint64_t start_time, end_time;
  uint64_t busy_loop_start_time;

  // Read HW rev
  hw_revision = readl(mmio_base_for_core(wdev, 0) + 0x34);
  wdev->hw_revision_major = hw_revision >> 16;
  wdev->hw_revision_minor = hw_revision & 0xffff;
  if (wdev->hw_revision_major != 1)
  {
    dev_err(&wdev->pdev->dev, "unsupported hardware revision: %u.%u\n",
            wdev->hw_revision_major, wdev->hw_revision_minor);
    return -ENODEV;
  }

  // Read PE info
  pe_info = readl(mmio_base_for_core(wdev, 0) + 0x30);
  wdev->num_pe = pe_info >> 16;
  if (wdev->num_pe == 0 || wdev->num_pe > MAX_NUM_PE)
  {
    dev_err(&wdev->pdev->dev, "unsupported number of processing elements: %u\n", wdev->num_pe);
    return -ENODEV;
  }

  wdev->insn_buffer_size = readl(mmio_base_for_core(wdev, 0) + 0x0c);
  if (wdev->insn_buffer_size == 0 ||
      wdev->insn_buffer_size > MAX_INSN_BUFFER_SIZE)
  {
    dev_err(&wdev->pdev->dev, "unsupported instruction buffer size: %u\n", wdev->insn_buffer_size);
    return -ENODEV;
  }

  dev_info(&wdev->pdev->dev,
           "hardware revision: %u.%u, number of processing elements: %d, instruction buffer size: %u words\n",
           wdev->hw_revision_major, wdev->hw_revision_minor,
           wdev->num_pe, wdev->insn_buffer_size);

  // At this point we haven't enabled IRQ yet.
  start_time = ktime_to_ns(ktime_get());
  for (i = 0; i < wdev->num_pe; i++)
  {
    writel(0x1, mmio_base_for_core(wdev, i) + 0x04); // stop
  }
  for (i = 0; i < wdev->num_pe; i++)
  {
    busy_loop_start_time = ktime_to_ns(ktime_get());
    while (
        readl(mmio_base_for_core(wdev, i) + 0x20) !=
        ((uint32_t)WBPF_EXC_STOP | (1U << 31)))
    {
      if (ktime_to_ns(ktime_get()) - busy_loop_start_time > 10000000)
      {
        dev_err(&wdev->pdev->dev, "timeout waiting for processing element %lu to stop\n", i);
        return -ENODEV;
      }
      yield();
    }
    // Do not ack yet - `lock_and_update_pe_exc` will do it
  }
  end_time = ktime_to_ns(ktime_get());
  dev_info(&wdev->pdev->dev, "reset all processing elements in %llu ns\n", end_time - start_time);

  // Initial exception state
  lock_and_update_pe_exc(wdev);

  memset(wdev->dmem_dma_buffer, 0, wdev->dm.size);
  start_time = ktime_to_ns(ktime_get());
  ret = wbpf_device_xmit_data_memory_dma(wdev, 0, wdev->dmem_dma_buffer_phys_addr, wdev->dm.size);
  if (ret)
  {
    dev_err(&wdev->pdev->dev, "failed to zero data memory\n");
    return ret;
  }
  end_time = ktime_to_ns(ktime_get());
  dev_info(&wdev->pdev->dev, "cleared %u bytes of data memory in %llu ns\n", wdev->dm.size, end_time - start_time);

  return 0;
}

int wbpf_device_init_dma(struct wbpf_device *wdev)
{
  int ret;
  dma_cap_mask_t dma_mask;
  dma_cap_zero(dma_mask);
  dma_cap_set(DMA_SLAVE, dma_mask);

  if (wdev->dm.size & (PAGE_SIZE - 1))
  {
    dev_err(&wdev->pdev->dev, "data memory size must be page aligned\n");
    return -EINVAL;
  }

  wdev->dmem_dma_buffer = dma_alloc_coherent(&wdev->pdev->dev, wdev->dm.size, &wdev->dmem_dma_buffer_phys_addr, GFP_KERNEL);
  if (!wdev->dmem_dma_buffer)
  {
    return -ENOMEM;
  }

  wdev->dmem_dma = dma_request_channel(dma_mask, NULL, NULL);
  if (IS_ERR(wdev->dmem_dma))
  {
    dev_err(&wdev->pdev->dev, "failed to allocate DMA channel\n");
    ret = PTR_ERR(wdev->dmem_dma);
    goto fail_request_channel;
  }

  mutex_init(&wdev->dmem_dma_lock);
  mutex_init(&wdev->dmem_dma_buffer_lock);
  return 0;

fail_request_channel:
  dma_free_coherent(&wdev->pdev->dev, wdev->dm.size, wdev->dmem_dma_buffer, wdev->dmem_dma_buffer_phys_addr);
  return ret;
}

void wbpf_device_release_dma(struct wbpf_device *wdev)
{
  mutex_destroy(&wdev->dmem_dma_buffer_lock);
  mutex_destroy(&wdev->dmem_dma_lock);
  dma_release_channel(wdev->dmem_dma);
  dma_free_coherent(&wdev->pdev->dev, wdev->dm.size, wdev->dmem_dma_buffer, wdev->dmem_dma_buffer_phys_addr);
}

static int device_xfer(struct wbpf_device *wdev, uint32_t offset, dma_addr_t dma_buffer, uint32_t size, int to_device)
{
  int ret;
  struct dma_async_tx_descriptor *txdesc;
  DECLARE_WAIT_QUEUE_HEAD_ONSTACK(completion);
  struct dmacopy_completion_context completion_ctx = {
      .wq = &completion,
      .done = 0,
  };

  if ((offset & 0x7) || (size & 0x7))
  {
    dev_err(&wdev->pdev->dev, "offset and size must be 8 byte aligned\n");
    return -EINVAL;
  }

  if (size == 0)
  {
    dev_err(&wdev->pdev->dev, "size must be greater than 0\n");
    return -EINVAL;
  }

  if (offset + size < offset || offset + size > wdev->dm.size)
  {
    dev_err(&wdev->pdev->dev, "invalid size\n");
    return -EINVAL;
  }

  ret = mutex_lock_interruptible(&wdev->dmem_dma_lock);
  if (ret)
    return ret;

  if (to_device)
    txdesc = wdev->dmem_dma->device->device_prep_dma_memcpy(
        wdev->dmem_dma,
        wdev->dm.phys + offset,
        dma_buffer,
        size, DMA_PREP_INTERRUPT);
  else
    txdesc = wdev->dmem_dma->device->device_prep_dma_memcpy(
        wdev->dmem_dma,
        dma_buffer,
        wdev->dm.phys + offset,
        size, DMA_PREP_INTERRUPT);

  if (!txdesc)
  {
    ret = -EINVAL;
    goto fail_prep;
  }

  txdesc->callback = dmacopy_completion;
  txdesc->callback_param = &completion_ctx;
  ret = dma_submit_error(dmaengine_submit(txdesc));
  if (ret)
  {
    dev_err(&wdev->pdev->dev, "xmit: DMA submit failed\n");
    goto fail_submit;
  }

  dma_async_issue_pending(wdev->dmem_dma);
  wait_event(completion, completion_ctx.done);
  ret = 0;

fail_submit:
fail_prep:
  mutex_unlock(&wdev->dmem_dma_lock);
  return ret;
}

int wbpf_device_xmit_data_memory_dma(
    struct wbpf_device *wdev,
    uint32_t offset,
    dma_addr_t src, uint32_t size)
{
  return device_xfer(wdev, offset, src, size, 1);
}

int wbpf_device_recv_data_memory_dma(
    struct wbpf_device *wdev,
    uint32_t offset,
    dma_addr_t dst, uint32_t size)
{
  return device_xfer(wdev, offset, dst, size, 0);
}

uint64_t wbpf_device_read_register(struct wbpf_device *wdev, uint32_t pe_index, uint32_t regindex)
{
  void *lower;

  BUG_ON(pe_index >= wdev->num_pe || regindex >= 16);
  lower = mmio_base_for_core(wdev, pe_index) + 0x80 + regindex * 8;
  return ((uint64_t)readl(lower + 4) << 32) | (uint64_t)readl(lower);
}

void wbpf_device_write_register(struct wbpf_device *wdev, uint32_t pe_index, uint32_t regindex, uint64_t value)
{
  BUG_ON(pe_index >= wdev->num_pe || regindex >= 16);
  writel((uint32_t)value, mmio_base_for_core(wdev, pe_index) + 0x08);
  writel((uint32_t)(value >> 32), mmio_base_for_core(wdev, pe_index) + 0x80 + regindex * 8 + 4);
}

static void dmacopy_completion(void *arg)
{
  struct dmacopy_completion_context *ctx = arg;
  ctx->done = 1;
  wake_up_all(ctx->wq);
}

irqreturn_t prepare_wbpf_intr(int irq, void *pdev_v)
{
  struct platform_device *pdev = pdev_v;
  struct wbpf_device *wdev = platform_get_drvdata(pdev);
  int i;

  // Quick check
  for (i = 0; i < wdev->num_pe; i++)
  {
    void *base = mmio_base_for_core(wdev, i);
    void *code_reg = base + 0x20;
    uint32_t new_code = readl(code_reg);
    if (new_code & (1U << 31))
    {
      disable_irq_nosync(irq);
      return IRQ_WAKE_THREAD;
    }
  }

  return IRQ_NONE;
}

irqreturn_t handle_wbpf_intr(int irq, void *pdev_v)
{
  struct platform_device *pdev = pdev_v;
  struct wbpf_device *wdev = platform_get_drvdata(pdev);

  // printk(KERN_INFO "wbpf: interrupt received: %s\n", pdev->name);
  if (lock_and_update_pe_exc(wdev))
  {
    wake_up_interruptible(&wdev->intr_wq);
  }

  enable_irq(irq);
  return IRQ_HANDLED;
}
