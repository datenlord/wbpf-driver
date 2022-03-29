#include "wbpf_device.h"
#include <asm/io.h>
#include <linux/dma-mapping.h>
#include <linux/slab.h>

struct dmacopy_completion_context
{
  struct wait_queue_head *wq;
  uint8_t done;
};

static void dmacopy_completion(void *arg);

int wbpf_device_probe(struct wbpf_device *wdev)
{
  unsigned long i;
  int ret;
  uint32_t pe_info;
  uint32_t hw_revision;
  uint64_t start_time, end_time;
  void *zero_buffer;

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

  dev_info(&wdev->pdev->dev, "hardware revision: %u.%u, number of processing elements: %d\n",
           wdev->hw_revision_major, wdev->hw_revision_minor,
           wdev->num_pe);

  // At this point we haven't enabled IRQ yet.
  start_time = ktime_to_ns(ktime_get());
  for (i = 0; i < wdev->num_pe; i++)
  {
    writel(0x1, mmio_base_for_core(wdev, i) + 0x04); // stop
  }
  for (i = 0; i < wdev->num_pe; i++)
  {
    while (readl(mmio_base_for_core(wdev, i) + 0x20) != WBPF_EXC_STOP)
      cpu_relax();
    writel(0x1, mmio_base_for_core(wdev, i) + 0x20); // ack exception
  }
  end_time = ktime_to_ns(ktime_get());
  dev_info(&wdev->pdev->dev, "reset all processing elements in %llu ns\n", end_time - start_time);

  start_time = ktime_to_ns(ktime_get());
  zero_buffer = kzalloc(wdev->dm.size, GFP_KERNEL);
  if (!zero_buffer)
  {
    return -ENOMEM;
  }
  ret = wbpf_device_xmit_data_memory_dma(wdev, 0, zero_buffer, wdev->dm.size);
  kfree(zero_buffer);
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
  dma_cap_mask_t dma_mask;
  dma_cap_zero(dma_mask);
  dma_cap_set(DMA_SLAVE, dma_mask);

  wdev->dmem_dma = dma_request_channel(dma_mask, NULL, NULL);
  if (IS_ERR(wdev->dmem_dma))
  {
    dev_err(&wdev->pdev->dev, "failed to allocate DMA channel\n");
    return PTR_ERR(wdev->dmem_dma);
  }
  mutex_init(&wdev->dmem_dma_lock);
  return 0;
}

void wbpf_device_release_dma(struct wbpf_device *wdev)
{
  dma_release_channel(wdev->dmem_dma);
}

static int device_xfer(struct wbpf_device *wdev, uint32_t offset, void *kernel_buffer, uint32_t size, int to_device)
{

  int ret;
  struct dma_async_tx_descriptor *txdesc;
  struct device *chan_dev = wdev->dmem_dma->device->dev;
  dma_addr_t dma_buf;
  DECLARE_WAIT_QUEUE_HEAD_ONSTACK(completion);
  struct dmacopy_completion_context completion_ctx = {
      .wq = &completion,
      .done = 0,
  };
  enum dma_data_direction direction = to_device ? DMA_TO_DEVICE : DMA_FROM_DEVICE;

  if (offset + size < offset || offset + size > wdev->dm.size)
  {
    return -EINVAL;
  }

  mutex_lock(&wdev->dmem_dma_lock);

  dma_buf = dma_map_single(chan_dev, kernel_buffer, size, direction);
  ret = dma_mapping_error(chan_dev, dma_buf);
  if (ret)
  {
    dev_err(&wdev->pdev->dev, "xmit: DMA mapping failed\n");
    goto fail_dma_mapping;
  }

  if (to_device)
    txdesc = wdev->dmem_dma->device->device_prep_dma_memcpy(
        wdev->dmem_dma,
        wdev->dm.phys + offset,
        dma_buf,
        size, 0);
  else
    txdesc = wdev->dmem_dma->device->device_prep_dma_memcpy(
        wdev->dmem_dma,
        dma_buf,
        wdev->dm.phys + offset,
        size, 0);

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
  dma_unmap_single(chan_dev, dma_buf, size, direction);
fail_dma_mapping:
  mutex_unlock(&wdev->dmem_dma_lock);
  return ret;
}

int wbpf_device_xmit_data_memory_dma(
    struct wbpf_device *wdev,
    uint32_t offset,
    void *src, uint32_t size)
{
  return device_xfer(wdev, offset, src, size, 1);
}

int wbpf_device_recv_data_memory_dma(
    struct wbpf_device *wdev,
    uint32_t offset,
    void *dst, uint32_t size)
{
  return device_xfer(wdev, offset, dst, size, 0);
}

static void dmacopy_completion(void *arg)
{
  struct dmacopy_completion_context *ctx = arg;
  ctx->done = 1;
  wake_up_all(ctx->wq);
}
