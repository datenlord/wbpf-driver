#pragma once

#include <linux/module.h>
#include <linux/clk.h>
#include <linux/cdev.h>
#include <linux/dmaengine.h>

struct wbpf_device_region
{
  phys_addr_t phys;
  void __iomem *virt;
  size_t size;
};

struct wbpf_device
{
  struct wbpf_device_region mmio;
  struct wbpf_device_region dm;
  struct clk *clk;

  int num_pe;
  uint32_t hw_revision_major;
  uint32_t hw_revision_minor;
  int irq;
  int minor;
  atomic_t in_use;
  struct cdev cdev;
  struct device *chrdev;
  wait_queue_head_t intr_wq;
  struct dma_chan *dmem_dma_tx;
  struct mutex dmem_dma_tx_lock;
};

int wbpf_device_probe(struct wbpf_device *wdev);
int wbpf_device_init_dma(struct wbpf_device *wdev);
void wbpf_device_release_dma(struct wbpf_device *wdev);
int wbpf_device_xmit_data_memory_dma(
    struct wbpf_device *wdev,
    uint32_t offset,
    void *src, uint32_t size);

static inline void __iomem *mmio_base_for_core(struct wbpf_device *wdev, size_t core_index)
{
  return wdev->mmio.virt + (core_index + 1) * 0x1000UL;
}

#define WBPF_EXC_INVALID 0
#define WBPF_EXC_NOT_INIT 1
#define WBPF_EXC_PENDING_BRANCH 2
#define WBPF_EXC_BAD_INSTRUCTION 3
#define WBPF_EXC_BAD_MEMORY_ACCESS 4
#define WBPF_EXC_EXIT 5
#define WBPF_EXC_CALL 6
#define WBPF_EXC_STOP 7
