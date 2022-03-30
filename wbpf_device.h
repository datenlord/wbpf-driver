#pragma once

#include <linux/module.h>
#include <linux/clk.h>
#include <linux/cdev.h>
#include <linux/dmaengine.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include "wbpf_uapi.h"

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
  struct platform_device *pdev;

  uint32_t num_pe;
  uint32_t insn_buffer_size;
  uint32_t hw_revision_major;
  uint32_t hw_revision_minor;
  int irq;
  int minor;
  atomic_t in_use;
  struct cdev cdev;
  struct device *chrdev;
  wait_queue_head_t intr_wq;
  struct dma_chan *dmem_dma;
  struct mutex dmem_dma_lock;
  void *dmem_dma_buffer;
  dma_addr_t dmem_dma_buffer_phys_addr;
  struct mutex dmem_dma_buffer_lock;

  spinlock_t pe_exc_lock;
  struct wbpf_uapi_pe_exception_state pe_exc[MAX_NUM_PE];
  uint64_t pe_exc_generation; // used by userspace poll
};

int wbpf_device_probe(struct wbpf_device *wdev);
int wbpf_device_init_dma(struct wbpf_device *wdev);
void wbpf_device_release_dma(struct wbpf_device *wdev);
int wbpf_device_xmit_data_memory_dma(
    struct wbpf_device *wdev,
    uint32_t offset,
    dma_addr_t src, uint32_t size);
int wbpf_device_recv_data_memory_dma(
    struct wbpf_device *wdev,
    uint32_t offset,
    dma_addr_t dst, uint32_t size);
irqreturn_t handle_wbpf_intr(int irq, void *pdev_v);

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
