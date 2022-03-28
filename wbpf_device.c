#include "wbpf_device.h"
#include <asm/io.h>

int wbpf_device_probe(struct wbpf_device *wdev)
{
  uint32_t pe_info;
  uint32_t hw_revision;

  pe_info = readl(mmio_base_for_core(wdev, 0) + 0x30);
  wdev->num_pe = pe_info >> 16;
  pr_info("wbpf: number of processing elements: %d\n",
          wdev->num_pe);
  hw_revision = readl(mmio_base_for_core(wdev, 0) + 0x34);
  wdev->hw_revision_major = hw_revision >> 16;
  wdev->hw_revision_minor = hw_revision & 0xffff;
  if (wdev->hw_revision_major != 1)
  {
    pr_err("wbpf: unsupported hardware revision: %u.%u\n",
           wdev->hw_revision_major, wdev->hw_revision_minor);
    return -ENODEV;
  }
  pr_info("wbpf: hardware revision: %u.%u\n",
          wdev->hw_revision_major, wdev->hw_revision_minor);
  return 0;
}
