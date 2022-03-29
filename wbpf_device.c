#include "wbpf_device.h"
#include <asm/io.h>

int wbpf_device_probe(struct wbpf_device *wdev)
{
  int i;
  uint32_t pe_info;
  uint32_t hw_revision;
  uint64_t start_time, end_time;

  // Read HW rev
  hw_revision = readl(mmio_base_for_core(wdev, 0) + 0x34);
  wdev->hw_revision_major = hw_revision >> 16;
  wdev->hw_revision_minor = hw_revision & 0xffff;
  if (wdev->hw_revision_major != 1)
  {
    pr_err("wbpf: unsupported hardware revision: %u.%u\n",
           wdev->hw_revision_major, wdev->hw_revision_minor);
    return -ENODEV;
  }

  // Read PE info
  pe_info = readl(mmio_base_for_core(wdev, 0) + 0x30);
  wdev->num_pe = pe_info >> 16;

  pr_info("wbpf: hardware revision: %u.%u, number of processing elements: %d\n",
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
    {
      cpu_relax();
    }
    writel(0x1, mmio_base_for_core(wdev, i) + 0x20); // ack exception
  }
  end_time = ktime_to_ns(ktime_get());
  pr_info("wbpf: reset all processing elements in %llu ns\n", end_time - start_time);

  return 0;
}
