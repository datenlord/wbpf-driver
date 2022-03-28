#include "wbpf_device.h"
#include <asm/io.h>

int wbpf_device_probe(struct wbpf_device *wdev)
{
  uint32_t reg;
  
  reg = readl(mmio_base_for_core(wdev, 0) + 0x30);
  wdev->num_pe = reg >> 16;
  pr_info("Number of processing elements: %d\n",
          wdev->num_pe);
  return 0;
}
