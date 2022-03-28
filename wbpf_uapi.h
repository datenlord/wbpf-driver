#pragma once

#include <linux/module.h>

#define WBPF_IOCTL_LOAD_CODE 0x1

#define MAX_LOAD_CODE_SIZE 16384

struct wbpf_uapi_load_code_args
{
  uint32_t pe_index;
  uint32_t offset;
  unsigned long code; // uaddr
  uint32_t code_len;
};

union wbpf_uapi_arg {
  struct wbpf_uapi_load_code_args load_code;
};
