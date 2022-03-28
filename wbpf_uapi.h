#pragma once

#include <linux/module.h>

#define WBPF_IOCTL_LOAD_CODE 0x1
#define WBPF_IOCTL_STOP 0x2
#define WBPF_IOCTL_START 0x3

#define MAX_LOAD_CODE_SIZE 16384

struct wbpf_uapi_load_code_args
{
  uint32_t pe_index;
  uint32_t offset;
  unsigned long code; // uaddr
  uint32_t code_len;
};

struct wbpf_uapi_stop_args
{
  uint32_t pe_index;
};

struct wbpf_uapi_start_args
{
  uint32_t pe_index;
  uint32_t pc;
};

union wbpf_uapi_arg
{
  struct wbpf_uapi_load_code_args load_code;
  struct wbpf_uapi_stop_args stop;
  struct wbpf_uapi_start_args start;
};
