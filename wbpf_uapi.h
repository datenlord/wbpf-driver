#pragma once

#include <linux/module.h>
#include <asm-generic/ioctl.h>

#define WBPF_IOC_MAGIC 'w'

#define WBPF_IOC_LOAD_CODE _IOW(WBPF_IOC_MAGIC, 1, struct wbpf_uapi_load_code_args)
#define WBPF_IOC_STOP _IOW(WBPF_IOC_MAGIC, 2, struct wbpf_uapi_stop_args)
#define WBPF_IOC_START _IOW(WBPF_IOC_MAGIC, 3, struct wbpf_uapi_start_args)

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
