#pragma once

#include <linux/module.h>
#include <asm-generic/ioctl.h>

#define WBPF_IOC_MAGIC 'w'

#define WBPF_IOC_LOAD_CODE _IOW(WBPF_IOC_MAGIC, 1, struct wbpf_uapi_load_code_args)
#define WBPF_IOC_STOP _IOW(WBPF_IOC_MAGIC, 2, struct wbpf_uapi_stop_args)
#define WBPF_IOC_START _IOW(WBPF_IOC_MAGIC, 3, struct wbpf_uapi_start_args)
#define WBPF_IOC_WRITE_DM _IOW(WBPF_IOC_MAGIC, 4, struct wbpf_uapi_write_dm_args)
#define WBPF_IOC_READ_DM _IOW(WBPF_IOC_MAGIC, 5, struct wbpf_uapi_read_dm_args)
#define WBPF_IOC_GET_HW_REVISION _IOR(WBPF_IOC_MAGIC, 6, struct wbpf_uapi_hw_revision)
#define WBPF_IOC_GET_NUM_PE _IOR(WBPF_IOC_MAGIC, 7, struct wbpf_uapi_num_pe)
#define WBPF_IOC_GET_PERFORMANCE_COUNTERS _IOW(WBPF_IOC_MAGIC, 8, struct wbpf_uapi_read_performance_counters_args)

#define MAX_NUM_PE 16

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

struct wbpf_uapi_write_dm_args
{
  uint32_t offset;
  unsigned long data; // uaddr
  uint32_t data_len;
};

struct wbpf_uapi_read_dm_args
{
  uint32_t offset;
  unsigned long data; // uaddr
  uint32_t data_len;
};

struct wbpf_uapi_hw_revision
{
  uint32_t major;
  uint32_t minor;
};

struct wbpf_uapi_num_pe
{
  uint32_t num_pe;
};

struct wbpf_uapi_pe_exception_state
{
  uint32_t pc;
  uint32_t code;
  uint64_t data;
};

struct wbpf_uapi_read_performance_counters_args
{
  uint32_t pe_index;
  unsigned long out; // wbpf_uapi_performance_counters *
  size_t size;
};

struct wbpf_uapi_performance_counters
{
  uint64_t cycles;
  uint64_t commits;
};

union wbpf_uapi_arg
{
  struct wbpf_uapi_load_code_args load_code;
  struct wbpf_uapi_stop_args stop;
  struct wbpf_uapi_start_args start;
  struct wbpf_uapi_write_dm_args write_dm;
  struct wbpf_uapi_read_dm_args read_dm;
  struct wbpf_uapi_read_performance_counters_args read_performance_counters;
};
