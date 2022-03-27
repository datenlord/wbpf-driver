#include <linux/kernel.h>
#include <linux/module.h>

int init_module(void)
{
  pr_info("init\n");
  return 0;
}

void cleanup_module(void)
{
  pr_info("cleanup\n");
}

MODULE_LICENSE("GPL");
