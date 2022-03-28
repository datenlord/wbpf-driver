# wbpf-driver

wBPF Linux driver.

## Usage

This kernel module targets [my fork of linux-xlnx](https://github.com/losfair/linux-xlnx).

```bash
KSRC=/path/to/linux-xlnx make
```

And then on your Zynq-7000:

```
insmod wbpf.ko
```
