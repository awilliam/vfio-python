# vfio-python
Framework for userspace VFIO drivers and test code written in python

Credit to Olav Morken for their [python-ioctl]: https://github.com/olavmrk/python-ioctl
module used here for VFIO ioctls.

## Documentation

TBD

## NVMe Interrupt test driver

The included NVMe driver will try to open the first NVMe device with vfio support.
Please make sure your user has ownership of the VFIO group file associated with the
device.  For example:

```
$ lspci -Dd ::0108
0000:05:00.0 Non-Volatile memory controller: Silicon Motion, Inc. SM2262/SM2262EN SSD Controller (rev 03)

$ readlink -f /sys/bus/pci/devices/0000:05:00.0/iommu_group
/sys/kernel/iommu_groups/15

$ ls -l /dev/vfio/15
crw-rw----. 1 alwillia alwillia 505, 0 Mar 18 12:41 /dev/vfio/15
```

NB. The code makes use of the vfio-dev directory to detect a device is bound to a 
VFIO driver.  This support was added in kernel v6.1 and as been backported for
various downstream distributions.

The driver initializes the device, creates DMA mapped buffers for admin queue, fills
the command queue with NOP commands, and increments head and tail pointers to trigger
interrupts as quickly as possible.  Each of INTx, MSI, and MSI-X mode interrupts are
exercised:

```
$ python nvme-interrupt-test-driver.py 
Found vfio NVMe device: 0000:05:00.0
Testing INTx...
INTx: 12422.95 interrupts/second
Testing MSI...
MSI: 61855.1 interrupts/second
Testing MSI-X...
MSI-X: 61771.4 interrupts/second
['INTx', 12422.95, 'MSI', 61855.1, 'MSI-X', 61771.4]
```

# Current status

This code initially only supports the "legacy" container/group access to vfio devices
with the "type1" IOMMU backing device.  The intention is to also include support for
VFIO cdev access with IOMMUFD.
