import ioctl
import ioctl.linux
import os
import ctypes


class VFIODevice:
    VFIODeviceIoctls = {
        "VFIO_DEVICE_GET_INFO": ioctl.linux.IO(';', 107),
        "VFIO_DEVICE_GET_REGION_INFO": ioctl.linux.IO(';', 108),
        "VFIO_DEVICE_GET_IRQ_INFO": ioctl.linux.IO(';', 109),
        "VFIO_DEVICE_SET_IRQS": ioctl.linux.IO(';', 110),
        "VFIO_DEVICE_RESET": ioctl.linux.IO(';', 111),
    }

    class VFIODeviceInfo(ctypes.Structure):
        _fields_ = [
            ('argsz', ctypes.c_uint32),
            ('flags', ctypes.c_uint32),
            ('num_regions', ctypes.c_uint32),
            ('num_irqs', ctypes.c_uint32),
        ]

    VFIODeviceInfoFlags = {
        "VFIO_DEVICE_FLAGS_RESET": 1,
        "VFIO_DEVICE_FLAGS_PCI": 2,
    }

    class VFIODeviceRegionInfo(ctypes.Structure):
        _fields_ = [
            ('argsz', ctypes.c_uint32),
            ('flags', ctypes.c_uint32),
            ('index', ctypes.c_uint32),
            ('size', ctypes.c_uint64),
            ('offset', ctypes.c_uint64),
        ]

    VFIORegionInfoFlags = {
        "VFIO_REGION_INFO_FLAG_READ": 1,
        "VFIO_REGION_INFO_FLAG_WRITE": 2,
        "VFIO_REGION_INFO_FLAG_MMAP": 4,
    }

    class VFIODeviceIRQInfo(ctypes.Structure):
        _fields_ = [
            ('argsz', ctypes.c_uint32),
            ('flags', ctypes.c_uint32),
            ('index', ctypes.c_uint32),
            ('count', ctypes.c_uint32),
        ]

    VFIODeviceIRQInfoFlags = {
        "VFIO_IRQ_INFO_EVENTFD": 1,
        "VFIO_IRQ_INFO_MASKABLE": 2,
        "VFIO_IRQ_INFO_AUTOMASKED": 4,
        "VFIO_IRQ_INFO_NORESIZE": 8,
    }

    VFIODeviceIRQSetFlags = {
        "VFIO_IRQ_SET_DATA_NONE": 1,
        "VFIO_IRQ_SET_DATA_BOOL": 2,
        "VFIO_IRQ_SET_DATA_EVENTFD": 4,
        "VFIO_IRQ_SET_ACTION_MASK": 8,
        "VFIO_IRQ_SET_ACTION_UNMASK": 16,
        "VFIO_IRQ_SET_ACTION_TRIGGER": 32,
    }

    class VFIODeviceIRQSet(ctypes.Structure):
        _fields_ = [
            ('argsz', ctypes.c_uint32),
            ('flags', ctypes.c_uint32),
            ('index', ctypes.c_uint32),
            ('start', ctypes.c_uint32),
            ('count', ctypes.c_uint32),
            ('eventfd', ctypes.c_int32),
        ]

    def set_irq(self, index, fd):
        irq_set = self.VFIODeviceIRQSet(argsz=ctypes.sizeof(self.VFIODeviceIRQSet),
                                        flags=self.VFIODeviceIRQSetFlags["VFIO_IRQ_SET_DATA_EVENTFD"] |
                                              self.VFIODeviceIRQSetFlags["VFIO_IRQ_SET_ACTION_TRIGGER"],
                                        index=index, start=0, count=1, eventfd=fd)
        ioctl.ioctl(self.fd, self.VFIODeviceIoctls["VFIO_DEVICE_SET_IRQS"], ctypes.byref(irq_set))

    def disable_irq(self, index):
        irq_set = self.VFIODeviceIRQSet(argsz=ctypes.sizeof(self.VFIODeviceIRQSet),
                                        flags=self.VFIODeviceIRQSetFlags["VFIO_IRQ_SET_ACTION_TRIGGER"] |
                                              self.VFIODeviceIRQSetFlags["VFIO_IRQ_SET_DATA_NONE"],
                                        index=index, start=0, count=0)
        ioctl.ioctl(self.fd, self.VFIODeviceIoctls["VFIO_DEVICE_SET_IRQS"], ctypes.byref(irq_set))

    def set_unmask_fd(self, index, fd):
        irq_set = self.VFIODeviceIRQSet(argsz=ctypes.sizeof(self.VFIODeviceIRQSet),
                                        flags=self.VFIODeviceIRQSetFlags["VFIO_IRQ_SET_DATA_EVENTFD"] |
                                              self.VFIODeviceIRQSetFlags["VFIO_IRQ_SET_ACTION_UNMASK"],
                                        index=index, start=0, count=1, eventfd=fd)
        ioctl.ioctl(self.fd, self.VFIODeviceIoctls["VFIO_DEVICE_SET_IRQS"], ctypes.byref(irq_set))

    def unmask_irq(self, index):
        irq_set = self.VFIODeviceIRQSet(argsz=ctypes.sizeof(self.VFIODeviceIRQSet),
                                        flags=self.VFIODeviceIRQSetFlags["VFIO_IRQ_SET_ACTION_UNMASK"] |
                                              self.VFIODeviceIRQSetFlags["VFIO_IRQ_SET_DATA_NONE"],
                                        index=index, start=0, count=1)
        ioctl.ioctl(self.fd, self.VFIODeviceIoctls["VFIO_DEVICE_SET_IRQS"], ctypes.byref(irq_set))

    def trigger_irq(self, index):
        irq_set = self.VFIODeviceIRQSet(argsz=ctypes.sizeof(self.VFIODeviceIRQSet),
                                        flags=self.VFIODeviceIRQSetFlags["VFIO_IRQ_SET_ACTION_TRIGGER"] |
                                              self.VFIODeviceIRQSetFlags["VFIO_IRQ_SET_DATA_NONE"],
                                        index=index, start=0, count=1)
        ioctl.ioctl(self.fd, self.VFIODeviceIoctls["VFIO_DEVICE_SET_IRQS"], ctypes.byref(irq_set))

    def reset(self):
        ioctl.ioctl(self.fd, self.VFIODeviceIoctls["VFIO_DEVICE_RESET"])

    def region_read(self, index, offset, size):
        if self.regions[index].size == 0:
            raise Exception("Region not supported")
        val = os.pread(self.fd, size, self.regions[index].offset + offset)
        return int.from_bytes(val, byteorder='little', signed=False)

    def region_readb(self, index, offset):
        return self.region_read(index, offset, 1)

    def region_readw(self, index, offset):
        return self.region_read(index, offset, 2)

    def region_readl(self, index, offset):
        return self.region_read(index, offset, 4)

    def region_readq(self, index, offset):
        return self.region_read(index, offset, 8)

    def region_write(self, index, offset, size, val):
        if self.regions[index].size == 0:
            raise Exception("Region not supported")
        os.pwrite(self.fd, val.to_bytes(size, byteorder='little', signed=False),
                  self.regions[index].offset + offset)

    def region_writeb(self, index, offset, val):
        self.region_write(index, offset, 1, val)

    def region_writew(self, index, offset, val):
        self.region_write(index, offset, 2, val)

    def region_writel(self, index, offset, val):
        self.region_write(index, offset, 4, val)

    def region_writeq(self, index, offset, val):
        self.region_write(index, offset, 8, val)

    def __init__(self, group, device):
        self.group = group
        self.device = device
        self.fd = group.get_device(device)
        if self.fd is None:
            raise Exception("Failed to get device")
        self.info = self.VFIODeviceInfo(argsz=ctypes.sizeof(self.VFIODeviceInfo))
        if ioctl.ioctl(self.fd, self.VFIODeviceIoctls["VFIO_DEVICE_GET_INFO"], ctypes.byref(self.info)) != 0:
            self.fd.close()
            raise Exception("Failed to get device info")
        self.regions = []
        self.mmaps = []
        for i in range(self.info.num_regions):
            region = self.VFIODeviceRegionInfo(argsz=ctypes.sizeof(self.VFIODeviceRegionInfo), index=i)
            try:
                ioctl.ioctl(self.fd, self.VFIODeviceIoctls["VFIO_DEVICE_GET_REGION_INFO"], ctypes.byref(region))
            except OSError as e:
                if e.errno != 22:
                    raise e
            self.regions.append(region)
            self.mmaps.append(None)
        self.irqs = []
        for i in range(self.info.num_irqs):
            irq = self.VFIODeviceIRQInfo(argsz=ctypes.sizeof(self.VFIODeviceIRQInfo), index=i)
            try:
                ioctl.ioctl(self.fd, self.VFIODeviceIoctls["VFIO_DEVICE_GET_IRQ_INFO"], ctypes.byref(irq))
            except OSError as e:
                if e.errno != 22:
                    raise e
            self.irqs.append(irq)
