import ioctl
import ioctl.linux
from enum import Enum
import ctypes


class VFIOGroup:
    VFIOGroupIoctls = {
        "VFIO_GROUP_GET_STATUS": ioctl.linux.IO(';', 103),
        "VFIO_GROUP_SET_CONTAINER": ioctl.linux.IO(';', 104),
        "VFIO_GROUP_UNSET_CONTAINER": ioctl.linux.IO(';', 105),
        "VFIO_GROUP_GET_DEVICE_FD": ioctl.linux.IO(';', 106),
    }

    class VFIOGroupStatusFlags(Enum):
        VFIO_GROUP_FLAGS_VIABLE = 1
        VFIO_GROUP_FLAGS_CONTAINER_SET = 2

    class VFIOGroupStatus(ctypes.Structure):
        _fields_ = [
            ('argsz', ctypes.c_uint32),
            ('flags', ctypes.c_uint32),
        ]

    def __init__(self, group, container):
        self.container = container
        self.group = group
        self.fd = open("/dev/vfio/" + group, "rb")
        if self.fd.fileno() < 0:
            raise Exception("Failed to open group")
        status = self.VFIOGroupStatus(argsz=ctypes.sizeof(self.VFIOGroupStatus))
        if ioctl.ioctl(self.fd.fileno(), self.VFIOGroupIoctls["VFIO_GROUP_GET_STATUS"],
                       ctypes.byref(status)) != 0:
            self.fd.close()
            raise Exception("Failed to get group status")
        if status.flags & self.VFIOGroupStatusFlags["VFIO_GROUP_FLAGS_VIABLE"].value == 0:
            self.fd.close()
            raise Exception("Group not viable")
        if ioctl.ioctl(self.fd.fileno(), self.VFIOGroupIoctls["VFIO_GROUP_SET_CONTAINER"],
                       ctypes.byref(ctypes.c_int32(container.fd.fileno()))) != 0:
            self.fd.close()
            raise Exception("Failed to set container")
        container.attach_group(self)

    def get_device(self, device):
        name = device.encode('utf-8')
        device_fd = ioctl.ioctl(self.fd.fileno(), self.VFIOGroupIoctls["VFIO_GROUP_GET_DEVICE_FD"],
                                ctypes.c_char_p(name))
        if device_fd < 0:
            raise Exception("Failed to get device")

        return device_fd
