import ioctl
import ioctl.linux
import ctypes


class VFIOContainer:
    VFIOContainerIoctls = {
        "VFIO_GET_API_VERSION": ioctl.linux.IO(';', 100),
        "VFIO_CHECK_EXTENSION": ioctl.linux.IO(';', 101),
        "VFIO_SET_IOMMU": ioctl.linux.IO(';', 102),
        "VFIO_IOMMU_GET_INFO": ioctl.linux.IO(';', 112),
        "VFIO_IOMMU_MAP_DMA": ioctl.linux.IO(';', 113),
        "VFIO_IOMMU_UNMAP_DMA": ioctl.linux.IO(';', 114),
    }

    VFIOContainerExtensions = {
        "VFIO_TYPE1_IOMMU": 1,
        "VFIO_SPAPR_TCE_IOMMU": 2,
        "VFIO_TYPE1v2_IOMMU": 3,
        "VFIO_DMA_CC_IOMMU": 4,
        "VFIO_EEH": 5,
        "VFIO_TYPE1_NESTING_IOMMU": 6,
        "VFIO_SPAPR_TCE_v2_IOMMU": 7,
        "VFIO_NOIOMMU_IOMMU": 8,
        "VFIO_UNMAP_ALL": 9,
        "VFIO_UPDATE_VADDR": 10,
    }

    def update_extensions(self):
        self.extensions.clear()
        for name, value in self.VFIOContainerExtensions.items():
            if ioctl.ioctl(self.fd.fileno(), self.VFIOContainerIoctls["VFIO_CHECK_EXTENSION"], value):
                self.extensions.append(name)

    def set_iommu(self):
        if "VFIO_TYPE1v2_IOMMU" in self.extensions:
            ioctl.ioctl(self.fd.fileno(), self.VFIOContainerIoctls["VFIO_SET_IOMMU"],
                        self.VFIOContainerExtensions["VFIO_TYPE1v2_IOMMU"])
        elif "VFIO_TYPE1_IOMMU" in self.extensions:
            ioctl.ioctl(self.fd.fileno(), self.VFIOContainerIoctls["VFIO_SET_IOMMU"],
                        self.VFIOContainerExtensions["VFIO_TYPE1_IOMMU"])
        else:
            raise Exception("No IOMMU found")

    def attach_group(self, group):
        if len(self.groups) == 0:
            self.set_iommu()
        self.update_extensions()
        self.groups.append(group)

    class VFIOIOMMUType1DMAMap(ctypes.Structure):
        _fields_ = [
            ('argsz', ctypes.c_uint32),
            ('flags', ctypes.c_uint32),
            ('vaddr', ctypes.c_uint64),
            ('iova', ctypes.c_uint64),
            ('size', ctypes.c_uint64),
        ]

    VFIOIOMMUType1DMAMapFlags = {
        "VFIO_DMA_MAP_FLAG_READ": 1,
        "VFIO_DMA_MAP_FLAG_WRITE": 2,
    }

    def dma_map(self, vaddr, iova, size):
        dma_map = self.VFIOIOMMUType1DMAMap(argsz=ctypes.sizeof(self.VFIOIOMMUType1DMAMap),
                                            flags=self.VFIOIOMMUType1DMAMapFlags["VFIO_DMA_MAP_FLAG_READ"] |
                                                  self.VFIOIOMMUType1DMAMapFlags["VFIO_DMA_MAP_FLAG_WRITE"],
                                            vaddr=vaddr, iova=iova, size=size)
        if ioctl.ioctl(self.fd.fileno(), self.VFIOContainerIoctls["VFIO_IOMMU_MAP_DMA"], ctypes.byref(dma_map)) != 0:
            raise Exception("Failed to map DMA")

    class VFIOIOMMUType1DMAUnmap(ctypes.Structure):
        _fields_ = [
            ('argsz', ctypes.c_uint32),
            ('flags', ctypes.c_uint32),
            ('iova', ctypes.c_uint64),
            ('size', ctypes.c_uint64),
        ]

    def dma_unmap(self, iova, size):
        dma_unmap = self.VFIOIOMMUType1DMAMap(argsz=ctypes.sizeof(self.VFIOIOMMUType1DMAMap), iova=iova, size=size)
        if ioctl.ioctl(self.fd.fileno(), self.VFIOContainerIoctls["VFIO_IOMMU_UNMAP_DMA"],
                       ctypes.byref(dma_unmap)) != 0:
            raise Exception("Failed to unmap DMA")

    def __init__(self):
        self.fd = open("/dev/vfio/vfio", "rb")
        if self.fd.fileno() < 0:
            raise Exception("Failed to open container")
        self.groups = []
        self.extensions = []
        if ioctl.ioctl(self.fd.fileno(), self.VFIOContainerIoctls["VFIO_GET_API_VERSION"]) != 0:
            self.fd.close()
            raise Exception("Unsupported VFIO API version")
        self.update_extensions()
