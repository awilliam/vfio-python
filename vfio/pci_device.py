import os
import mmap
from vfio import VFIODevice


class VFIOPCIDevice(VFIODevice):
    VFIOPCIRegionIndex = {
        "VFIO_PCI_BAR0_REGION_INDEX": 0,
        "VFIO_PCI_BAR1_REGION_INDEX": 1,
        "VFIO_PCI_BAR2_REGION_INDEX": 2,
        "VFIO_PCI_BAR3_REGION_INDEX": 3,
        "VFIO_PCI_BAR4_REGION_INDEX": 4,
        "VFIO_PCI_BAR5_REGION_INDEX": 5,
        "VFIO_PCI_ROM_REGION_INDEX": 6,
        "VFIO_PCI_CONFIG_REGION_INDEX": 7,
    }

    VFIOPCIIRQIndex = {
        "VFIO_PCI_INTX_IRQ_INDEX": 0,
        "VFIO_PCI_MSI_IRQ_INDEX": 1,
        "VFIO_PCI_MSIX_IRQ_INDEX": 2,
    }

    def __init__(self, group, device):
        super().__init__(group, device)
        if self.info.flags & self.VFIODeviceInfoFlags["VFIO_DEVICE_FLAGS_PCI"] == 0:
            raise Exception("Not a PCI device")
        self.irq_type = -1
        self.unmask_irqfd = -1

    def set_pci_intx_irq(self, fd):
        self.set_irq(self.VFIOPCIIRQIndex["VFIO_PCI_INTX_IRQ_INDEX"], fd)
        self.irq_type = self.VFIOPCIIRQIndex["VFIO_PCI_INTX_IRQ_INDEX"]

    def set_pci_intx_unmask_fd(self):
        if self.irq_type != self.VFIOPCIIRQIndex["VFIO_PCI_INTX_IRQ_INDEX"]:
            raise Exception("Not INTx IRQ")
        fd = os.eventfd(0)
        self.set_unmask_fd(self.VFIOPCIIRQIndex["VFIO_PCI_INTX_IRQ_INDEX"], fd)
        self.unmask_irqfd = fd

    def disable_pci_intx_unmask_fd(self):
        if self.irq_type != self.VFIOPCIIRQIndex["VFIO_PCI_INTX_IRQ_INDEX"]:
            raise Exception("Not INTx IRQ")
        if self.unmask_irqfd < 0:
            raise Exception("No unmask fd")
        self.set_unmask_fd(self.VFIOPCIIRQIndex["VFIO_PCI_INTX_IRQ_INDEX"], -1)
        os.close(self.unmask_irqfd)
        self.unmask_irqfd = -1

    def set_pci_msi_irq(self, fd):
        self.set_irq(self.VFIOPCIIRQIndex["VFIO_PCI_MSI_IRQ_INDEX"], fd)
        self.irq_type = self.VFIOPCIIRQIndex["VFIO_PCI_MSI_IRQ_INDEX"]

    def set_pci_msix_irq(self, fd):
        self.set_irq(self.VFIOPCIIRQIndex["VFIO_PCI_MSIX_IRQ_INDEX"], fd)
        self.irq_type = self.VFIOPCIIRQIndex["VFIO_PCI_MSIX_IRQ_INDEX"]

    def disable_pci_irq(self):
        if self.irq_type == -1:
            raise Exception("No IRQ")
        self.disable_irq(self.irq_type)
        if self.unmask_irqfd >= 0:
            os.close(self.unmask_irqfd)
            self.unmask_irqfd = -1
        self.irq_type = -1

    def trigger_pci_intx_irq(self):
        if self.irq_type != self.VFIOPCIIRQIndex["VFIO_PCI_INTX_IRQ_INDEX"]:
            raise Exception("Not INTx IRQ")
        self.trigger_irq(self.VFIOPCIIRQIndex["VFIO_PCI_INTX_IRQ_INDEX"])

    def trigger_pci_msi_irq(self):
        if self.irq_type != self.VFIOPCIIRQIndex["VFIO_PCI_MSI_IRQ_INDEX"]:
            raise Exception("Not MSI IRQ")
        self.trigger_irq(self.VFIOPCIIRQIndex["VFIO_PCI_MSI_IRQ_INDEX"])

    def trigger_pci_msix_irq(self):
        if self.irq_type != self.VFIOPCIIRQIndex["VFIO_PCI_MSIX_IRQ_INDEX"]:
            raise Exception("Not MSI-X IRQ")
        self.trigger_irq(self.VFIOPCIIRQIndex["VFIO_PCI_MSIX_IRQ_INDEX"])

    def unmask_pci_intx(self):
        if self.irq_type != self.VFIOPCIIRQIndex["VFIO_PCI_INTX_IRQ_INDEX"]:
            raise Exception("Not INTx IRQ")
        if self.unmask_irqfd >= 0:
            os.eventfd_write(self.unmask_irqfd, 1)
        else:
            self.unmask_irq(self.VFIOPCIIRQIndex["VFIO_PCI_INTX_IRQ_INDEX"])

    def auto_unmask_pci_irq(self):
        if self.irq_type == self.VFIOPCIIRQIndex["VFIO_PCI_INTX_IRQ_INDEX"]:
            self.unmask_pci_intx()

    def mmap_bar(self, index):
        if self.regions[index].size == 0:
            raise Exception("Region not supported")

        if self.regions[index].flags & self.VFIORegionInfoFlags["VFIO_REGION_INFO_FLAG_MMAP"] == 0:
            raise Exception("Region not mappable")

        # XXX Not checking sparse mmap for compatibility
        self.mmaps[index] = mmap.mmap(self.fd, length=self.regions[index].size,
                                      flags=mmap.MAP_SHARED, prot=mmap.PROT_READ|mmap.PROT_WRITE,
                                      offset=self.regions[index].offset)
        return self.mmaps[index]

    PCIRegs = {
        "PCI_REG_COMMAND": 0x04,
    }

    PCICommandBits = {
        "PCI_COMMAND_IO": 0x1,
        "PCI_COMMAND_MEMORY": 0x2,
        "PCI_COMMAND_MASTER": 0x4,
    }

    def enable_mmio(self):
        command = self.region_readw(self.VFIOPCIRegionIndex["VFIO_PCI_CONFIG_REGION_INDEX"],
                                    self.PCIRegs["PCI_REG_COMMAND"])
        command |= self.PCICommandBits["PCI_COMMAND_MEMORY"]
        self.region_writew(self.VFIOPCIRegionIndex["VFIO_PCI_CONFIG_REGION_INDEX"],
                           self.PCIRegs["PCI_REG_COMMAND"], command)

    def enable_io(self):
        command = self.region_readw(self.VFIOPCIRegionIndex["VFIO_PCI_CONFIG_REGION_INDEX"],
                                    self.PCIRegs["PCI_REG_COMMAND"])
        command |= self.PCICommandBits["PCI_COMMAND_IO"]
        self.region_writew(self.VFIOPCIRegionIndex["VFIO_PCI_CONFIG_REGION_INDEX"],
                           self.PCIRegs["PCI_REG_COMMAND"], command)

    def enable_bus_master(self):
        command = self.region_readw(self.VFIOPCIRegionIndex["VFIO_PCI_CONFIG_REGION_INDEX"],
                                    self.PCIRegs["PCI_REG_COMMAND"])
        command |= self.PCICommandBits["PCI_COMMAND_MASTER"]
        self.region_writew(self.VFIOPCIRegionIndex["VFIO_PCI_CONFIG_REGION_INDEX"],
                           self.PCIRegs["PCI_REG_COMMAND"], command)
