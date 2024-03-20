import ctypes
import select
import subprocess
import os
import mmap
import time
import threading
from vfio.container import VFIOContainer
from vfio.group import VFIOGroup
from vfio.pci_device import VFIOPCIDevice


class VFIOPCINVMeDevice(VFIOPCIDevice):
    NVMERegs = {
        "NVME_REG_CAP": 0x00,
        "NVME_CC": 0x14,
        "NVME_CSTS": 0x1c,
        "NVME_AQA": 0x24,
        "NVME_ASQ": 0x28,
        "NVME_ACQ": 0x30,
        "NVME_REG_DBS": 0x1000,
    }

    class NVMEPhysRegs(ctypes.Structure):
        _fields_ = [
            ('cap', ctypes.c_uint64),
            ('blah', ctypes.c_uint8 * (0x1000 - 8)),
            ('tail', ctypes.c_uint32),
            ('head', ctypes.c_uint32),
        ]

    NVMEPhysRegsStride = 4 # Implicit in the above structure, not sure how to access with a variable stride

    class NVMECommonCommand(ctypes.Structure):
        _fields_ = [
            ('opcode', ctypes.c_uint8),
            ('resv1', ctypes.c_uint8),
            ('command_id', ctypes.c_uint16),
            ('fctype', ctypes.c_uint8),
            ('resv2', ctypes.c_uint8 * 35),
            ('ts', ctypes.c_uint8 * 24),
        ]

    class NVMEResult(ctypes.Union):
        _fields_ = [
            ('u16', ctypes.c_uint16),
            ('u32', ctypes.c_uint32),
            ('u64', ctypes.c_uint64)
        ]

    class NVMECompletion(ctypes.Structure):
        _fields_ = [
            ('result', ctypes.c_uint64),  # Should be VFIOPCINVMeDevice.NVMEResult but doesn't work
            ('sq_head', ctypes.c_uint16),
            ('sq_id', ctypes.c_uint16),
            ('command_id', ctypes.c_uint16),
            ('status', ctypes.c_uint16),
        ]

    def wait_ready(self, enable):
        start_time = time.time()
        end_time = start_time + (float(self.timeout_ms) / 1000)
        while True:
            if bool(self.region_readl(self.VFIOPCIRegionIndex["VFIO_PCI_BAR0_REGION_INDEX"],
                                      self.NVMERegs["NVME_CSTS"]) & 1) == enable:
                return
            time.sleep(0.1)
            if time.time() > end_time:
                raise Exception("Timeout")

    def get_stride(self):
        return 1 << (2 + ((self.capabilities >> 32) & 0xf))

    def get_timeout(self):
        return ((self.capabilities >> 24) & 0xff) * 500

    def set_aqa(self):
        aqa = self.aq_depth - 1
        aqa = (aqa << 16) | aqa
        self.region_writeq(self.VFIOPCIRegionIndex["VFIO_PCI_BAR0_REGION_INDEX"], self.NVMERegs["NVME_AQA"], aqa)

    def write_tail(self, tail, with_mmap=False):
        if with_mmap:
            self.regs[0].tail = tail
        else:
            self.region_writel(self.VFIOPCIRegionIndex["VFIO_PCI_BAR0_REGION_INDEX"], self.NVMERegs["NVME_REG_DBS"], self.tail)

    def write_head(self, head, with_mmap=False):
        if with_mmap:
            self.regs[0].head = head
        else:
             self.region_writel(self.VFIOPCIRegionIndex["VFIO_PCI_BAR0_REGION_INDEX"], self.NVMERegs["NVME_REG_DBS"] + self.stride, self.head)

    def read_cap(self, with_mmap=False):
        if with_mmap:
            return self.regs[0].cap
        else:
            return self.region_readq(self.VFIOPCIRegionIndex["VFIO_PCI_BAR0_REGION_INDEX"], self.NVMERegs["NVME_REG_CAP"])

    def submit_command(self, with_mmap=False):
        self.tail += 1
        if self.tail == self.aq_depth:
            self.tail = 0
        self.write_tail(self.tail, with_mmap)
        self.read_cap(with_mmap)

    def irq_thread(self, eventfd, with_mmap=False):
        poll = select.poll()
        poll.register(eventfd, select.POLLIN)
        while True:
            events = poll.poll()

            for fd, event in events:
                if event & select.POLLIN:
                    self.irq_count += 1
                    os.eventfd_read(fd)
                    self.head += 1
                    if self.head == self.aq_depth:
                        self.head = 0
                    self.write_head(self.head, with_mmap)
                    self.auto_unmask_pci_irq()
                    if self.irq_thread_exit:
                        return
                    self.submit_command()

    def roundup_page_size(self, size):
        page_size = os.sysconf('SC_PAGESIZE')
        page_mask = page_size - 1
        return (size + page_mask) & ~page_mask

    def irq_index_test(self, index, test_time, with_unmask_irqfd=False, with_mmap=False):
        if index == self.VFIOPCIIRQIndex["VFIO_PCI_INTX_IRQ_INDEX"]:
            name = "INTx"
        elif index == self.VFIOPCIIRQIndex["VFIO_PCI_MSI_IRQ_INDEX"]:
            name = "MSI"
        elif index == self.VFIOPCIIRQIndex["VFIO_PCI_MSIX_IRQ_INDEX"]:
            name = "MSI-X"

        if with_unmask_irqfd:
            name += " with unmask fd"

        if with_mmap:
            name += " with mmap"

        # print("Testing " + name + "...")
        self.irq_fd = os.eventfd(0)

        self.region_writel(self.VFIOPCIRegionIndex["VFIO_PCI_BAR0_REGION_INDEX"], self.NVMERegs["NVME_CC"],
                           6 << 16 | 4 << 20 | 1)
        self.wait_ready(True)
        self.irq_thread_exit = False
        self.irq_count = 0

        if index == self.VFIOPCIIRQIndex["VFIO_PCI_INTX_IRQ_INDEX"]:
            self.set_pci_intx_irq(self.irq_fd)
            if with_unmask_irqfd:
                self.set_pci_intx_unmask_fd()
        elif index == self.VFIOPCIIRQIndex["VFIO_PCI_MSI_IRQ_INDEX"]:
            self.set_pci_msi_irq(self.irq_fd)
        elif index == self.VFIOPCIIRQIndex["VFIO_PCI_MSIX_IRQ_INDEX"]:
            self.set_pci_msix_irq(self.irq_fd)

        self.thread = threading.Thread(target=self.irq_thread, args=(self.irq_fd, with_mmap))
        self.thread.start()
        self.submit_command(with_mmap)

        time.sleep(test_time)
        self.irq_thread_exit = True

        self.thread.join(1)
        if self.thread.is_alive():
            print("Kicking stuck interrupt")
            if index == self.VFIOPCIIRQIndex["VFIO_PCI_INTX_IRQ_INDEX"]:
                self.trigger_pci_intx_irq()
            elif index == self.VFIOPCIIRQIndex["VFIO_PCI_MSI_IRQ_INDEX"]:
                self.trigger_pci_msi_irq()
            elif index == self.VFIOPCIIRQIndex["VFIO_PCI_MSIX_IRQ_INDEX"]:
                self.trigger_pci_msix_irq()
            self.thread.join()
            self.irq_count -= 1

        if index == self.VFIOPCIIRQIndex["VFIO_PCI_INTX_IRQ_INDEX"]:
            self.disable_pci_intx_unmask_fd
        self.disable_pci_irq()
        os.close(self.irq_fd)
        rate = round(float(self.irq_count) / test_time)
        self.region_writel(self.VFIOPCIRegionIndex["VFIO_PCI_BAR0_REGION_INDEX"], self.NVMERegs["NVME_CC"], 0)
        self.wait_ready(False)

        return name, rate

    def irq_index_test_best_of(self, index, test_time, count, with_unmask_irqfd=False, with_mmap=False):
        best_rate = 0
        for i in range(count):
            name, rate = self.irq_index_test(index, test_time, with_unmask_irqfd, with_mmap)
            if rate > best_rate:
                best_rate = rate

        print(name + ": " + str(best_rate) + " interrupts/second")
        return name, best_rate

    def timing_test(self, with_mmap=False):
        count=100000
        start = time.time()

        for i in range(count):
            self.write_head(0, with_mmap)
            self.write_tail(0, with_mmap)
            self.read_cap(with_mmap)

        end = time.time()

        return count / (end - start)

    def irq_test(self, test_time=2, best_of=3):
        self.sq_size = self.roundup_page_size(ctypes.sizeof(self.NVMECommonCommand) * self.aq_depth)
        self.sq_buf = mmap.mmap(-1, self.sq_size, mmap.MAP_SHARED, mmap.PROT_READ | mmap.PROT_WRITE)
        self.sq_type = ctypes.ARRAY(self.NVMECommonCommand, self.aq_depth)
        self.sq = self.sq_type.from_buffer(self.sq_buf)
        for i in range(self.aq_depth):
            self.sq[i].opcode = 0x3f  # NOP
        self.sq_addr = ctypes.addressof(self.sq)
        self.sq_iova = 1 << 20
        self.group.container.dma_map(self.sq_addr, self.sq_iova, self.sq_size)
        self.region_writeq(self.VFIOPCIRegionIndex["VFIO_PCI_BAR0_REGION_INDEX"],
                           self.NVMERegs["NVME_ASQ"], self.sq_iova)

        self.cq_size = self.roundup_page_size(ctypes.sizeof(self.NVMECompletion) * self.aq_depth)
        self.cq_buf = mmap.mmap(-1, self.cq_size, mmap.MAP_SHARED, mmap.PROT_READ | mmap.PROT_WRITE)
        self.cq_type = ctypes.ARRAY(self.NVMECompletion, self.aq_depth)
        self.cq = self.cq_type.from_buffer(self.cq_buf)
        self.cq_addr = ctypes.addressof(self.cq)
        self.cq_iova = self.sq_iova + self.sq_size
        self.group.container.dma_map(self.cq_addr, self.cq_iova, self.cq_size)
        self.region_writeq(self.VFIOPCIRegionIndex["VFIO_PCI_BAR0_REGION_INDEX"],
                           self.NVMERegs["NVME_ACQ"], self.cq_iova)

        results = []
        results += "device: ", self.device

        if self.stride == self.NVMEPhysRegsStride:
            map = self.mmap_bar(self.VFIOPCIRegionIndex["VFIO_PCI_BAR0_REGION_INDEX"])
            regs_type = ctypes.ARRAY(self.NVMEPhysRegs, 1)
            self.regs = regs_type.from_buffer(map)
            pread_time = self.timing_test()
            mmap_time = self.timing_test(with_mmap=True)
            print("mmap vs pread: " + str(round(pread_time/mmap_time * 100)) + "%")
            results += "mmap vs pread", round(pread_time/mmap_time * 100)
            with_mmap = True
        else:
            print("NVMe device stride (" + str(self.stride) + ") is not compatible with current ctypes layout - skipping mmap tests")
            with_mmap = False

        if self.irqs[self.VFIOPCIIRQIndex["VFIO_PCI_INTX_IRQ_INDEX"]].count > 0:
            results += self.irq_index_test_best_of(self.VFIOPCIIRQIndex["VFIO_PCI_INTX_IRQ_INDEX"],
                                                   test_time, best_of)
            results += self.irq_index_test_best_of(self.VFIOPCIIRQIndex["VFIO_PCI_INTX_IRQ_INDEX"],
                                                   test_time, best_of, with_unmask_irqfd=True)
            if with_mmap:
                results += self.irq_index_test_best_of(self.VFIOPCIIRQIndex["VFIO_PCI_INTX_IRQ_INDEX"],
                                                       test_time, best_of, with_mmap=with_mmap)
                results += self.irq_index_test_best_of(self.VFIOPCIIRQIndex["VFIO_PCI_INTX_IRQ_INDEX"],
                                                       test_time, best_of, with_unmask_irqfd=True,
                                                       with_mmap=with_mmap)
        else:
            print("No INTx IRQ")

        if self.irqs[self.VFIOPCIIRQIndex["VFIO_PCI_MSI_IRQ_INDEX"]].count > 0:
            results += self.irq_index_test_best_of(self.VFIOPCIIRQIndex["VFIO_PCI_MSI_IRQ_INDEX"],
                                                   test_time, best_of)
            if with_mmap:
                results += self.irq_index_test_best_of(self.VFIOPCIIRQIndex["VFIO_PCI_MSI_IRQ_INDEX"],
                                                       test_time, best_of, with_mmap=with_mmap)
        else:
            print("No MSI IRQ")

        if self.irqs[self.VFIOPCIIRQIndex["VFIO_PCI_MSIX_IRQ_INDEX"]].count > 0:
            results += self.irq_index_test_best_of(self.VFIOPCIIRQIndex["VFIO_PCI_MSIX_IRQ_INDEX"],
                                                   test_time, best_of)
            if with_mmap:
                results += self.irq_index_test_best_of(self.VFIOPCIIRQIndex["VFIO_PCI_MSIX_IRQ_INDEX"],
                                                       test_time, best_of, with_mmap=with_mmap)
        else:
            print("No MSI-X IRQ")

        print(results)

    def __init__(self, group, device):
        super().__init__(group, device)
        if self.regions[self.VFIOPCIRegionIndex["VFIO_PCI_BAR0_REGION_INDEX"]].size == 0:
            raise Exception("No BAR0 region")
        self.head = 0
        self.tail = 0
        self.aq_depth = 32
        self.enable_mmio()
        self.enable_bus_master()
        self.capabilities = self.region_readq(self.VFIOPCIRegionIndex["VFIO_PCI_BAR0_REGION_INDEX"],
                                              self.NVMERegs["NVME_REG_CAP"])
        self.stride = self.get_stride()
        self.timeout_ms = self.get_timeout()
        self.set_aqa()


result = subprocess.run(["lspci", "-Dd", "::0108"], capture_output=True, text=True)
devices = result.stdout.splitlines()
for device in devices:
    if os.path.exists("/sys/bus/pci/devices/" + device.split()[0] + "/vfio-dev"):
        device_name = device.split()[0]
        break
try:
    print("Found vfio NVMe device: " + device_name)
except NameError:
    print("No vfio NVMe device found")
    exit(1)

group_num = os.readlink("/sys/bus/pci/devices/" + device_name + "/iommu_group").split("/")[-1]
container = VFIOContainer()
group = VFIOGroup(group_num, container)
device = VFIOPCINVMeDevice(group, device_name)
device.irq_test()
