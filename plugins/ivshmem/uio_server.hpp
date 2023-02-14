// -*- coding: utf-8 -*-
// Copyright (C) 2022 Tan Li Boon (liboon.tan@mujin.co.jp)
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#ifndef OPENRAVE_UIO_SERVER_HPP
#define OPENRAVE_UIO_SERVER_HPP

#include <array>
#include <atomic>
#include <condition_variable>
#include <string>
#include <vector>

#include "fd.hpp"
#include "semaphore.hpp"

struct uio_irq_data {
	int fd;
	int vector;
};

class UIOServer final {
public:

    // For more details on the origin of these paths, see
    // https://projectacrn.github.io/latest/tutorials/inter-vm_communication.html
    // and
    // https://github.com/qemu/qemu/blob/master/docs/specs/ivshmem-spec.txt

    // BAR0 holds device registers (256 Byte MMIO)
    // To signal an interrupt register, memory-map the space and write to the appropriate offset.
    static constexpr const char* UIO_FILE_PATH_0 = "/sys/class/uio/uio0/device/resource0";

    // BAR 0 contains the following registers, interpreted as an array of 32-bit integers:
    // uint32_t[0] - R/W Interrupt Mask
    //         [1] - R/W Interrupt Status
    //         [2] - RO  IVPosition
    //         [3] - WO  Doorbell
    //         [4+] - Reserved
    // In the ACRN documentation, the offset to get the Doorbell is written as (IVSHMEM_DOORBELL_REG >> 2)
    // The value written to the register is a combination of 16-bit shifted peer-ID ORed with the ID of the desired vector.
    // https://projectacrn.github.io/latest/developer-guides/hld/ivshmem-hld.html
    static constexpr size_t      IVSH_BAR0_SIZE  = 256;
    static constexpr uint32_t    IVSHMEM_IRQ_MASK_REG = 0x0;
    static constexpr uint32_t    IVSHMEM_IRQ_STA_REG  = 0x4;
    static constexpr uint32_t    IVSHMEM_IV_POS_REG   = 0x8;
    static constexpr uint32_t    IVSHMEM_DOORBELL_REG = 0xC;

    // BAR1 holds MSI-X table and PBA (only ivshmem-doorbell)
    // This is not used at the moment.
    //static constexpr const char* UIO_FILE_PATH_1 = "/sys/class/uio/uio0/device/resource1";

    // BAR2 is used for exposing a shared memory region
    // 'wc' means write-combined, to allow for both reading and writing.
    static constexpr const char* UIO_FILE_PATH_2 = "/sys/class/uio/uio0/device/resource2_wc";

    // Used for doorbell mode
    // Need to perform an ioctl call in order to set event FDs of MSIX to the kernel driver.
    static constexpr const char* UIO_DEV_NAME    = "/dev/uio0";

    enum {
        UIO_SUCCESS = 0x00,
        UIO_CONFIG_OPEN_FAIL = 0xF1,
        UIO_FILE_OPEN_FAIL = 0xF2,
        UIO_MMAP_FAIL = 0xF3,
        UIO_INVALID_CONTENT = 0xF4,
        UIO_FAIL = 0xFF,
    };

    UIOServer(std::string uio_device_path = UIO_DEV_NAME);
    UIOServer(const UIOServer&) = delete;
    UIOServer(UIOServer&&) noexcept;
    ~UIOServer();

    void Thread();

    // AKA trigger doorbell
    void InterruptPeer() const;

    inline void WaitVector() {
        _sem.acquire();
    }

    void Stop();

private:
    std::atomic_bool _stop;

    FileDescriptor _uio_fd;

    static constexpr int IVSHMEM_VECTOR_COUNT = 2;
    struct IVShMemPeer {
    public:
        int16_t id; // ivshmem only provides 16 bits of client ID.
        FileDescriptor fd; // To receive data from peers when there is an event
    };
    IVShMemPeer _peer;

    std::binary_semaphore _sem;
};

#endif // OPENRAVE_UIO_SERVER_HPP
