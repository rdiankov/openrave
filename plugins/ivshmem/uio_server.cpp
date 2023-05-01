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

#include <errno.h>
#include <endian.h>
#include <fcntl.h>
#include <signal.h>
#include <sys/epoll.h>
#include <sys/eventfd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/signalfd.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/un.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cstdarg>
#include <functional>
#include <stdexcept>
#include <thread>

#include <openrave/config.h>
#include <openrave/logging.h>

#include "uio_server.hpp"

#ifndef IVSHMEM_PROTOCOL_VERSION
#define IVSHMEM_PROTOCOL_VERSION 0
#endif // IVSHMEM_PROTOCOL_VERSION

#define UIO_WRITE _IOW('u', 100, struct uio_irq_data)

using namespace std::literals;

UIOServer::UIOServer(std::string uio_device_path)
    : _stop(false) {
    RAVELOG_INFO("UIO server, built on %s\n", __TIMESTAMP__);
    _peer.fd = FileDescriptor(::open, UIO_FILE_PATH_0, O_RDWR);
    _peer.id = 0; // Hard-coded, rtlinux is VM 1 and peer Zephyr is VM 0.
}

UIOServer::UIOServer(UIOServer&& other) noexcept
    : _peer(std::move(other._peer)) {
    _stop.store(other._stop.load());
    other._stop = true;
}

UIOServer::~UIOServer() {
    _stop = true;
}

void UIOServer::Thread() {
    _uio_fd = FileDescriptor(::open, UIO_DEV_NAME, O_RDWR);
    if (!_uio_fd) {
        throw std::runtime_error("Failed to create fd for "s +  UIO_DEV_NAME + ": "s + strerror(errno));
    }

    /** Set up epoll **/
    auto ep_fd = FileDescriptor(::epoll_create, 1);
    if (!ep_fd) {
        throw std::runtime_error("Failed to create epoll fd: "s + strerror(errno));
    }

    auto evt_fd = FileDescriptor(::eventfd, 0, 0);
    if (!evt_fd) {
        throw std::runtime_error("Failed to create eventfds: "s + strerror(errno));
    }

    uio_irq_data irq_data{0};
    irq_data.fd = evt_fd.get();
    irq_data.vector = 0;
    struct epoll_event irq_ev{0};
    irq_ev.events = EPOLLIN;
    irq_ev.data.ptr = &irq_data;
    if (::epoll_ctl(ep_fd.get(), EPOLL_CTL_ADD, evt_fd.get(), &irq_ev) == -1) {
        throw std::runtime_error("Failed to register epoll for eventfd: "s + strerror(errno));
    }

    // Trigger an irq event if it is written to.
    if (::ioctl(_uio_fd.get(), UIO_WRITE, &irq_data) < 0) {
        throw std::runtime_error("Failed to ioctl!");
    }

    // Add a signalfd to epoll as part of exit criteria.
    sigset_t mask;
    sigemptyset(&mask);
    sigaddset(&mask, SIGINT);
    auto sig_fd = FileDescriptor(signalfd, -1, &mask, 0);
    struct epoll_event sig_ev{0};
    sig_ev.events = EPOLLIN | EPOLLPRI | EPOLLERR | EPOLLRDHUP;
    sig_ev.data.fd = sig_fd.get();
    if (::epoll_ctl(ep_fd.get(), EPOLL_CTL_ADD, sig_fd.get(), &sig_ev) == -1) {
        throw std::runtime_error("Failed to register epoll for signalfd: "s + strerror(errno));
    }

    ::epoll_event epevent{0};
    while (!_stop) {
        int num_fds = ::epoll_wait(ep_fd.get(), &epevent, 1, 500);
        if (num_fds == -1) {
            throw std::runtime_error("Error caught in epoll_wait: "s + strerror(errno));
        }
        // Signal event, it's time to stop
        if (epevent.data.fd == sig_fd.get()) {
            RAVELOG_INFO("Stop signal caught.\n");
            Stop();
            break;
        }

        // IRQ Event
        if (epevent.data.ptr == &irq_data) {
            eventfd_t event;
            uio_irq_data* irq_ptr = static_cast<uio_irq_data*>(epevent.data.ptr);
            if (::eventfd_read(irq_ptr->fd, &event) == -1) {
                RAVELOG_ERROR("eventfd_read failed to read the correct number of bytes.\n");
                continue;
            }
            _sem.release();
        }
    }
}

void UIOServer::Stop() {
    _stop = true;
}

void UIOServer::InterruptPeer() const {
    constexpr uint16_t vector = 0;
    uint32_t* memptr = (uint32_t*)::mmap(NULL, IVSH_BAR0_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, _peer.fd.get(), 0);
    if (memptr == MAP_FAILED) {
        ::munmap(memptr, IVSH_BAR0_SIZE);
        throw std::runtime_error("Failed to map memory for interrupt.");
    }
    memptr[3] = (static_cast<uint32_t>(_peer.id) << 16) | vector;
    ::munmap(memptr, IVSH_BAR0_SIZE);
}
