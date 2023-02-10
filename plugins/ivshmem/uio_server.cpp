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

#include <openrave/config.h>
#include <openrave/logging.h>

#include "uio_server.hpp"

#ifndef IVSHMEM_PROTOCOL_VERSION
#define IVSHMEM_PROTOCOL_VERSION 0
#endif // IVSHMEM_PROTOCOL_VERSION

#define UIO_WRITE _IOW('u', 100, struct uio_irq_data)

using namespace std::literals;

// Add a file descriptor to watch in epoll.
// Returns true if successful, false if -1 occurred.
static bool AddToEpoll(int fd, int epoll_fd, void* data, uint32_t flags = EPOLLIN | EPOLLOUT | EPOLLPRI | EPOLLERR | EPOLLHUP) noexcept {
    struct epoll_event ev;
    ev.events = flags;
    ev.data.fd = fd;
    ev.data.ptr = data;
    return ::epoll_ctl(epoll_fd, EPOLL_CTL_ADD, ev.data.fd, &ev) != -1;
}

static bool AddToEpoll(const FileDescriptor& fd, const FileDescriptor& ep_fd, void* data, uint32_t flags = EPOLLIN | EPOLLOUT | EPOLLPRI | EPOLLERR | EPOLLHUP) noexcept {
    return AddToEpoll(fd.get(), ep_fd.get(), data, flags);
}

static bool RemoveFromEpoll(int fd, int ep_fd) {
    return ::epoll_ctl(ep_fd, EPOLL_CTL_DEL, fd, NULL) != -1;
}

static bool RemoveFromEpoll(const FileDescriptor& fd, const FileDescriptor& ep_fd) {
    return RemoveFromEpoll(fd.get(), ep_fd.get());
}

UIOServer::UIOServer(std::string uio_device_path)
    : _stop(false) {
        RAVELOG_INFO("Starting other ivshmem server, built on %s", __TIMESTAMP__);
}

UIOServer::UIOServer(UIOServer&& other) noexcept
    : _vpeer(std::move(other._vpeer)) {
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
    auto evt_fd = FileDescriptor(::eventfd, 0, 0);
    if (!evt_fd) {
        throw std::runtime_error("Failed to create eventfds: "s + strerror(errno));
    }

    /** Set up epoll **/
    static constexpr size_t MAX_EPOLL_EVENTS = 4;
    auto ep_fd = FileDescriptor(epoll_create, MAX_EPOLL_EVENTS);

    uio_irq_data data{0};
    data.fd = 0;
    data.vector = ep_fd.get();
    // Adding the UIO fd to epoll allows us to detect when new guests come online.
    if (!AddToEpoll(evt_fd, ep_fd, &data, EPOLLIN | EPOLLPRI | EPOLLERR | EPOLLRDHUP)) {
        throw std::runtime_error("Failed to register epoll event: "s + strerror(errno));
    }
    if (::ioctl(_uio_fd.get(), UIO_WRITE, &data) < 0) {
        throw std::runtime_error("Failed to ioctl!");
    }

    // Add virtual peer to allow us to handle signals coming from guests
    //if (!AddToEpoll(_vpeer.vectors[0], ep_fd, EPOLLIN | EPOLLPRI | EPOLLERR | EPOLLRDHUP)) {
    //    throw std::runtime_error("Failed to register _vpeer.vectors[1] with epoll event: "s + strerror(errno));
    //}
    //if (!AddToEpoll(_vpeer.vectors[1], ep_fd, EPOLLIN | EPOLLPRI | EPOLLERR | EPOLLRDHUP)) {
    //    throw std::runtime_error("Failed to register _vpeer.vectors[1] with epoll event: "s + strerror(errno));
    //}

    // Add a signalfd to epoll as part of exit criteria.
    sigset_t mask;
    sigemptyset(&mask);
    sigaddset(&mask, SIGINT);
    auto sig_fd = FileDescriptor(signalfd, -1, &mask, 0);
    if (!AddToEpoll(sig_fd, ep_fd, NULL, EPOLLIN | EPOLLPRI | EPOLLERR | EPOLLRDHUP)) {
        throw std::runtime_error("Failed to register epoll event: "s + strerror(errno));
    }

    ::epoll_event events[MAX_EPOLL_EVENTS];

    // ivshmem only provides 16 bits of client ID.
    // Start from 1 because 0 is reserved by the virtual peer.
    //int16_t guest_id = 1;

    while (!_stop) {
        int num_fds = ::epoll_wait(ep_fd.get(), events, MAX_EPOLL_EVENTS, -1);
        if (num_fds <= 0) {
            throw std::runtime_error("Error caught in epoll_wait: "s + strerror(errno));
        }
        for (int i = 0; (i < num_fds) && !_stop; ++i) {
            int fd = events[i].data.fd;
            // Signal event, it's time to stop
            if (fd == sig_fd.get()) {
                _OnStop();
                break;
            }
            if (fd == _uio_fd.get()) {
                eventfd_t event;
                ::eventfd_read(static_cast<uio_irq_data*>(events[i].data.ptr)->fd, &event);
                RAVELOG_INFO("[UIO] Received IRQ!");
                _irq_cv.notify_one();
            }
        }
    }
}

void UIOServer::_InitFDs() {
}

void UIOServer::_NewGuest(int16_t guest_id) {
    // TODO Not needed for now.
}

void UIOServer::_OnStop() {
    RAVELOG_INFO("Stop signal caught.");
    _stop.store(true);
}

void UIOServer::InterruptPeer() const {
    uint32_t* memptr = (uint32_t*)::mmap(NULL, IVSH_BAR0_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, _peer.fd, 0);
    if (memptr == MAP_FAILED) {
        throw std::runtime_error("Failed to map memory for interrupt.");
    }
    memptr[3] = (_peer.id << 16) | 0; // 0 is vector ID
    ::munmap(memptr, IVSH_BAR0_SIZE);
}
