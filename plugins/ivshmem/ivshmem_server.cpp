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

#include "ivshmem_server.hpp"

#ifndef IVSHMEM_PROTOCOL_VERSION
#define IVSHMEM_PROTOCOL_VERSION 0
#endif // IVSHMEM_PROTOCOL_VERSION

using namespace std::literals;

const std::string IVShMemServer::_sock_path = "/tmp/ivshmem_socket";

// Add a file descriptor to watch in epoll.
// Returns true if successful, false if -1 occurred.
static bool AddToEpoll(int fd, int epoll_fd, uint32_t flags = EPOLLIN | EPOLLOUT | EPOLLPRI | EPOLLERR | EPOLLHUP) noexcept {
    struct epoll_event ev;
    ev.events = flags;
    ev.data.fd = fd;
    return ::epoll_ctl(epoll_fd, EPOLL_CTL_ADD, ev.data.fd, &ev) != -1;
}

static bool AddToEpoll(const FileDescriptor& fd, const FileDescriptor& ep_fd, uint32_t flags = EPOLLIN | EPOLLOUT | EPOLLPRI | EPOLLERR | EPOLLHUP) noexcept {
    return AddToEpoll(fd.get(), ep_fd.get(), flags);
}

static bool RemoveFromEpoll(int fd, int ep_fd) {
    return ::epoll_ctl(ep_fd, EPOLL_CTL_DEL, fd, NULL) != -1;
}

static bool RemoveFromEpoll(const FileDescriptor& fd, const FileDescriptor& ep_fd) {
    return RemoveFromEpoll(fd.get(), ep_fd.get());
}

IVShMemServer::IVShMemServer(std::string shmem_path)
    : _stop(false)
    , _shmem_path(std::move(shmem_path))
    , _sock_fd() {
        RAVELOG_INFO("Starting other ivshmem server, built on %s", __TIMESTAMP__);
}

IVShMemServer::IVShMemServer(IVShMemServer&& other) noexcept
    : _sock_fd(std::move(other._sock_fd)) {
    _stop.store(other._stop.load());
    other._stop = true;
}

IVShMemServer::~IVShMemServer() {
    _stop = true;
}

void IVShMemServer::Thread() try {
    _InitSocket();

    /** Set up epoll **/
    static constexpr size_t MAX_EPOLL_EVENTS = 4;
    auto ep_fd = FileDescriptor(epoll_create, MAX_EPOLL_EVENTS);

    if (!AddToEpoll(_sock_fd, ep_fd, EPOLLIN | EPOLLPRI | EPOLLERR | EPOLLHUP)) {
        throw std::runtime_error("Failed to register epoll event: "s + strerror(errno));
    }

    // Add a signalfd to epoll as part of exit criteria.
    sigset_t mask;
    sigemptyset(&mask);
    sigaddset(&mask, SIGINT);
    auto sig_fd = FileDescriptor(signalfd, -1, &mask, 0);
    if (!AddToEpoll(sig_fd, ep_fd, EPOLLIN | EPOLLPRI | EPOLLERR | EPOLLHUP)) {
        throw std::runtime_error("Failed to register epoll event: "s + strerror(errno));
    }

    ::epoll_event events[MAX_EPOLL_EVENTS];
    // ivshmem only provides 16 bits of client ID.
    // Start from 1 because 0 is reserved.
    int16_t guest_id = 1;

    while (!_stop) {
        int num_fds = ::epoll_wait(ep_fd.get(), events, MAX_EPOLL_EVENTS, -1);
        if (num_fds <= 0) {
            throw std::runtime_error("Error caught in epoll_wait: "s + strerror(errno));
        }
        for (int i = 0; i < num_fds; ++i) {
            if (_stop) break;
            int fd = events[i].data.fd;
            // Signal event, it's time to stop
            if (fd == sig_fd.get()) {
                _stop = true;
                _cv.notify_all();
                RAVELOG_INFO("Stop signal caught.");
                break;
            }
            // Event on the socket fd, there's a new guest.
            if (fd == _sock_fd.get()) {
                _NewGuest(guest_id);
                guest_id++;
                if (!AddToEpoll(_peers.back().sock_fd, ep_fd)) {
                    throw std::runtime_error("Failed to register epoll event: "s + strerror(errno));
                }
                _cv.notify_one();
                continue;
            }
            // Any event from guest socket means the guest has exited.
            for (const IVShMemPeer& peer : _peers) {
                if (peer.sock_fd == fd) {
                    int msgdata = 0;
                    int msg = _ShMem_RecvMsg(fd, peer.id, msgdata);
                    if (msg == 0 && msgdata == -1) {
                        _RemoveGuest(peer.id);
                        RemoveFromEpoll(fd, ep_fd.get());
                    }
                    break;
                }
            }
        }
    }
} catch (const std::exception& e) {
    RAVELOG_ERROR("Exception caught: %s", e.what());
}

void IVShMemServer::InterruptPeer(int64_t peerid) const noexcept {
    static const uint64_t INTERRUPT_VALUE = htole64(1);
    for (auto& peer : _peers) {
        if (peer.id == peerid) {
            ssize_t written = ::write(peer.vectors[0].get(), &INTERRUPT_VALUE, sizeof(INTERRUPT_VALUE));
            if (written == -1) {
                RAVELOG_ERROR("Failed to interrupt peer %d: %s", peerid, strerror(errno));
            } else if (written < sizeof(INTERRUPT_VALUE)) {
                RAVELOG_WARN("Wrote fewer bytes than expected.");
            }
            if (written == sizeof(INTERRUPT_VALUE)) {
                RAVELOG_VERBOSE("Sent interrupt to peer %d", peerid);
            }
            break;
        }
    }
}

void IVShMemServer::_InitSocket() {
    _sock_fd = FileDescriptor(socket, AF_UNIX, SOCK_STREAM, 0);
    if (!_sock_fd) {
        throw std::runtime_error("Failed to create unix socket: "s + strerror(errno));
    }

    ::unlink(_sock_path.c_str());

    struct sockaddr_un local;
    local.sun_family = AF_UNIX;
    ::strncpy(local.sun_path, _sock_path.c_str(), sizeof(local.sun_path));
    if (::bind(_sock_fd.get(), reinterpret_cast<struct sockaddr*>(&local), sizeof(local)) == -1) {
        throw std::runtime_error("Failed to bind socket to address: "s + strerror(errno));
    }

    if (::listen(_sock_fd.get(), 4) == -1) {
        throw std::runtime_error("Failed to listen on socket: "s + strerror(errno));
    }
}

void IVShMemServer::_NewGuest(int16_t guest_id) {
    IVShMemPeer peer;
    peer.id = guest_id;

    struct sockaddr_un remote;
    socklen_t t = sizeof(remote);
    peer.sock_fd = FileDescriptor(::accept, _sock_fd.get(), reinterpret_cast<struct sockaddr*>(&remote), &t);
    if (!peer.sock_fd) {
        throw std::runtime_error("Failed to accept connection on socket: "s + strerror(errno));
    }

    int ret = 0;
    // ivshmem spec step 1: Send the protocol version number.
    ret = _ShMem_SendMsg(peer.sock_fd.get(), IVSHMEM_PROTOCOL_VERSION, -1);
    if (ret == -1) {
        RAVELOG_WARN("Failed to send msg: %s", strerror(errno));
    }

    // ivshmem spec step 2: Send the client's ID.
    ret = _ShMem_SendMsg(peer.sock_fd.get(), peer.id, -1);
    if (ret == -1) {
        RAVELOG_WARN("Failed to send msg: %s", strerror(errno));
    }

    // ivshmem spec step 3: send -1 followed by file descriptor of the shared memory.
    auto shmem_fd = FileDescriptor(shm_open, _shmem_path.c_str(), O_RDWR | O_CREAT, S_IRWXU);
    ret = _ShMem_SendMsg(peer.sock_fd.get(), -1, shmem_fd.get());
    if (ret == -1) {
        RAVELOG_WARN("Failed to send msg: %s", strerror(errno));
    }

    // init the eventfd for each interrupt vector
    for (int i = 0; i < IVSHMEM_VECTOR_COUNT; ++i) {
        peer.vectors[i] = FileDescriptor(::eventfd, 0, 0);
        if (!peer.vectors[i]) {
            RAVELOG_WARN("Failed to create eventfd: %s", strerror(errno));
        }
    }

    // ivshmem spec step 4: Advertise new peer to all peers.
    for (const auto& otherpeer : _peers) {
        for (int i = 0; i < peer.vectors.size(); ++i) {
            ret = _ShMem_SendMsg(otherpeer.sock_fd.get(), peer.id, peer.vectors[i].get());
            if (ret == -1) {
                RAVELOG_WARN("Failed to send msg: %s", strerror(errno));
                RAVELOG_WARN("Peer %d will be removed.", otherpeer.id);
                _RemoveGuest(otherpeer.id);
            }
        }
    }

    // Advertise all other peers to the new peer, excluding itself.
    for (const auto& otherpeer : _peers) {
        for (int i = 0; i < peer.vectors.size(); ++i) {
            ret = _ShMem_SendMsg(peer.sock_fd.get(), otherpeer.id, otherpeer.vectors[i].get());
            if (ret == -1) {
                RAVELOG_WARN("Failed to send msg: %s", strerror(errno));
            }
        }
    }

    // Advertise new peer to itself.
    for (int i = 0; i < peer.vectors.size(); ++i) {
        ret = _ShMem_SendMsg(peer.sock_fd.get(), peer.id, peer.vectors[i].get());
        if (ret == -1) {
            RAVELOG_WARN("Failed to send msg: %s", strerror(errno));
        }
    }

    RAVELOG_INFO("Added new peer ID %d", peer.id);
    _peers.emplace_back(std::move(peer));
}

void IVShMemServer::_RemoveGuest(int16_t guest_id) {
    auto guestiter = std::remove_if(_peers.begin(), _peers.end(), [guest_id](const IVShMemPeer& peer) {
        return peer.id == guest_id;
    });
    if (guestiter == _peers.end()) {
        return;
    }
    RAVELOG_INFO("Removing guest ID %d", guest_id);
    _peers.erase(guestiter);
}

int IVShMemServer::_ShMem_SendMsg(int sock_fd, int64_t peer_id, int message) noexcept {
    peer_id = htole64(peer_id);
    struct iovec iov = {
        .iov_base = &peer_id,
        .iov_len = sizeof(peer_id),
    };
    struct msghdr msg = {
        .msg_name = NULL,
        .msg_namelen = 0,
        .msg_iov = &iov,
        .msg_iovlen = 1,
        .msg_control = NULL,
        .msg_controllen = 0,
        .msg_flags = 0,
    };

    // If `message` is a value greater than 0, then add it as control content
    char control[CMSG_SPACE(sizeof(int))]; // Buffer must live outside the if block below
    if (message >= 0) {
        ::memset(control, 0, sizeof(control));
        msg.msg_control = control;
        msg.msg_controllen = sizeof(control);

        struct cmsghdr* cmsg = CMSG_FIRSTHDR(&msg);
        cmsg->cmsg_len = CMSG_LEN(sizeof(int));
        cmsg->cmsg_level = SOL_SOCKET;
        cmsg->cmsg_type = SCM_RIGHTS;
        ::memcpy(CMSG_DATA(cmsg), &message, sizeof(message));
    }

    return ::sendmsg(sock_fd, &msg, MSG_NOSIGNAL);
}

int IVShMemServer::_ShMem_RecvMsg(int sock_fd, int64_t peer_id, int& message) noexcept {
    char control[CMSG_SPACE(sizeof(message))];
    struct iovec iov = {
        .iov_base = &peer_id,
        .iov_len = sizeof(peer_id),
    };
    struct msghdr msg = {
        .msg_name = NULL,
        .msg_namelen = 0,
        .msg_iov = &iov,
        .msg_iovlen = 1,
        .msg_control = control,
        .msg_controllen = sizeof(control),
        .msg_flags = 0,
    };
    ssize_t len = 0;
    do {
        len = ::recvmsg(sock_fd, &msg, 0);
    } while (len == -1 && (errno == EINTR || errno == EAGAIN));
    if (len == -1) {
        RAVELOG_WARN("Error on recvmsg: %s", strerror(errno));
        return -1; // Some error on socket
    }
    // Any empty message from guest means a disconnect.
    else if (len == 0) {
        RAVELOG_WARN("Peer %d disconnected.", peer_id);
        message = -1;
        return 0;
    }

    // Search messages for content, though we do not expect guests to send anything
    for (struct cmsghdr* cmsg = CMSG_FIRSTHDR(&msg); cmsg != NULL; cmsg = CMSG_NXTHDR(&msg, cmsg)) {
        if (cmsg->cmsg_level != SOL_SOCKET || cmsg->cmsg_type != SCM_RIGHTS) {
            if (cmsg->cmsg_len != sizeof(control)) {
                continue;
            }
        }
        ::memcpy(&message, CMSG_DATA(cmsg), sizeof(message));
        message = le64toh(message);
        return 1;
    }

    return -1; // Nothing with content found, error
}
