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

#ifndef OPENRAVE_IVSHMEM_SERVER_HPP
#define OPENRAVE_IVSHMEM_SERVER_HPP

#include <array>
#include <atomic>
#include <condition_variable>
#include <string>
#include <vector>

#include "fd.hpp"

class IVShMemServer final {
public:
    IVShMemServer(std::string shmem_path = "ivshmem");
    IVShMemServer(const IVShMemServer&) = delete;
    IVShMemServer(IVShMemServer&&) noexcept;
    ~IVShMemServer();

    /// \brief Main thread loop function. The loop should be run in the class containing this class.
    void Thread();

    void Stop() noexcept { _stop = true; }

    int NumPeers() const noexcept { return _peers.size(); }

    // For the purposes of this demo we will assume that there is only one peer.
    // It is highly likely that we will not need to handle more than 1 peer in production either, due to the design of this system.
    int64_t PeerID() const noexcept {
        return _peers[0].id;
    }

    // Set the callback fn when a new peer is registered.
    // Function signature: peer_id, pointer to memory block, size of memory available.
    // Function should return: number of bytes written, and -1 if there was an error.
    // If the return value is > 0, an interrupt signal is also sent to the peer.
    void OnNewPeer(std::function<size_t(int16_t, uint8_t*, size_t)> doOnNewPeer) {
        _onNewPeer = std::move(doOnNewPeer);
    }

    // Overload of OnNewPeer to unset any existing callback function.
    void OnNewPeer() {
        _onNewPeer = default_on_new_peer_fn;
    }

    void InterruptPeer(int64_t peerid) const noexcept;

    // Convenience overload: Assume there is only one peer.
    void InterruptPeer() const noexcept {
        InterruptPeer(_peers[0].id);
    }

    // Need to wait until at least one collision system comes online.
    void Wait(std::mutex& mutex) {
        std::unique_lock<std::mutex> lock(mutex);
        std::condition_variable cv;
        cv.wait(lock, [this]() {
            return (!_peers.empty()) || _stop.load();
        });
    }

    /// This object is in an invalid state if:
    /// - Shared memory file descriptor is invalid(-1), or
    /// - Interrupt socket is invalid(-1), or
    /// - It has been stopped.
    operator bool() const noexcept {
        return !(_stop || _sock_fd);
    }

private:
    // Initializes the socket that listens for interrupts.
    // Throws exceptions if the socket can't be bound to a path.
    void _InitSocket();

    // Add a new guest to the peers list
    void _NewGuest(int64_t guest_id);

    // Remove a guest from the peers list
    int _RemoveGuest(int64_t guest_id);

    // Handle a signal from a guest, including exit signals (which will result in _RemoveGuest)
    void _HandleGuest(int64_t guest_id);

    /// \brief Sends a signal to the peer. `message` must be a positive value if it is a control signal; otherwise it is ignored.
    /// Usually this message is the fd of the shared memory.
    /// If peer_id is -1, then message is broadcast.
    /// Returns the result of ::sendmsg.
    static int _ShMem_SendMsg(int sock_fd, int64_t peer_id, int message) noexcept;

    static int _ShMem_RecvMsg(int sock_fd, int64_t peer_id, int& message) noexcept;

private:
    std::atomic_bool _stop; // Signals the thread to stop.

    std::string _shmem_path;

    // Socket for interrupts, only when emulating.
    static const std::string _sock_path;
    FileDescriptor _sock_fd;

    static constexpr int IVSHMEM_VECTOR_COUNT = 2;
    struct IVShMemPeer final {
        FileDescriptor sock_fd;
        int64_t id;
        std::array<FileDescriptor, IVSHMEM_VECTOR_COUNT> vectors;
    };
    std::vector<IVShMemPeer> _peers;

    std::function<size_t(int16_t, uint8_t*, size_t)> _onNewPeer = default_on_new_peer_fn;

    static size_t default_on_new_peer_fn(int16_t, uint8_t*, size_t) {
        return 0;
    }
};

#endif // OPENRAVE_IVSHMEM_SERVER_HPP
