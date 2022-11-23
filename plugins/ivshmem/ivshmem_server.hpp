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

    void InterruptPeer(int64_t peerid) const noexcept;

    // Convenience overload: Assume there is only one peer.
    void InterruptPeer() const noexcept {
        InterruptPeer(_peers[0].id);
    }

    // Block calling thread until some condition becomes true.
    template <typename Pred>
    void Wait(Pred&& predicate) {
        std::unique_lock<std::mutex> lock(_mtx);
        _cv.wait(lock, [this, pred = std::move(predicate)]() {
            return pred(*this) || _stop.load();
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
    void _NewGuest(int16_t guest_id);

    // Remove a guest from the peers list
    void _RemoveGuest(int16_t guest_id);

    /// \brief Sends a signal to the peer. `message` must be a positive value if it is a control signal; otherwise it is ignored.
    /// Usually this message is the fd of the shared memory.
    /// If peer_id is -1, then message is broadcast.
    /// Returns the result of ::sendmsg.
    // NOTE: peer_id is widened on purpose from 16 bits to 64 bits.
    static int _ShMem_SendMsg(int sock_fd, int64_t peer_id, int  message) noexcept;
    static int _ShMem_RecvMsg(int sock_fd, int64_t peer_id, int& message) noexcept;

private:
    std::atomic_bool _stop; // Signals the thread to stop.

    std::string _shmem_path;

    // Socket for interrupts, only when emulating.
    static const std::string _sock_path;
    FileDescriptor _sock_fd;

    static constexpr int IVSHMEM_VECTOR_COUNT = 2;
    struct IVShMemPeer final {
        int16_t id; // ivshmem only provides 16 bits of client ID.
        FileDescriptor sock_fd;
        std::array<FileDescriptor, IVSHMEM_VECTOR_COUNT> vectors;
    };
    std::vector<IVShMemPeer> _peers;
    IVShMemPeer _vpeer;
    FileDescriptor _vpeer_sock_fd;

    std::mutex _mtx;
    std::condition_variable _cv;
};

#endif // OPENRAVE_IVSHMEM_SERVER_HPP
