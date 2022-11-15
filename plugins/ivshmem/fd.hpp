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

#ifndef OPENRAVE_FD_HPP
#define OPENRAVE_FD_HPP

#include <unistd.h> // For close(2)

// RAII file descriptors that closes itself upon scope exit.
// This is useful to avoid file descriptor leaks.
// Currently, only file descriptors that close with `close(2)` is supported.
struct FileDescriptor final {
public:
    // Default constructor creates a FileDescriptor with an invalid value (-1)
    FileDescriptor() noexcept;

    // Transfers ownership of the file descriptor to this object.
    explicit FileDescriptor(int fd) noexcept;

    // Constructs the file descriptor with the given creation function and arguments.
    template <typename Ctor, typename... Args>
    FileDescriptor(Ctor&& ctor, Args&&... args) noexcept : _fd(ctor(std::forward<Args>(args)...)) {}

    FileDescriptor(const FileDescriptor&) = delete;
    FileDescriptor(FileDescriptor&&) noexcept;

    ~FileDescriptor() noexcept;

    int get() const noexcept {
        return _fd;
    }

    // Releases ownership of its file descriptor, returning it and sets its own fd to -1.
    int release() noexcept {
        int fd = _fd;
        _fd = -1;
        return fd;
    }

    // Closes the file descriptor immediately and sets its own fd to -1.
    void dispose() noexcept {
        if (_fd > 0) {
            ::close(_fd);
        }
        _fd = -1;
    }

    operator bool() const noexcept;
    bool operator==(const FileDescriptor&) const noexcept;
    bool operator==(int fd) const noexcept;
    FileDescriptor& operator=(const FileDescriptor&) = delete;
    FileDescriptor& operator=(FileDescriptor&&) noexcept;

private:
    int _fd;
};

inline FileDescriptor::FileDescriptor() noexcept : _fd(-1) {}

inline FileDescriptor::FileDescriptor(int fd) noexcept : _fd(fd) {}

inline FileDescriptor::FileDescriptor(FileDescriptor&& other) noexcept : _fd(std::move(other._fd)) {
    other._fd = -1;
}

inline FileDescriptor::~FileDescriptor() noexcept {
    if (_fd > -1) {
        ::close(_fd);
    }
}

inline FileDescriptor::operator bool() const noexcept {
    return (_fd > -1);
}

inline bool FileDescriptor::operator==(const FileDescriptor& other) const noexcept {
    return _fd == other._fd;
}

inline bool FileDescriptor::operator==(int fd) const noexcept {
    return _fd == fd;
}

inline FileDescriptor& FileDescriptor::operator=(FileDescriptor&& other) noexcept {
    _fd = other._fd;
    other._fd = -1;
    return *this;
}

#endif // OPENRAVE_FD_HPP