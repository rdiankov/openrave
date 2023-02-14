#ifndef OPENRAVE_TRIPLE_BUFFERED_SHARED_MEMORY_HPP
#define OPENRAVE_TRIPLE_BUFFERED_SHARED_MEMORY_HPP

#include <atomic>
#include <cstdint>
#include <string>

#include "fd.hpp"

// Write-only
class TripleBufferedSharedMemory {
public:
    // \param size Size of the buffer region, in bytes. This class will reserve four times as much buffer.
    TripleBufferedSharedMemory(std::size_t size, std::string shmpath = "ivshmem");
    TripleBufferedSharedMemory(const TripleBufferedSharedMemory&) = delete;
    TripleBufferedSharedMemory(TripleBufferedSharedMemory&&) noexcept;
    ~TripleBufferedSharedMemory() noexcept;
    
    TripleBufferedSharedMemory& operator=(TripleBufferedSharedMemory&&) = default;
    operator bool() const noexcept { return !!_mmap; }

    // Finish writing the current buffer, submitting it for swapping.
    void write_ready() noexcept;
    uintptr_t get_writable() const noexcept;
    std::size_t writable_size() const noexcept { return _writesize; }

private:
    std::size_t _totalsize;
    std::size_t _writesize;
    std::string _shmpath;
    void* _mmap;
};

// Read-write capable
class TripleBufferedSharedIOMemory {
public:
    // \param size Size of the buffer region, in bytes. This class will reserve four times as much buffer for each read-write region.
    TripleBufferedSharedIOMemory(std::size_t writesize, std::size_t readsize, std::string shmpath = "ivshmem");
    // \param size Size of the buffer region, in bytes. This class will reserve eight times as much buffer.
    TripleBufferedSharedIOMemory(std::size_t size, const std::string& shmpath = "ivshmem");
    TripleBufferedSharedIOMemory(const TripleBufferedSharedIOMemory&) = delete;
    TripleBufferedSharedIOMemory(TripleBufferedSharedIOMemory&&) noexcept;
    ~TripleBufferedSharedIOMemory() noexcept;

    TripleBufferedSharedIOMemory& operator=(TripleBufferedSharedIOMemory&&) = default;
    operator bool() const noexcept { return !!_mmap; }

    // Fetch the base address for examination. Not intended for the contents to be writable.
    const void* const base_address() const noexcept { return _mmap; }

    // Finish writing the current buffer, submitting it for swapping.
    void write_ready(size_t size) noexcept;
    // Fetch the current buffer for writing.
    uintptr_t get_writable() const noexcept;
    std::size_t writable_size() const noexcept { return _writesize; }

    // USAGE NOTE: For reading, ready the buffer BEFORE reading (opposite of write, where you write, and then ready the buffer)
    void read_ready(size_t size) noexcept;
    // Fetch the current buffer for reading.
    uintptr_t get_readable() const noexcept;
    std::size_t readable_size() const noexcept { return _readsize; }

private:
    std::size_t _totalsize;
    std::size_t _writesize;
    std::size_t _readsize;
    std::string _path;
    void* _mmap;
};

#endif // OPENRAVE_TRIPLE_BUFFERED_SHARED_MEMORY_HPP