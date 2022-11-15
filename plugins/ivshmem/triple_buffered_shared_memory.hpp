#ifndef MUJIN_TRIPLE_BUFFERED_SHARED_MEMORY_HPP
#define MUJIN_TRIPLE_BUFFERED_SHARED_MEMORY_HPP

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

    // Finish writing the current buffer, submitting it for swapping.
    void write_ready() noexcept;
    // Fetch the current buffer for writing.
    uintptr_t get_writable() const noexcept;
    std::size_t writable_size() const noexcept { return _writesize; }

    // DESIGN CONCERN: Read semantics shouldn't need to work the same way as write semantics,
    // since we can just get a new buffer and read it on demand, which is a little different from writing.

    // Finish reading the current buffer, submitting it for swapping.
    void read_ready() noexcept;
    // Fetch the current buffer for reading.
    uintptr_t get_readable() const noexcept;
    std::size_t readable_size() const noexcept { return _readsize; }

private:
    std::size_t _totalsize;
    std::size_t _writesize;
    std::size_t _readsize;
    std::string _shmpath;
    void* _mmap;
};

#endif // MUJIN_TRIPLE_BUFFERED_SHARED_MEMORY_HPP