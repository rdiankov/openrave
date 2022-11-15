#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <sys/mman.h>
#include <unistd.h>

#include <stdexcept>

#include <openrave/openrave.h>

#include "triple_buffered_shared_memory.hpp"

using namespace std::literals;

constexpr size_t PTR_SIZE = sizeof(uintptr_t);
static_assert(PTR_SIZE == 8); // We expect the size of a pointer to be 8 bytes.
static_assert(PTR_SIZE == sizeof(size_t)); // We expect the size of size_t to be the same as a pointer's.

/// TripleBufferedSharedMemory

TripleBufferedSharedMemory::TripleBufferedSharedMemory(std::size_t size, std::string shmpath)
    : _totalsize((size * 4)) // Zephyr seems to choke on sizes that are not strict powers of 2.
    , _writesize(size)
    , _shmpath(std::move(shmpath))
    , _mmap(NULL) {
    auto shmfd = FileDescriptor(shm_open, _shmpath.c_str(), O_RDWR | O_CREAT, S_IRWXU);
    if (!shmfd) {
        throw std::runtime_error("Failed to shm_open: "s + strerror(errno));
    }
    int ret = ::ftruncate(shmfd.get(), _totalsize);
    if (ret == -1) {
        throw std::runtime_error("Failed to ftruncate ivshmem: "s + strerror(errno));
    }
    _mmap = ::mmap(NULL, _totalsize, PROT_READ | PROT_WRITE, MAP_SHARED, shmfd.get(), 0);
    RAVELOG_INFO("Base mapped address is %p\n", _mmap);
    if (_mmap == MAP_FAILED) {
        throw std::runtime_error("Failed to map memory of ivshmem: "s + strerror(errno));
    }

    // Compute the starting buffer address offsets
    uintptr_t present_offset = PTR_SIZE * 3;
    uintptr_t ready_offset = present_offset + size;
    uintptr_t inprogress_offset = ready_offset + size;

    uintptr_t* const offset = static_cast<uintptr_t*>(_mmap);
    // Write each offset value to the start of the shared memory block
    ::memcpy(&offset[0], &present_offset,    PTR_SIZE);
    ::memcpy(&offset[1], &ready_offset,      PTR_SIZE);
    ::memcpy(&offset[2], &inprogress_offset, PTR_SIZE);
}

TripleBufferedSharedMemory::TripleBufferedSharedMemory(TripleBufferedSharedMemory&& other) noexcept
    : _totalsize(other._totalsize)
    , _writesize(other._writesize)
    , _shmpath(std::move(other._shmpath))
    , _mmap(other._mmap) {
    other._mmap = NULL;
}

TripleBufferedSharedMemory::~TripleBufferedSharedMemory() noexcept {
    if (_mmap) {
        ::munmap(_mmap, _totalsize);
    }
    ::shm_unlink(_shmpath.c_str());
}

void TripleBufferedSharedMemory::write_ready() noexcept {
    // Exchange the values at _mmap[0] and _mmap[1]. Supported by GCC and Clang.
    // __atomic_exchange_n writes the value of __map[1] into *_mmap[0], and returns the previous value of *_mmap[0].
    // See https://stackoverflow.com/questions/32418511/gcc-atomic-exchange-seems-to-produce-non-atomic-asm-why
    static_cast<uintptr_t*>(_mmap)[1] = __atomic_exchange_n(
        &static_cast<uintptr_t*>(_mmap)[0],
        static_cast<uintptr_t*>(_mmap)[1],
        __ATOMIC_SEQ_CST
    );
}

uintptr_t TripleBufferedSharedMemory::get_writable() const noexcept {
    uintptr_t offset = static_cast<uintptr_t*>(_mmap)[0];
    RAVELOG_DEBUG("Active write offset is %p", offset);
    return reinterpret_cast<uintptr_t>(_mmap) + offset;
}

/// TripleBufferedSharedIOMemory

TripleBufferedSharedIOMemory::TripleBufferedSharedIOMemory(std::size_t writesize, std::size_t readsize, std::string shmpath)
    : _totalsize((writesize + readsize) * 4) // Zephyr seems to choke on sizes that are not strict powers of 2.
    , _writesize(writesize)
    , _readsize(readsize)
    , _shmpath(std::move(shmpath))
    , _mmap(NULL) {
    auto shmfd = FileDescriptor(shm_open, _shmpath.c_str(), O_RDWR | O_CREAT, S_IRWXU);
    if (!shmfd) {
        throw std::runtime_error("Failed to shm_open: "s + strerror(errno));
    }
    int ret = ::ftruncate(shmfd.get(), _totalsize);
    if (ret == -1) {
        throw std::runtime_error("Failed to ftruncate ivshmem: "s + strerror(errno));
    }
    _mmap = ::mmap(NULL, _totalsize, PROT_READ | PROT_WRITE, MAP_SHARED, shmfd.get(), 0);
    RAVELOG_INFO("Base mapped address is %p\n", _mmap);
    if (_mmap == MAP_FAILED) {
        throw std::runtime_error("Failed to map memory of ivshmem: "s + strerror(errno));
    }

    // Compute the starting write buffer address offsets
    uintptr_t write_present_offset = PTR_SIZE * 8; // Eight values: 3 for write, 3 for read, 2 for page sizes.
    uintptr_t write_ready_offset = write_present_offset + writesize;
    uintptr_t write_inprogress_offset = write_ready_offset + writesize;

    uintptr_t* const offset = static_cast<uintptr_t*>(_mmap);
    // Write each offset value to the start of the shared memory block
    ::memcpy(&offset[0], &write_present_offset,    PTR_SIZE);
    ::memcpy(&offset[1], &write_ready_offset,      PTR_SIZE);
    ::memcpy(&offset[2], &write_inprogress_offset, PTR_SIZE);

    // Compute the starting read buffer address offsets
    uintptr_t read_present_offset = write_inprogress_offset + writesize;
    uintptr_t read_ready_offset = read_present_offset + readsize;
    uintptr_t read_inprogress_offset = read_ready_offset + readsize;

    // Write each offset value to the start of the shared memory block
    ::memcpy(&offset[3], &read_present_offset,    PTR_SIZE);
    ::memcpy(&offset[4], &read_ready_offset,      PTR_SIZE);
    ::memcpy(&offset[5], &read_inprogress_offset, PTR_SIZE);

    // Write the size of one page to the next 2 values.
    ::memcpy(&offset[6], &writesize, PTR_SIZE);
    ::memcpy(&offset[7], &readsize,  PTR_SIZE);
}

TripleBufferedSharedIOMemory::TripleBufferedSharedIOMemory(std::size_t size, const std::string& shmpath)
    : TripleBufferedSharedIOMemory(size, size, shmpath) {}

TripleBufferedSharedIOMemory::TripleBufferedSharedIOMemory(TripleBufferedSharedIOMemory&& other) noexcept
    : _totalsize(other._totalsize)
    , _writesize(other._writesize)
    , _readsize(other._readsize)
    , _shmpath(std::move(_shmpath))
    , _mmap(other._mmap)
{
    other._mmap = NULL;
}

TripleBufferedSharedIOMemory::~TripleBufferedSharedIOMemory() noexcept {
    if (_mmap) {
        ::munmap(_mmap, _totalsize);
    }
    ::shm_unlink(_shmpath.c_str());
}

void TripleBufferedSharedIOMemory::write_ready() noexcept {
    uintptr_t* const offset = static_cast<uintptr_t*>(_mmap);
    offset[1] = __atomic_exchange_n(
        &offset[0], offset[1], __ATOMIC_SEQ_CST
    );
}

uintptr_t TripleBufferedSharedIOMemory::get_writable() const noexcept {
    uintptr_t offset = static_cast<uintptr_t*>(_mmap)[0];
    return reinterpret_cast<uintptr_t>(_mmap) + offset;
}

void TripleBufferedSharedIOMemory::read_ready() noexcept {
    uintptr_t* const offset = static_cast<uintptr_t*>(_mmap);
    offset[4] = __atomic_exchange_n(
        &offset[3], offset[4], __ATOMIC_SEQ_CST
    );
}

uintptr_t TripleBufferedSharedIOMemory::get_readable() const noexcept {
    uintptr_t offset = static_cast<uintptr_t*>(_mmap)[3];
    return reinterpret_cast<uintptr_t>(_mmap) + offset;
}
