#ifndef OPENRAVE_SEMAPHORE_HPP
#define OPENRAVE_SEMAPHORE_HPP

#include <semaphore.h>

namespace std {

template <int LeastMaxValue>
class counting_semaphore {
public:
    static_assert(LeastMaxValue > 0, "LeastMaxValue must be a positive value.");

    constexpr explicit counting_semaphore(int desired) noexcept {
        [[maybe_unused]] int ec = ::sem_init(&_sem, 0, desired);
    }
    constexpr explicit counting_semaphore() noexcept : counting_semaphore(0) {}
    counting_semaphore(const counting_semaphore&) = delete;
    ~counting_semaphore() noexcept {
        [[maybe_unused]] int ec = ::sem_destroy(&_sem);
    }

    void release() noexcept {
        int count = 0;
        [[maybe_unused]] int ec = ::sem_getvalue(&_sem, &count);
        if (count < LeastMaxValue) {
            ec = ::sem_post(&_sem);
        }
    }

    void release(int update) noexcept {
        for (; update > 0; --update) {
            release();
        }
    }

    /// \copydoc sem_t()
    void acquire() noexcept {
        [[maybe_unused]] int ec = ::sem_wait(&_sem);
    }

    /// \copydoc sem_trywait()
    bool try_acquire() noexcept {
        return ::sem_trywait(&_sem) == 0;
    }

    constexpr static int max() noexcept {
        return LeastMaxValue;
    }

    sem_t* native_handle() noexcept {
        return &_sem;
    }

    // Non-std extensions available in Zephyr
    unsigned int Count() const noexcept {
        int count = 0;
        [[maybe_unused]] int ec = ::sem_getvalue(&_sem, &count);
        return count;
    }

private:
    ::sem_t _sem;
};

using binary_semaphore = counting_semaphore<1>;

} // namespace

#endif // OPENRAVE_SEMAPHORE_HPP