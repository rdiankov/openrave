#ifndef MY_TIMER_H
#define MY_TIMER_H

#ifdef WIN32
#include <windows.h>
typedef DWORD TimerCounterType;
#else
#include <sys/time.h>
typedef timeval TimerCounterType;
#endif //WIN32

class Timer
{
public:
    Timer();
    void Reset();

    // Returns elapsed time in milliseconds,seconds respectively
    long long ElapsedTicks();
    double ElapsedTime();

    // Doesn't refresh the current time
    long long LastElapsedTicks() const;
    double LastElapsedTime() const;

private:
    TimerCounterType start;
    TimerCounterType current;
};

#endif
