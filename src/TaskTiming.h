#ifndef TASK_TIMING_H
#define TASK_TIMING_H

// Einfache Intervallsteuerung für loop()-Zyklen
struct TaskTimer {
    unsigned long lastRun = 0;
    unsigned long interval = 100;

    bool isDue(unsigned long now) {
        return (now - lastRun) >= interval;
    }

    void reset(unsigned long now) {
        lastRun = now;
    }
};

#endif
