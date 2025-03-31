#pragma once
#ifndef STATISTICS_H
#define STATISTICS_H

#include <vector>

#include <QThread>
#include <QTimer>
#include <QMutex>

#include <windows.h> // for timing
#include <psapi.h> // for memory collection

#include "FileHandler.h"

class QWidget; // Forward declaration?

class StatTracker : public QThread {
    Q_OBJECT
public:
    explicit StatTracker(QWidget* parent = nullptr);
    ~StatTracker();  // Declare the destructor
    void initializeThread();
    void MemUsage();
    void CpuUsage();

    void GlobalTimeStart();
    void GlobalTimeStop();
    double GlobalTimeGet() const;

    void StoreGlobalTime(int nodecount);

    void StoreGlobalMemory(int nodecount);

    void start();
    void stop();  // Method to request stopping the thread safely
    bool saveFile();
    bool saveFile2(int type);

    void collectStats();

    bool running;  // Controls whether the timer should continue

protected:
    //void run() override;
private slots:
    
    //void collectStatsMem();
private:
    QTimer* timer;
    
    QMutex mutex;          // Mutex for thread-safe operations

    ULONGLONG previous_idle;
    ULONGLONG previous_total;

    LARGE_INTEGER frequency;
    LARGE_INTEGER startTime;
    LARGE_INTEGER endTime;

    double _cpuUsage = 0;
    double _memUsage = 0;

    double _totalTime = 0;

    QWidget* _parent = nullptr;


    std::vector<std::vector<double>> statistics;  // 2D vector to store statistics
    std::vector<std::pair<int, double>> statistics2;  // 2D vector to store CPU and memory statistics for n nodes
    std::vector<std::pair<int, double>> statisticsMem;  // 2D vector to store CPU and memory statistics for n nodes
};

#endif // STATISTICS_H