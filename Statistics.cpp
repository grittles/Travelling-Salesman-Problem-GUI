#include "Statistics.h"
//#include <QDebug>

StatTracker::~StatTracker() {
    //stopStatisticsCollection();  // Signal the thread to stop
    if (!isFinished()) {         // Check if the thread is not already finished
        wait();                  // Wait for the thread to complete
    }
}

void StatTracker::MemUsage() {

    double tempMemUsage;
    HANDLE hProcess = GetCurrentProcess();  // Get handle to current process
    PROCESS_MEMORY_COUNTERS_EX pmc;

    if (GetProcessMemoryInfo(GetCurrentProcess(), (PROCESS_MEMORY_COUNTERS*)&pmc, sizeof(pmc))) {

        double physicalusage = double(pmc.WorkingSetSize);
        double privusage = double(pmc.PrivateUsage);

        tempMemUsage = ((physicalusage > privusage) ? physicalusage : privusage) / (1 << 20); // use max between the two and get size in MB

    }
    else {
        tempMemUsage = -1;
    }

    CloseHandle(hProcess);  // Close handle when done

    if (tempMemUsage > _memUsage) _memUsage = tempMemUsage;
}

// because CPU usage is limited to one core, this essentially will give us no useful information.
void StatTracker::CpuUsage() {

    FILETIME idleTime, kernelTime, userTime;
    if (GetSystemTimes(&idleTime, &kernelTime, &userTime)) {
        // Conversion from FILETIME (100-nanosecond intervals) to seconds
        ULONGLONG idle = *((ULONGLONG*)&idleTime);
        ULONGLONG kernel = *((ULONGLONG*)&kernelTime);
        ULONGLONG user = *((ULONGLONG*)&userTime);

        ULONGLONG total = kernel + user;
        ULONGLONG idle_diff = idle - previous_idle;
        ULONGLONG total_diff = total - previous_total;

        _cpuUsage = (1.0 - ((double)idle_diff / total_diff)) * 100.0;

        previous_idle = idle;
        previous_total = total;
    }
}

StatTracker::StatTracker(QWidget* parent)
    : QThread(parent),
    running(false),
    _cpuUsage(0.0),
    _memUsage(0.0),
    previous_idle(0),
    previous_total(0),
    _totalTime(0),
    startTime({0}),
    endTime({0})
    {
    // kept getting stupid warnings about initializing these
    _parent = parent;
    QueryPerformanceFrequency(&frequency);

    timer = new QTimer();  // Initialize the timer
    timer->moveToThread(this);  // Move timer to the thread
    connect(timer, &QTimer::timeout, this, &StatTracker::collectStats);

}

void StatTracker::initializeThread() {
    //running = true; // Set running to true before starting the thread
    QThread::start(); // Start the thread itself
}

void StatTracker::start() {
    running = true;
    if (!timer->isActive()) {
        timer->start(10);  // Start the timer with an interval of 10 milliseconds
    }
}

void StatTracker::collectStats() {

    // pause global time tracker and append it before continuing
    QueryPerformanceCounter(&endTime);
    _totalTime += (double)(endTime.QuadPart - startTime.QuadPart) * 1000.0 / frequency.QuadPart;

    // lock the thread while we collect these statistics
    mutex.lock();
    
    MemUsage();

    mutex.unlock();

    //QueryPerformanceCounter(&startTime);

    if (running) start(); // Re-trigger the timer if needed
}

void StatTracker::GlobalTimeStart() {
    QueryPerformanceCounter(&startTime);
}

void StatTracker::GlobalTimeStop() {
    QueryPerformanceCounter(&endTime);
    _totalTime += (double)(endTime.QuadPart - startTime.QuadPart) * 1000.0 / frequency.QuadPart;
}

double StatTracker::GlobalTimeGet() const {
    return _totalTime;
}

void StatTracker::StoreGlobalTime(int nodecount) {
    statistics2.push_back(std::make_pair(nodecount, _totalTime));
    _totalTime = 0;
}

void StatTracker::StoreGlobalMemory(int nodecount) {
    statisticsMem.push_back(std::make_pair(nodecount, _memUsage));
    _memUsage = 0;
}

void StatTracker::stop() {
    mutex.lock();
    running = false;
    mutex.unlock();
}

// not really used
bool StatTracker::saveFile() {
    int doublesize = sizeof(double);
    int statsize = statistics.size();
    size_t totalSize = statsize * ((3 * doublesize) + 3); // the extra +3 is for 2 commas and newline character

    char* b = (char*)malloc(totalSize + doublesize + 2); // +2 is (1) newline on the first line and (2) null termiantor at the end
    if (b == nullptr) return false; // malloc failed

    int strsize = sprintf(b, "%d\n", statsize); // returns the number of characters written, not including null terminator

    char* p = b + strsize;

    for (const auto& stats : statistics) {
        int written = sprintf(p, "%f,%f,%f\n", stats[0], stats[1], stats[2]);
        p += written; // Move the pointer by the number of characters written
        strsize += written;
    }
    *p = '\0'; // Null-terminate the string
    strsize += 1;

    QFile* file = openFileForWriting(_parent);
    if (file == nullptr) {
        delete file; // Clean up
        return false;
    }

    file->write(b, strsize - 1); // Write all but the null terminator
    free(b);
    file->close();

    delete file;

    return true;
}

bool StatTracker::saveFile2(int type) {
    std::vector<std::pair<int, double>> statisticsUsed;

    if (type == 1) {
        statisticsUsed = statistics2;
    }
    else 
    if (type == 2) {
        statisticsUsed = statisticsMem;
    }

    int doublesize = sizeof(double);
    int statsize = statisticsUsed.size();
    size_t totalSize = statsize * ((sizeof(int) + sizeof(double)) + 2); // the extra +2 is for a comma and newline character

    char* b = (char*)malloc(totalSize + doublesize + 2); // +2 is (1) newline on the first line and (2) null termiantor at the end
    if (b == nullptr) return false; // malloc failed

    int strsize = sprintf(b, "%d\n", statsize); // returns the number of characters written, not including null terminator

    char* p = b + strsize;

    for (const auto& stats : statisticsUsed) {
        int written = sprintf(p, "%d,%f\n", stats.first, stats.second);
        p += written; // Move the pointer by the number of characters written
        strsize += written;
    }
    *p = '\0'; // Null-terminate the string
    strsize += 1;

    QFile* file = openFileForWriting(_parent);
    if (file == nullptr) {
        delete file; // Clean up
        return false;
    }

    file->write(b, strsize - 1); // Write all but the null terminator
    free(b);
    file->close();

    delete file;

    return true;
}