#pragma once

#include <QCoreApplication>
#include <QTimer>
#include <QDebug>

class ApplicationManager : public QObject {
    Q_OBJECT
        QTimer timer;

public:
    explicit ApplicationManager(QObject* parent = nullptr) : QObject(parent) {
        connect(&timer, &QTimer::timeout, this, &ApplicationManager::onTimerTick);
        timer.setInterval(100);  // Set the timer interval to 1000 ms
    }

    //QtWidgetsApplication1* main;

    

public slots:
    void onTimerTick() {

        //qDebug() << "Timer tick";
    }

    void handleStartTimer() {
        if (!timer.isActive())
            timer.start();
    }

    void handleStopTimer() {
        if (timer.isActive())
            timer.stop();
    }
};