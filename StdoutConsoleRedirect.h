#pragma once

#include <QPlainTextEdit>
#include <QMutex>
#include <iostream>
#include <streambuf>

class ConsoleRedirect : public std::streambuf {
public:
    ConsoleRedirect(QPlainTextEdit* textWidget) : textWidget(textWidget) {}

protected:
    virtual int overflow(int c) override {
        if (c != EOF) {
            QMutexLocker locker(&mutex);
            textWidget->appendPlainText(QString(QChar(c)));
        }
        return c;
    }

private:
    QPlainTextEdit* textWidget;
    QMutex mutex;
};
