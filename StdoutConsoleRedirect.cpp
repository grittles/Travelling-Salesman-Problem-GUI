#include "ConsoleRedirect.h"

QPlainTextEdit* consoleOutput;
ConsoleRedirect* consoleRedirect;

void MainWindow::setupConsoleRedirect() {
    consoleOutput = new QPlainTextEdit(this);
    consoleOutput->setReadOnly(true);
    setCentralWidget(consoleOutput);

    consoleRedirect = new ConsoleRedirect(consoleOutput);
    std::cout.rdbuf(consoleRedirect);
}
