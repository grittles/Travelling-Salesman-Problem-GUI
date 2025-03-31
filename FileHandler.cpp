#include "FileHandler.h"

// Helper function to open a file using a dialog, with the default path being the executable's directory
//QFile* openFileUsingDialog(char* mode, QWidget* parent) {
//    if (*mode == 'r') {
//        return openFileForReading(parent);
//    }
//    else
//    if(*mode == 'w') {
//        return openFileForWriting(parent);
//    }
//}

// opens a file for reading using a dialogue box
// this just returns the pointer, we still have close the file and delete later.
QFile* openFileForReading(QWidget* parent) {
    QString fileName = QFileDialog::getOpenFileName(parent,
        QObject::tr("Open File"),
        QCoreApplication::applicationDirPath(),
        QObject::tr("Text Files (*.txt);;All Files (*)"));

    if (!fileName.isEmpty()) {
        QFile* file = new QFile(fileName);
        if (file->open(QIODevice::ReadOnly | QIODevice::Text)) {
            return file;
        }
    }
    return nullptr;
}

// Specific function for opening a file for writing
// this just returns the pointer, we still have close the file and delete later.
QFile* openFileForWriting(QWidget* parent) {
    QString fileName = QFileDialog::getSaveFileName(parent,
        QObject::tr("Save File"),
        QCoreApplication::applicationDirPath(),
        QObject::tr("Text Files (*.txt);;CSV Files (*.csv);;All Files (*)")
        );

    if (!fileName.isEmpty()) {
        QFile* file = new QFile(fileName);
        if (file->open(QIODevice::WriteOnly)) {
            return file;
        }
    }
    return new QFile();
}
