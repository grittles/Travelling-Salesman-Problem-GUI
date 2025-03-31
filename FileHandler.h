#pragma once
#ifndef FILEHANDLER_H
#define FILEHANDLER_H

#include <QFileDialog>
#include <QFile>
#include <QCoreApplication>

// https://learn.microsoft.com/en-us/windows/win32/shell/common-file-dialog#controlling-the-default-folder
// https://github.com/microsoft/Windows-classic-samples/blob/main/Samples/Win7Samples/winui/shell/appplatform/commonfiledialog/CommonFileDialogApp.cpp
//QFile* openFileUsingDialog(char* mode, QWidget* parent);
QFile* openFileForReading(QWidget* parent);
QFile* openFileForWriting(QWidget* parent);

#endif
