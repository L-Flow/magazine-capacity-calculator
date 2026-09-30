#include "MainWindow.hpp"

#include <QApplication>
#include <QFile>
#include <QFileInfo>
#include <QTextStream>
#include <QTimer>

int main(int argc, char** argv) {
    QApplication application(argc, argv);
    QApplication::setApplicationName(QStringLiteral("Magazine Capacity"));
    const bool smokeTest = argc == 3 && QString::fromLocal8Bit(argv[1]) ==
                                      QStringLiteral("--smoke-test");
    MainWindow window(!smokeTest);
    window.show();

    if (smokeTest) {
        const QString output = QFileInfo(QString::fromLocal8Bit(argv[2]))
                                   .absoluteFilePath();
        QFile logFile(output + QStringLiteral(".log"));
        logFile.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text);
        QTextStream log(&logFile);
        log << "window-shown\n";
        log.flush();
        QTimer::singleShot(1800, &application,
                           [&application, &window, output] {
            QFile logFile(output + QStringLiteral(".log"));
            logFile.open(QIODevice::WriteOnly | QIODevice::Append |
                         QIODevice::Text);
            QTextStream log(&logFile);
            log << "snapshot-start\n";
            log.flush();
            const bool saved = window.saveSnapshot(output);
            log << (saved ? "snapshot-ok\n" : "snapshot-failed\n");
            log.flush();
            logFile.close();
            application.exit(saved ? 0 : 3);
        });
    }
    return application.exec();
}

