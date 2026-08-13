#include "MainWindow.h"

#include <QApplication>

int main(int argc, char* argv[])
{
    QApplication application(argc, argv);
    QApplication::setApplicationName(QStringLiteral("JustFloatMonitor"));
    QApplication::setOrganizationName(QStringLiteral("BeaconImageAnalyzer"));

    MainWindow window;
    window.show();
    return application.exec();
}
