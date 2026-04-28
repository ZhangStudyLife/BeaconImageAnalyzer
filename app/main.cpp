#include "MainWindow.h"

#include "VideoExporter.h"
#include "VideoReader.h"

#include <QApplication>
#include <QTextStream>

namespace
{
int printUsage()
{
    QTextStream out(stdout);
    out << "BeaconImageAnalyzer\n"
        << "GUI: BeaconImageAnalyzer.exe\n"
        << "CLI:\n"
        << "  --probe <input.avi>\n"
        << "  --export-marked <input.avi> <output.avi>\n"
        << "  --export-csv <input.avi> <output.csv>\n";
    return 0;
}
}

int main(int argc, char* argv[])
{
    QApplication app(argc, argv);
    const QStringList args = app.arguments();

    if (args.size() > 1)
    {
        QTextStream out(stdout);
        QTextStream err(stderr);
        const QString command = args.at(1);

        if (command == QStringLiteral("--help") || command == QStringLiteral("-h"))
        {
            return printUsage();
        }

        if (command == QStringLiteral("--probe") && args.size() == 3)
        {
            VideoReader reader;
            QString error;
            if (!reader.open(args.at(2), &error))
            {
                err << error << '\n';
                return 2;
            }
            out << "file=" << reader.filePath() << '\n'
                << "size=" << reader.width() << 'x' << reader.height() << '\n'
                << "frames=" << reader.frameCount() << '\n'
                << "fps=" << reader.videoFps() << '\n'
                << "backend=" << reader.backendName() << '\n'
                << "codec=" << reader.codecName().trimmed() << '\n'
                << "bit_count=" << reader.bitCount() << '\n';
            return 0;
        }

        if (command == QStringLiteral("--export-marked") && args.size() == 4)
        {
            VideoExporter exporter;
            QString error;
            const bool ok = exporter.exportMarkedAvi(args.at(2), args.at(3), 50.0,
                                                     nullptr,
                                                     nullptr,
                                                     [](int current, int total) {
                                                         if (current == total || current % 100 == 0)
                                                         {
                                                             QTextStream(stdout) << "exported " << current << "/" << total << '\n';
                                                         }
                                                         return true;
                                                     },
                                                     &error);
            if (!ok)
            {
                err << error << '\n';
                return 3;
            }
            return 0;
        }

        if (command == QStringLiteral("--export-csv") && args.size() == 4)
        {
            VideoExporter exporter;
            QString error;
            const bool ok = exporter.exportResultCsv(args.at(2), args.at(3), 50.0,
                                                     nullptr,
                                                     [](int current, int total) {
                                                         if (current == total || current % 100 == 0)
                                                         {
                                                             QTextStream(stdout) << "csv " << current << "/" << total << '\n';
                                                         }
                                                         return true;
                                                     },
                                                     &error);
            if (!ok)
            {
                err << error << '\n';
                return 4;
            }
            return 0;
        }

        return printUsage();
    }

    MainWindow window;
    window.show();
    return app.exec();
}
