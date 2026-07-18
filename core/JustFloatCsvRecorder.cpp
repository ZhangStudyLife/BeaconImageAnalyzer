#include "JustFloatCsvRecorder.h"

#include <QDir>
#include <QFile>
#include <QSaveFile>

namespace
{
bool writeAll(QFileDevice* file, const QByteArray& data)
{
    qint64 written = 0;
    while (written < data.size())
    {
        const qint64 result = file->write(data.constData() + written, data.size() - written);
        if (result <= 0)
        {
            return false;
        }
        written += result;
    }
    return true;
}
}

JustFloatCsvRecorder::JustFloatCsvRecorder()
{
    m_temporaryFile.setAutoRemove(true);
    m_temporaryFile.setFileTemplate(
        QDir::temp().absoluteFilePath(QStringLiteral("BeaconImageAnalyzer-JustFloat-XXXXXX.csv")));
}

bool JustFloatCsvRecorder::start(QString* errorMessage)
{
    if (m_state != State::Idle)
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("当前记录尚未保存或放弃。");
        }
        return false;
    }

    if (m_temporaryFile.isOpen())
    {
        m_temporaryFile.close();
    }
    if (!m_temporaryFile.fileName().isEmpty())
    {
        m_temporaryFile.remove();
    }
    if (!m_temporaryFile.open())
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("无法创建临时记录文件：%1").arg(m_temporaryFile.errorString());
        }
        return false;
    }

    const QByteArray header = JustFloatLog::csvHeader().toUtf8() + '\n';
    if (!writeAll(&m_temporaryFile, header))
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("无法写入临时记录文件：%1").arg(m_temporaryFile.errorString());
        }
        m_temporaryFile.close();
        m_temporaryFile.remove();
        return false;
    }

    m_rowCount = 0;
    m_state = State::Recording;
    return true;
}

bool JustFloatCsvRecorder::append(const JustFloatLogRow& row, QString* errorMessage)
{
    if (m_state != State::Recording || !m_temporaryFile.isOpen())
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("当前未在记录 UDP 数据。");
        }
        return false;
    }

    const QByteArray line = JustFloatLog::csvRow(row).toUtf8() + '\n';
    if (!writeAll(&m_temporaryFile, line))
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("写入临时记录文件失败：%1").arg(m_temporaryFile.errorString());
        }
        return false;
    }

    ++m_rowCount;
    return true;
}

bool JustFloatCsvRecorder::stop(QString* errorMessage)
{
    if (m_state == State::PendingSave)
    {
        return true;
    }
    if (m_state != State::Recording)
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("当前没有正在进行的记录。");
        }
        return false;
    }

    const bool flushed = m_temporaryFile.flush();
    if (!flushed && errorMessage != nullptr)
    {
        *errorMessage = QStringLiteral("刷新临时记录文件失败：%1").arg(m_temporaryFile.errorString());
    }
    m_temporaryFile.close();
    m_state = State::PendingSave;
    return flushed;
}

bool JustFloatCsvRecorder::resume(QString* errorMessage)
{
    if (m_state == State::Recording)
    {
        return true;
    }
    if (m_state != State::PendingSave)
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("没有可继续的 UDP 记录。");
        }
        return false;
    }
    if (!m_temporaryFile.open())
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("无法继续写入临时记录文件：%1")
                                .arg(m_temporaryFile.errorString());
        }
        return false;
    }
    if (!m_temporaryFile.seek(m_temporaryFile.size()))
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("无法定位到临时记录文件末尾：%1")
                                .arg(m_temporaryFile.errorString());
        }
        m_temporaryFile.close();
        return false;
    }

    m_state = State::Recording;
    return true;
}

bool JustFloatCsvRecorder::saveAs(const QString& path, QString* errorMessage)
{
    if (m_state != State::PendingSave)
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("没有等待保存的记录。");
        }
        return false;
    }
    if (path.trimmed().isEmpty())
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("保存路径为空。");
        }
        return false;
    }

    QFile source(m_temporaryFile.fileName());
    if (!source.open(QIODevice::ReadOnly))
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("无法读取临时记录文件：%1").arg(source.errorString());
        }
        return false;
    }

    QSaveFile destination(path);
    if (!destination.open(QIODevice::WriteOnly))
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("无法创建 CSV：%1").arg(destination.errorString());
        }
        return false;
    }

    QByteArray buffer(64 * 1024, Qt::Uninitialized);
    while (true)
    {
        const qint64 bytesRead = source.read(buffer.data(), buffer.size());
        if (bytesRead < 0)
        {
            if (errorMessage != nullptr)
            {
                *errorMessage = QStringLiteral("读取临时记录文件失败：%1").arg(source.errorString());
            }
            destination.cancelWriting();
            return false;
        }
        if (bytesRead == 0)
        {
            break;
        }
        if (!writeAll(&destination, QByteArray::fromRawData(buffer.constData(), bytesRead)))
        {
            if (errorMessage != nullptr)
            {
                *errorMessage = QStringLiteral("写入 CSV 失败：%1").arg(destination.errorString());
            }
            destination.cancelWriting();
            return false;
        }
    }
    source.close();

    if (!destination.commit())
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("保存 CSV 失败：%1").arg(destination.errorString());
        }
        return false;
    }

    discard();
    return true;
}

void JustFloatCsvRecorder::discard()
{
    if (m_temporaryFile.isOpen())
    {
        m_temporaryFile.close();
    }
    if (!m_temporaryFile.fileName().isEmpty())
    {
        m_temporaryFile.remove();
    }
    m_state = State::Idle;
    m_rowCount = 0;
}

JustFloatCsvRecorder::State JustFloatCsvRecorder::state() const
{
    return m_state;
}

quint64 JustFloatCsvRecorder::rowCount() const
{
    return m_rowCount;
}
