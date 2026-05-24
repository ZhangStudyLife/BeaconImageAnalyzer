#ifndef AI_INSTANCE_DIALOG_H
#define AI_INSTANCE_DIALOG_H

#include <QDialog>
#include <QString>

class QComboBox;
class QLineEdit;
class QNetworkAccessManager;
class QNetworkReply;
class QPlainTextEdit;
class QPushButton;

class AiInstanceDialog : public QDialog
{
    Q_OBJECT

public:
    explicit AiInstanceDialog(const QString& defaultRootDir, QWidget* parent = nullptr);

    QString instanceName() const;
    QString rootDir() const;
    QString generatedSource() const;
    QString generatedHeader() const;

private slots:
    void chooseRootDir();
    void generateSource();
    void handleReplyFinished(QNetworkReply* reply);

private:
    struct GeneratedFiles
    {
        QString header;
        QString source;
    };

    enum class ApiFormat
    {
        OpenAiCompatible,
        ClaudeCompatible
    };

    void loadSettings();
    void saveSettings() const;
    void setGenerating(bool generating);
    void postGenerationRequest(const QString& prompt);
    ApiFormat apiFormat() const;
    QUrl completionUrl() const;
    QByteArray requestBody(const QString& prompt) const;
    QString systemPrompt() const;
    QString userPrompt() const;
    QString repairPrompt(const QString& validationError) const;
    QString learnedFailurePrompt() const;
    void rememberValidationFailure(const QString& reason) const;
    QString extractAssistantText(const QByteArray& payload, QString* errorMessage) const;
    GeneratedFiles extractGeneratedFiles(const QByteArray& payload, QString* errorMessage) const;
    bool validateGeneratedFiles(const GeneratedFiles& files, QString* errorMessage) const;
    QString stripCodeFence(const QString& text) const;
    QString combinedPreview(const GeneratedFiles& files) const;

    QComboBox* m_formatCombo = nullptr;
    QLineEdit* m_baseUrlEdit = nullptr;
    QLineEdit* m_apiKeyEdit = nullptr;
    QLineEdit* m_modelEdit = nullptr;
    QLineEdit* m_nameEdit = nullptr;
    QLineEdit* m_rootDirEdit = nullptr;
    QPlainTextEdit* m_userCodeEdit = nullptr;
    QPlainTextEdit* m_generatedCodeEdit = nullptr;
    QPushButton* m_generateButton = nullptr;
    QPushButton* m_okButton = nullptr;
    QString m_defaultRootDir;
    QString m_generatedHeader;
    QString m_generatedSource;
    bool m_generatedFilesValid = false;
    int m_repairAttempt = 0;
    QNetworkAccessManager* m_network = nullptr;
};

#endif
