#include "AiInstanceDialog.h"

#include <QComboBox>
#include <QDialogButtonBox>
#include <QDir>
#include <QFileDialog>
#include <QFileInfo>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonParseError>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QNetworkAccessManager>
#include <QNetworkReply>
#include <QNetworkRequest>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QSettings>
#include <QUrl>
#include <QVBoxLayout>

namespace
{
constexpr const char* kOpenAiFormat = "openai";
constexpr const char* kClaudeFormat = "claude";
constexpr int kMaxRepairAttempts = 3;
constexpr int kMaxLearnedFailures = 8;

QUrl appendEndpoint(const QString& baseUrl, const QString& endpoint)
{
    QString normalized = baseUrl.trimmed();
    while (normalized.endsWith(QLatin1Char('/')))
    {
        normalized.chop(1);
    }

    if (normalized.endsWith(QStringLiteral("/chat/completions")) ||
        normalized.endsWith(QStringLiteral("/messages")))
    {
        return QUrl(normalized);
    }
    return QUrl(normalized + endpoint);
}

QString normalizedSourceForStaticCheck(const QString& source)
{
    QString normalized;
    normalized.reserve(source.size());
    for (const QChar ch : source)
    {
        if (!ch.isSpace())
        {
            normalized.append(ch.toLower());
        }
    }
    normalized.replace(QStringLiteral("(float)"), QString());
    normalized.replace(QStringLiteral("(double)"), QString());
    return normalized;
}
}

AiInstanceDialog::AiInstanceDialog(const QString& defaultRootDir, QWidget* parent)
    : QDialog(parent),
      m_defaultRootDir(defaultRootDir),
      m_network(new QNetworkAccessManager(this))
{
    setWindowTitle(QStringLiteral("添加实例"));
    resize(980, 740);

    m_formatCombo = new QComboBox(this);
    m_formatCombo->addItem(QStringLiteral("OpenAI 兼容接口"), QString::fromLatin1(kOpenAiFormat));
    m_formatCombo->addItem(QStringLiteral("Claude 兼容接口"), QString::fromLatin1(kClaudeFormat));

    m_baseUrlEdit = new QLineEdit(this);
    m_baseUrlEdit->setPlaceholderText(QStringLiteral("例如 https://api.openai.com 或 http://127.0.0.1:11434"));
    m_apiKeyEdit = new QLineEdit(this);
    m_apiKeyEdit->setEchoMode(QLineEdit::Password);
    m_modelEdit = new QLineEdit(this);
    m_modelEdit->setPlaceholderText(QStringLiteral("例如 gpt-4.1-mini / claude-3-5-sonnet-latest"));
    m_nameEdit = new QLineEdit(this);
    m_nameEdit->setPlaceholderText(QStringLiteral("实例名称"));
    m_rootDirEdit = new QLineEdit(this);

    auto* chooseButton = new QPushButton(QStringLiteral("选择目录"), this);
    connect(chooseButton, &QPushButton::clicked, this, &AiInstanceDialog::chooseRootDir);
    auto* rootRow = new QHBoxLayout;
    rootRow->addWidget(m_rootDirEdit, 1);
    rootRow->addWidget(chooseButton);

    auto* configGroup = new QGroupBox(QStringLiteral("AI 配置"), this);
    auto* configLayout = new QFormLayout(configGroup);
    configLayout->addRow(QStringLiteral("接口格式"), m_formatCombo);
    configLayout->addRow(QStringLiteral("Base URL"), m_baseUrlEdit);
    configLayout->addRow(QStringLiteral("API Key"), m_apiKeyEdit);
    configLayout->addRow(QStringLiteral("模型"), m_modelEdit);

    auto* instanceGroup = new QGroupBox(QStringLiteral("实例信息"), this);
    auto* instanceLayout = new QFormLayout(instanceGroup);
    instanceLayout->addRow(QStringLiteral("名称"), m_nameEdit);
    instanceLayout->addRow(QStringLiteral("保存目录"), rootRow);

    m_userCodeEdit = new QPlainTextEdit(this);
    m_userCodeEdit->setPlaceholderText(QStringLiteral("在这里粘贴用户自己的图像处理 C 代码、伪代码或关键函数。"));
    m_userCodeEdit->setMinimumHeight(180);
    m_generatedCodeEdit = new QPlainTextEdit(this);
    m_generatedCodeEdit->setReadOnly(true);
    m_generatedCodeEdit->setPlaceholderText(QStringLiteral("AI 生成并通过自检的 beacon_image.h / beacon_image.c 会显示在这里。"));
    m_generatedCodeEdit->setMinimumHeight(240);

    m_generateButton = new QPushButton(QStringLiteral("生成实例代码"), this);
    connect(m_generateButton, &QPushButton::clicked, this, &AiInstanceDialog::generateSource);
    connect(m_network, &QNetworkAccessManager::finished, this, &AiInstanceDialog::handleReplyFinished);

    auto* buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, this);
    m_okButton = buttons->button(QDialogButtonBox::Ok);
    m_okButton->setText(QStringLiteral("保存并加载"));
    m_okButton->setEnabled(false);
    buttons->button(QDialogButtonBox::Cancel)->setText(QStringLiteral("取消"));
    connect(buttons, &QDialogButtonBox::accepted, this, [this]() {
        if (!m_generatedFilesValid)
        {
            QMessageBox::warning(this, QStringLiteral("添加实例"), QStringLiteral("请先生成并通过自检。"));
            return;
        }
        saveSettings();
        accept();
    });
    connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);

    auto* root = new QVBoxLayout(this);
    auto* topRow = new QHBoxLayout;
    topRow->addWidget(configGroup, 1);
    topRow->addWidget(instanceGroup, 1);
    root->addLayout(topRow);
    root->addWidget(new QLabel(QStringLiteral("用户代码"), this));
    root->addWidget(m_userCodeEdit, 1);
    root->addWidget(m_generateButton, 0, Qt::AlignRight);
    root->addWidget(new QLabel(QStringLiteral("生成结果"), this));
    root->addWidget(m_generatedCodeEdit, 1);
    root->addWidget(buttons);

    loadSettings();
    if (m_rootDirEdit->text().trimmed().isEmpty())
    {
        m_rootDirEdit->setText(QDir(m_defaultRootDir).absoluteFilePath(QStringLiteral("ai_instance")));
    }
}

QString AiInstanceDialog::instanceName() const
{
    const QString name = m_nameEdit->text().trimmed();
    return name.isEmpty() ? QStringLiteral("AI 实例") : name;
}

QString AiInstanceDialog::rootDir() const
{
    return QFileInfo(m_rootDirEdit->text().trimmed()).absoluteFilePath();
}

QString AiInstanceDialog::generatedSource() const
{
    return m_generatedSource;
}

QString AiInstanceDialog::generatedHeader() const
{
    return m_generatedHeader;
}

void AiInstanceDialog::chooseRootDir()
{
    const QString dir = QFileDialog::getExistingDirectory(this,
                                                          QStringLiteral("选择实例保存目录"),
                                                          m_rootDirEdit->text().trimmed().isEmpty()
                                                              ? m_defaultRootDir
                                                              : m_rootDirEdit->text().trimmed());
    if (!dir.isEmpty())
    {
        m_rootDirEdit->setText(dir);
    }
}

void AiInstanceDialog::generateSource()
{
    if (m_baseUrlEdit->text().trimmed().isEmpty() ||
        m_apiKeyEdit->text().trimmed().isEmpty() ||
        m_modelEdit->text().trimmed().isEmpty())
    {
        QMessageBox::warning(this, QStringLiteral("AI 配置"), QStringLiteral("请填写 Base URL、API Key 和模型名称。"));
        return;
    }
    if (m_userCodeEdit->toPlainText().trimmed().isEmpty())
    {
        QMessageBox::warning(this, QStringLiteral("用户代码"), QStringLiteral("请先粘贴需要适配的图像处理代码。"));
        return;
    }

    saveSettings();
    m_repairAttempt = 0;
    m_generatedHeader.clear();
    m_generatedSource.clear();
    m_generatedFilesValid = false;
    setGenerating(true);
    m_generatedCodeEdit->setPlainText(QStringLiteral("正在请求 AI 生成 beacon_image.h / beacon_image.c..."));
    postGenerationRequest(userPrompt());
}

void AiInstanceDialog::handleReplyFinished(QNetworkReply* reply)
{
    reply->deleteLater();
    setGenerating(false);

    const QByteArray payload = reply->readAll();
    if (reply->error() != QNetworkReply::NoError)
    {
        m_generatedCodeEdit->setPlainText(QString::fromUtf8(payload));
        QMessageBox::critical(this,
                              QStringLiteral("AI 生成失败"),
                              QStringLiteral("请求失败：%1").arg(reply->errorString()));
        m_generatedFilesValid = false;
        m_okButton->setEnabled(false);
        return;
    }

    QString error;
    const GeneratedFiles files = extractGeneratedFiles(payload, &error);
    if (!error.isEmpty())
    {
        rememberValidationFailure(error);
        m_generatedCodeEdit->setPlainText(QString::fromUtf8(payload));
        if (m_repairAttempt < kMaxRepairAttempts)
        {
            ++m_repairAttempt;
            setGenerating(true);
            m_generatedCodeEdit->appendPlainText(
                QStringLiteral("\nAI 输出格式未通过自检，正在自动修复（第 %1/%2 次）：%3")
                    .arg(m_repairAttempt)
                    .arg(kMaxRepairAttempts)
                    .arg(error));
            postGenerationRequest(repairPrompt(error));
            return;
        }
        QMessageBox::critical(this, QStringLiteral("AI 生成失败"), error);
        m_generatedFilesValid = false;
        m_okButton->setEnabled(false);
        return;
    }

    if (!validateGeneratedFiles(files, &error))
    {
        rememberValidationFailure(error);
        m_generatedHeader = files.header;
        m_generatedSource = files.source;
        m_generatedCodeEdit->setPlainText(combinedPreview(files) +
                                          QStringLiteral("\n\n/* 自检失败：%1 */").arg(error));
        if (m_repairAttempt < kMaxRepairAttempts)
        {
            ++m_repairAttempt;
            setGenerating(true);
            m_generatedCodeEdit->appendPlainText(
                QStringLiteral("\n正在让 AI 自动修复（第 %1/%2 次）...")
                    .arg(m_repairAttempt)
                    .arg(kMaxRepairAttempts));
            postGenerationRequest(repairPrompt(error));
            return;
        }

        QMessageBox::critical(this,
                              QStringLiteral("AI 自检未通过"),
                              QStringLiteral("AI 已自动修复 %1 次，仍未通过：%2")
                                  .arg(kMaxRepairAttempts)
                                  .arg(error));
        m_generatedFilesValid = false;
        m_okButton->setEnabled(false);
        return;
    }

    m_generatedHeader = files.header;
    m_generatedSource = files.source;
    m_generatedFilesValid = true;
    m_generatedCodeEdit->setPlainText(combinedPreview(files));
    m_okButton->setEnabled(true);
}

void AiInstanceDialog::loadSettings()
{
    QSettings settings(QStringLiteral("BeaconImageAnalyzer"), QStringLiteral("BeaconImageAnalyzer"));
    const QString format = settings.value(QStringLiteral("ai/format"), QString::fromLatin1(kOpenAiFormat)).toString();
    const int index = m_formatCombo->findData(format);
    if (index >= 0)
    {
        m_formatCombo->setCurrentIndex(index);
    }
    m_baseUrlEdit->setText(settings.value(QStringLiteral("ai/base_url")).toString());
    m_apiKeyEdit->setText(settings.value(QStringLiteral("ai/api_key")).toString());
    m_modelEdit->setText(settings.value(QStringLiteral("ai/model")).toString());
}

void AiInstanceDialog::saveSettings() const
{
    QSettings settings(QStringLiteral("BeaconImageAnalyzer"), QStringLiteral("BeaconImageAnalyzer"));
    settings.setValue(QStringLiteral("ai/format"), m_formatCombo->currentData().toString());
    settings.setValue(QStringLiteral("ai/base_url"), m_baseUrlEdit->text().trimmed());
    settings.setValue(QStringLiteral("ai/api_key"), m_apiKeyEdit->text().trimmed());
    settings.setValue(QStringLiteral("ai/model"), m_modelEdit->text().trimmed());
}

void AiInstanceDialog::setGenerating(bool generating)
{
    m_generateButton->setEnabled(!generating);
    m_okButton->setEnabled(!generating && m_generatedFilesValid);
}

void AiInstanceDialog::postGenerationRequest(const QString& prompt)
{
    QNetworkRequest request(completionUrl());
    request.setHeader(QNetworkRequest::ContentTypeHeader, QStringLiteral("application/json"));
    if (apiFormat() == ApiFormat::OpenAiCompatible)
    {
        request.setRawHeader("Authorization", QByteArray("Bearer ") + m_apiKeyEdit->text().trimmed().toUtf8());
    }
    else
    {
        request.setRawHeader("x-api-key", m_apiKeyEdit->text().trimmed().toUtf8());
        request.setRawHeader("anthropic-version", "2023-06-01");
    }

    m_network->post(request, requestBody(prompt));
}

AiInstanceDialog::ApiFormat AiInstanceDialog::apiFormat() const
{
    return m_formatCombo->currentData().toString() == QString::fromLatin1(kClaudeFormat)
        ? ApiFormat::ClaudeCompatible
        : ApiFormat::OpenAiCompatible;
}

QUrl AiInstanceDialog::completionUrl() const
{
    if (apiFormat() == ApiFormat::ClaudeCompatible)
    {
        return appendEndpoint(m_baseUrlEdit->text(), QStringLiteral("/v1/messages"));
    }
    return appendEndpoint(m_baseUrlEdit->text(), QStringLiteral("/v1/chat/completions"));
}

QByteArray AiInstanceDialog::requestBody(const QString& prompt) const
{
    QJsonObject root;
    root.insert(QStringLiteral("model"), m_modelEdit->text().trimmed());

    if (apiFormat() == ApiFormat::ClaudeCompatible)
    {
        root.insert(QStringLiteral("max_tokens"), 8192);
        root.insert(QStringLiteral("system"), systemPrompt());
        QJsonArray messages;
        messages.append(QJsonObject{
            { QStringLiteral("role"), QStringLiteral("user") },
            { QStringLiteral("content"), prompt }
        });
        root.insert(QStringLiteral("messages"), messages);
    }
    else
    {
        root.insert(QStringLiteral("temperature"), 0.0);
        QJsonArray messages;
        messages.append(QJsonObject{
            { QStringLiteral("role"), QStringLiteral("system") },
            { QStringLiteral("content"), systemPrompt() }
        });
        messages.append(QJsonObject{
            { QStringLiteral("role"), QStringLiteral("user") },
            { QStringLiteral("content"), prompt }
        });
        root.insert(QStringLiteral("messages"), messages);
    }

    return QJsonDocument(root).toJson(QJsonDocument::Compact);
}

QString AiInstanceDialog::systemPrompt() const
{
    QString prompt = QStringLiteral(
        "你是嵌入式视觉 C 代码适配器生成器。请把用户提供的图像处理代码改写/封装成 "
        "BeaconImageAnalyzer 可加载的 beacon_image.h 和 beacon_image.c。\n\n"
        "输出格式硬性要求：\n"
        "1. 只输出一个 JSON 对象，不要 Markdown，不要解释。\n"
        "2. JSON 格式必须是：{\"beacon_image.h\":\"...\",\"beacon_image.c\":\"...\"}。\n"
        "3. JSON 字符串内换行必须正常转义，不能输出代码块围栏。\n\n"
        "文件要求：\n"
        "1. beacon_image.h 必须自包含，并定义 BEACON_IMAGE_W=188、BEACON_IMAGE_H=120、"
        "BEACON_MAX_CIRCLE_COUNT=8、beacon_circle_t、beacon_rect_t、beacon_result_t，以及 beacon_image_init、"
        "beacon_image_process、beacon_image_debug_binary 的声明。\n"
        "2. beacon_image.c 必须 #include \"beacon_image.h\"。\n"
        "3. beacon_image.c 必须导出 void beacon_image_init(void)。\n"
        "4. beacon_image.c 必须导出 void beacon_image_process(const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W], beacon_result_t *result)。\n"
        "5. beacon_image.c 可导出 beacon_image_debug_binary，用于二值图调试。\n"
        "6. 不能依赖用户电脑上的其它源文件或头文件；如果需要用户代码中的逻辑，请内联到 beacon_image.c。\n"
        "7. 用户代码里的算法参数不允许擅自修改，包括但不限于阈值、半径、面积范围、窗口大小、比例系数、"
        "滤波参数、循环边界、候选数量、变量含义和排序规则。只能做接口适配、类型适配和必要的单文件内联。\n"
        "8. 如果用户代码里有可配置参数，必须保持原值；如果必须新增默认值，只能用于用户代码没有提供的接口胶水代码。\n"
        "9. 输入图像固定为灰度 image[y][x]，宽 BEACON_IMAGE_W=188，高 BEACON_IMAGE_H=120。\n"
        "10. result 必须先清零，count 最大不能超过 BEACON_MAX_CIRCLE_COUNT，每个有效目标必须设置 valid=1。\n\n"
        "最重要的坐标规则：\n"
        "beacon_result_t 不是图像左上角坐标，而是以图像中心为原点的算法坐标。"
        "若检测到图像像素坐标 (pixel_x, pixel_y)，写入 result 前必须转换为下面这两个表达式，不能省略，不能反号，不能交换 x/y：\n"
        "result_x = BEACON_IMAGE_W * 0.5f - pixel_x;\n"
        "result_y = pixel_y - BEACON_IMAGE_H * 0.5f;\n"
        "然后只能写入 result->circles[i].x = result_x; result->circles[i].y = result_y; 半径仍使用像素半径。"
        "禁止把 pixel_x/pixel_y、center_x/center_y 或图像左上角坐标直接写入 result->circles[i].x/y，"
        "否则上位机显示框一定会偏移。\n\n"
        "生成前请自检：JSON 合法性、两个文件都存在、函数名、数组维度、count 上限、valid 字段、"
        "中心坐标转换、用户参数未被修改、空指针保护、无需外部依赖。最终仍然只输出 JSON。");
    prompt += QStringLiteral(
        "\n\n新增结果协议要求：\n"
        "1. beacon_image.h 必须在 beacon_result_t 中保留 legacy 字段 circles[BEACON_MAX_CIRCLE_COUNT] 和 count，"
        "并追加 beacons[BEACON_MAX_BEACON_COUNT]、beacon_count、car_lamps[BEACON_MAX_CAR_LAMP_COUNT]、car_lamp_count。\n"
        "2. 必须定义 BEACON_MAX_BEACON_COUNT=8 和 BEACON_MAX_CAR_LAMP_COUNT=2。字段顺序必须先 legacy circles/count，"
        "再追加 beacons/beacon_count/car_lamps/car_lamp_count，不能重排旧字段。\n"
        "3. 信标和车灯是并列结果：信标写入 result->beacons[] / result->beacon_count（beacon_circle_t 类型）；车灯写入 "
        "result->car_lamps[] / result->car_lamp_count（beacon_rect_t 类型）。禁止把车灯混入 beacons[]。\n"
        "4. 为兼容旧显示，信标结果也要同步写入 result->circles[] / result->count；车灯不要写入 legacy circles[]。\n"
        "5. v1 只支持一辆车的一对车灯，所以 car_lamp_count 最大为 2；没有车灯时必须明确保持为 0。\n"
        "6. 坐标转换适用于 beacons[] 和 legacy circles[]："
        "result_x = BEACON_IMAGE_W * 0.5f - pixel_x; result_y = pixel_y - BEACON_IMAGE_H * 0.5f。\n"
        "   car_lamps[] 使用 beacon_rect_t，坐标转换同样适用于 cx/cy："
        "result_cx = BEACON_IMAGE_W * 0.5f - pixel_cx; result_cy = pixel_cy - BEACON_IMAGE_H * 0.5f。\n"
        "   width/length 为像素尺寸，angle 为旋转角度（度）。\n"
        "7. 生成前自检是否填充 beacon_count 或兼容 legacy count，识别车灯时是否填充 car_lamps[] / car_lamp_count，"
        "是否保持中心坐标转换，是否没有擅自修改用户算法参数。\n");
    prompt += learnedFailurePrompt();
    return prompt;
}

QString AiInstanceDialog::userPrompt() const
{
    return QStringLiteral("请根据下面的用户图像处理代码生成 beacon_image.h 和 beacon_image.c：\n\n%1")
        .arg(m_userCodeEdit->toPlainText());
}

QString AiInstanceDialog::repairPrompt(const QString& validationError) const
{
    return QStringLiteral(
        "你上一次生成的文件没有通过上位机自检。\n"
        "自检错误：%1\n\n"
        "修复时禁止改动用户原始代码里的算法参数，只能修接口、JSON、导出函数和中心坐标转换问题。\n"
        "中心坐标转换必须保持：result_x = BEACON_IMAGE_W * 0.5f - pixel_x; "
        "result_y = pixel_y - BEACON_IMAGE_H * 0.5f。\n\n"
        "上一版 beacon_image.h：\n%2\n\n"
        "上一版 beacon_image.c：\n%3\n\n"
        "请修复后重新输出同样格式的 JSON：{\"beacon_image.h\":\"...\",\"beacon_image.c\":\"...\"}。"
        "不要解释，不要 Markdown。")
        .arg(validationError, m_generatedHeader, m_generatedSource);
}

QString AiInstanceDialog::learnedFailurePrompt() const
{
    QSettings settings(QStringLiteral("BeaconImageAnalyzer"), QStringLiteral("BeaconImageAnalyzer"));
    const QStringList failures = settings.value(QStringLiteral("ai/learned_validation_failures")).toStringList();
    if (failures.isEmpty())
    {
        return QString();
    }

    QStringList lines;
    for (int i = 0; i < failures.size(); ++i)
    {
        lines.push_back(QStringLiteral("%1. %2").arg(i + 1).arg(failures[i]));
    }

    return QStringLiteral(
               "\n\n本机历史自检失败经验，第一次生成时必须主动规避这些问题：\n%1\n"
               "这些经验只用于约束当前生成，不允许修改用户原始算法参数。")
        .arg(lines.join(QLatin1Char('\n')));
}

void AiInstanceDialog::rememberValidationFailure(const QString& reason) const
{
    const QString normalized = reason.simplified();
    if (normalized.isEmpty())
    {
        return;
    }

    QSettings settings(QStringLiteral("BeaconImageAnalyzer"), QStringLiteral("BeaconImageAnalyzer"));
    QStringList failures = settings.value(QStringLiteral("ai/learned_validation_failures")).toStringList();
    failures.removeAll(normalized);
    failures.prepend(normalized);
    while (failures.size() > kMaxLearnedFailures)
    {
        failures.removeLast();
    }
    settings.setValue(QStringLiteral("ai/learned_validation_failures"), failures);
}

QString AiInstanceDialog::extractAssistantText(const QByteArray& payload, QString* errorMessage) const
{
    QJsonParseError parseError;
    const QJsonDocument doc = QJsonDocument::fromJson(payload, &parseError);
    if (parseError.error != QJsonParseError::NoError || !doc.isObject())
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("AI 返回内容不是合法 JSON。");
        }
        return QString();
    }

    QString content;
    const QJsonObject root = doc.object();
    if (apiFormat() == ApiFormat::ClaudeCompatible)
    {
        const QJsonArray blocks = root.value(QStringLiteral("content")).toArray();
        for (const QJsonValue& value : blocks)
        {
            const QJsonObject block = value.toObject();
            if (block.value(QStringLiteral("type")).toString() == QStringLiteral("text"))
            {
                content += block.value(QStringLiteral("text")).toString();
            }
        }
    }
    else
    {
        const QJsonArray choices = root.value(QStringLiteral("choices")).toArray();
        if (!choices.isEmpty())
        {
            content = choices.first().toObject()
                          .value(QStringLiteral("message")).toObject()
                          .value(QStringLiteral("content")).toString();
        }
    }

    content = stripCodeFence(content).trimmed();
    if (content.isEmpty() && errorMessage != nullptr)
    {
        *errorMessage = QStringLiteral("AI 返回中没有找到文本内容。");
    }
    return content;
}

AiInstanceDialog::GeneratedFiles AiInstanceDialog::extractGeneratedFiles(const QByteArray& payload,
                                                                         QString* errorMessage) const
{
    GeneratedFiles files;
    const QString text = extractAssistantText(payload, errorMessage);
    if (text.isEmpty())
    {
        return files;
    }

    QJsonParseError parseError;
    const QJsonDocument doc = QJsonDocument::fromJson(text.toUtf8(), &parseError);
    if (parseError.error != QJsonParseError::NoError || !doc.isObject())
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("AI 输出不是合法的文件 JSON。");
        }
        return files;
    }

    const QJsonObject root = doc.object();
    files.header = root.value(QStringLiteral("beacon_image.h")).toString().trimmed();
    files.source = root.value(QStringLiteral("beacon_image.c")).toString().trimmed();
    if ((files.header.isEmpty() || files.source.isEmpty()) && errorMessage != nullptr)
    {
        *errorMessage = QStringLiteral("AI 输出缺少 beacon_image.h 或 beacon_image.c。");
    }
    return files;
}

bool AiInstanceDialog::validateGeneratedFiles(const GeneratedFiles& files, QString* errorMessage) const
{
    const QString header = files.header;
    const QString source = files.source;
    const QString normalizedHeader = normalizedSourceForStaticCheck(header);
    const QString normalizedSource = normalizedSourceForStaticCheck(source);

    if (!header.contains(QStringLiteral("BEACON_IMAGE_W")) ||
        !header.contains(QStringLiteral("BEACON_IMAGE_H")) ||
        !header.contains(QStringLiteral("beacon_result_t")) ||
        !header.contains(QStringLiteral("beacon_image_process")))
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("beacon_image.h 缺少必要宏、结构体或函数声明。");
        }
        return false;
    }
    if (!source.contains(QStringLiteral("#include \"beacon_image.h\"")) ||
        !source.contains(QStringLiteral("beacon_image_init")) ||
        !source.contains(QStringLiteral("beacon_image_process")))
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("beacon_image.c 缺少头文件引用或必要导出函数。");
        }
        return false;
    }
    if (!normalizedHeader.contains(QStringLiteral("beacon_image_w188")) &&
        !normalizedHeader.contains(QStringLiteral("beacon_image_w=188")))
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("beacon_image.h 必须定义 BEACON_IMAGE_W 为 188。");
        }
        return false;
    }
    if (!normalizedHeader.contains(QStringLiteral("beacon_image_h120")) &&
        !normalizedHeader.contains(QStringLiteral("beacon_image_h=120")))
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("beacon_image.h 必须定义 BEACON_IMAGE_H 为 120。");
        }
        return false;
    }
    if (!header.contains(QStringLiteral("BEACON_MAX_BEACON_COUNT")) ||
        !header.contains(QStringLiteral("BEACON_MAX_CAR_LAMP_COUNT")) ||
        !header.contains(QStringLiteral("beacons")) ||
        !header.contains(QStringLiteral("beacon_count")) ||
        !header.contains(QStringLiteral("beacon_rect_t")) ||
        !header.contains(QStringLiteral("car_lamps")) ||
        !header.contains(QStringLiteral("car_lamp_count")))
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("beacon_image.h 必须支持并列结果 beacons[]/beacon_count 和 car_lamps[]/car_lamp_count。");
        }
        return false;
    }
    if (!normalizedSource.contains(QStringLiteral("memset(result,0")) &&
        !normalizedSource.contains(QStringLiteral("result->count=0")))
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("beacon_image_process 必须先清空 result。");
        }
        return false;
    }
    if (!normalizedSource.contains(QStringLiteral("valid=1")))
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("有效目标必须设置 valid=1。");
        }
        return false;
    }

    if (!normalizedSource.contains(QStringLiteral("beacon_count")) ||
        !normalizedSource.contains(QStringLiteral("beacons[")))
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("beacon_image_process 必须填充 beacons[]/beacon_count，不能只写 legacy circles/count。");
        }
        return false;
    }
    if (source.contains(QStringLiteral("car"), Qt::CaseInsensitive) &&
        (!normalizedSource.contains(QStringLiteral("car_lamps[")) ||
         !normalizedSource.contains(QStringLiteral("car_lamp_count"))))
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("检测到车灯相关逻辑时，必须填充 car_lamps[]/car_lamp_count。");
        }
        return false;
    }

    const bool hasCenterX = normalizedSource.contains(QStringLiteral("beacon_image_w*0.5f-")) ||
                            normalizedSource.contains(QStringLiteral("94.0f-"));
    const bool hasCenterY = normalizedSource.contains(QStringLiteral("-beacon_image_h*0.5f")) ||
                            normalizedSource.contains(QStringLiteral("-60.0f"));
    if (!hasCenterX || !hasCenterY)
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("缺少像素坐标到图像中心坐标的转换，可能导致信标框显示偏移。");
        }
        return false;
    }

    return true;
}

QString AiInstanceDialog::stripCodeFence(const QString& text) const
{
    QString result = text.trimmed();
    if (!result.startsWith(QStringLiteral("```")))
    {
        return result;
    }

    const int firstLineBreak = result.indexOf(QLatin1Char('\n'));
    if (firstLineBreak >= 0)
    {
        result = result.mid(firstLineBreak + 1);
    }
    if (result.endsWith(QStringLiteral("```")))
    {
        result.chop(3);
    }
    return result.trimmed();
}

QString AiInstanceDialog::combinedPreview(const GeneratedFiles& files) const
{
    return QStringLiteral("/* beacon_image.h */\n%1\n\n/* beacon_image.c */\n%2")
        .arg(files.header, files.source);
}
