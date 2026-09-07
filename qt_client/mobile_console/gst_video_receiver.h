#pragma once

#include <QQuickPaintedItem>
#include <QImage>
#include <QTimer>
#include <QString>
#include <gst/gst.h>
#include <gst/app/gstappsink.h>

/**
 * @brief GStreamer 工业级低延迟视频流接收渲染控件
 * 负责通过 UDP 接收 H.264 RTP 数据流，使用 avdec_h264 进行极速解码并通过 appsink 转换为 QImage 绘制。
 */
class GstVideoReceiver : public QQuickPaintedItem {
    Q_OBJECT
    Q_PROPERTY(int port READ port WRITE setPort NOTIFY portChanged)
    Q_PROPERTY(bool isConnected READ isConnected NOTIFY isConnectedChanged)
    Q_PROPERTY(int fps READ fps NOTIFY fpsChanged)
    Q_PROPERTY(int frameWidth READ frameWidth NOTIFY resolutionChanged)
    Q_PROPERTY(int frameHeight READ frameHeight NOTIFY resolutionChanged)
    Q_PROPERTY(bool autoStart READ autoStart WRITE setAutoStart NOTIFY autoStartChanged)
    Q_PROPERTY(QString statusText READ statusText NOTIFY statusTextChanged)

public:
    explicit GstVideoReceiver(QQuickItem *parent = nullptr);
    ~GstVideoReceiver() override;

    int port() const { return m_port; }
    void setPort(int port);

    bool isConnected() const { return m_isConnected; }
    int fps() const { return m_fps; }
    int frameWidth() const { return m_frameWidth; }
    int frameHeight() const { return m_frameHeight; }

    bool autoStart() const { return m_autoStart; }
    void setAutoStart(bool autoStart);

    QString statusText() const { return m_statusText; }

    void paint(QPainter *painter) override;
    void componentComplete() override;

public Q_SLOTS:
    void start();
    void stop();
    void restart();

Q_SIGNALS:
    void portChanged();
    void isConnectedChanged();
    void fpsChanged();
    void resolutionChanged();
    void autoStartChanged();
    void statusTextChanged();
    void frameReceived();

private Q_SLOTS:
    void onFrameDecoded(const QImage &image);
    void onWatchdogTimeout();
    void onFpsTimerTimeout();

public:
    void setSharedFrame(const QImage &image);

private:
    void cleanupPipeline();
    void setStatusText(const QString &text);
    static GstFlowReturn onNewSample(GstElement *sink, gpointer userData);
    static gboolean onBusMessage(GstBus *bus, GstMessage *message, gpointer userData);

    int m_port = 5002;
    bool m_isConnected = false;
    int m_fps = 0;
    int m_fpsCounter = 0;
    int m_frameWidth = 0;
    int m_frameHeight = 0;
    bool m_autoStart = true;
    QString m_statusText;

    QImage m_currentFrame;
    QTimer *m_watchdogTimer = nullptr;
    QTimer *m_fpsTimer = nullptr;

    GstElement *m_pipeline = nullptr;
    GstElement *m_appsink = nullptr;
    guint m_busWatchId = 0;
};
