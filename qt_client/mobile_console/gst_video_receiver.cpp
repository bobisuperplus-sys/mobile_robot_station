#include "gst_video_receiver.h"
#include <QPainter>
#include <QDebug>
#include <QHash>
#include <QList>
#include <gst/video/video.h>

static QHash<int, QList<GstVideoReceiver*>> s_portReceivers;
static QHash<int, GstVideoReceiver*> s_masterReceivers;

GstVideoReceiver::GstVideoReceiver(QQuickItem *parent)
    : QQuickPaintedItem(parent)
{
    setAntialiasing(true);
    setOpaquePainting(true);

    // 1.5 秒无帧输入触发断流判定
    m_watchdogTimer = new QTimer(this);
    m_watchdogTimer->setInterval(1500);
    connect(m_watchdogTimer, &QTimer::timeout, this, &GstVideoReceiver::onWatchdogTimeout);

    // 1 秒定时计算实际输出 FPS
    m_fpsTimer = new QTimer(this);
    m_fpsTimer->setInterval(1000);
    connect(m_fpsTimer, &QTimer::timeout, this, &GstVideoReceiver::onFpsTimerTimeout);
    m_fpsTimer->start();

    setStatusText(QString("等待视讯输入 (UDP:%1)...").arg(m_port));
}

GstVideoReceiver::~GstVideoReceiver()
{
    stop();
}

void GstVideoReceiver::setPort(int port)
{
    if (m_port == port)
        return;

    m_port = port;
    Q_EMIT portChanged();

    if (m_pipeline) {
        restart();
    }
}

void GstVideoReceiver::setAutoStart(bool autoStart)
{
    if (m_autoStart == autoStart)
        return;

    m_autoStart = autoStart;
    Q_EMIT autoStartChanged();
}

void GstVideoReceiver::setStatusText(const QString &text)
{
    if (m_statusText == text)
        return;

    m_statusText = text;
    Q_EMIT statusTextChanged();
}

void GstVideoReceiver::componentComplete()
{
    QQuickPaintedItem::componentComplete();
    if (m_autoStart) {
        start();
    }
}

void GstVideoReceiver::start()
{
    if (m_pipeline) {
        return;
    }

    // 黄金零延迟零绿屏接收管道 (启用 reuse=true 支持多视口多线程端口复用)
    QString pipeStr = QString(
        "udpsrc port=%1 reuse=true buffer-size=2097152 caps=\"application/x-rtp,media=video,clock-rate=90000,encoding-name=H264,payload=96\" ! "
        "rtpjitterbuffer latency=10 drop-on-latency=true ! "
        "rtph264depay ! "
        "h264parse ! "
        "avdec_h264 max-threads=2 ! "
        "videoconvert ! "
        "video/x-raw,format=RGBA ! "
        "appsink name=sink emit-signals=true max-buffers=1 drop=true"
    ).arg(m_port);

    GError *error = nullptr;
    m_pipeline = gst_parse_launch(pipeStr.toUtf8().constData(), &error);
    if (error) {
        qWarning() << "[GstVideoReceiver] 管道创建失败 (端口" << m_port << "):" << error->message;
        setStatusText(QString("管道错误: %1").arg(error->message));
        g_error_free(error);
        return;
    }

    m_appsink = gst_bin_get_by_name(GST_BIN(m_pipeline), "sink");
    if (!m_appsink) {
        qWarning() << "[GstVideoReceiver] 未能获取 appsink 元素";
        cleanupPipeline();
        return;
    }

    g_signal_connect(m_appsink, "new-sample", G_CALLBACK(onNewSample), this);

    GstBus *bus = gst_element_get_bus(m_pipeline);
    if (bus) {
        m_busWatchId = gst_bus_add_watch(bus, onBusMessage, this);
        gst_object_unref(bus);
    }

    GstStateChangeReturn ret = gst_element_set_state(m_pipeline, GST_STATE_PLAYING);
    if (ret == GST_STATE_CHANGE_FAILURE) {
        qWarning() << "[GstVideoReceiver] 无法启动管道至 PLAYING 状态 (端口" << m_port << ")";
        cleanupPipeline();
        return;
    }

    m_watchdogTimer->start();
    setStatusText(QString("正在侦听 UDP 端口 %1...").arg(m_port));
}

void GstVideoReceiver::stop()
{
    cleanupPipeline();
    if (m_watchdogTimer) {
        m_watchdogTimer->stop();
    }
    if (m_isConnected) {
        m_isConnected = false;
        Q_EMIT isConnectedChanged();
    }
    m_fps = 0;
    Q_EMIT fpsChanged();
    setStatusText(QString("推流已停止 (UDP:%1)").arg(m_port));
    update();
}

void GstVideoReceiver::restart()
{
    stop();
    start();
}

void GstVideoReceiver::cleanupPipeline()
{
    if (m_busWatchId != 0) {
        g_source_remove(m_busWatchId);
        m_busWatchId = 0;
    }
    if (m_appsink) {
        gst_object_unref(m_appsink);
        m_appsink = nullptr;
    }
    if (m_pipeline) {
        gst_element_set_state(m_pipeline, GST_STATE_NULL);
        gst_object_unref(m_pipeline);
        m_pipeline = nullptr;
    }
}

GstFlowReturn GstVideoReceiver::onNewSample(GstElement *sink, gpointer userData)
{
    auto *receiver = static_cast<GstVideoReceiver *>(userData);
    GstSample *sample = gst_app_sink_pull_sample(GST_APP_SINK(sink));
    if (!sample) {
        return GST_FLOW_OK;
    }

    GstBuffer *buffer = gst_sample_get_buffer(sample);
    GstCaps *caps = gst_sample_get_caps(sample);
    if (buffer && caps) {
        GstStructure *s = gst_caps_get_structure(caps, 0);
        int width = 0;
        int height = 0;
        gst_structure_get_int(s, "width", &width);
        gst_structure_get_int(s, "height", &height);

        GstMapInfo map;
        if (gst_buffer_map(buffer, &map, GST_MAP_READ)) {
            // 构造 QImage 并进行深拷贝以脱离 GStreamer 缓冲区生命周期约束
            QImage img(reinterpret_cast<const uchar *>(map.data), width, height, QImage::Format_RGBA8888);
            QImage frameCopy = img.copy();
            gst_buffer_unmap(buffer, &map);

            QMetaObject::invokeMethod(receiver, "onFrameDecoded", Qt::QueuedConnection, Q_ARG(QImage, frameCopy));
        }
    }

    gst_sample_unref(sample);
    return GST_FLOW_OK;
}

void GstVideoReceiver::onFrameDecoded(const QImage &image)
{
    m_currentFrame = image;
    m_fpsCounter++;

    if (m_frameWidth != image.width() || m_frameHeight != image.height()) {
        m_frameWidth = image.width();
        m_frameHeight = image.height();
        Q_EMIT resolutionChanged();
    }

    if (!m_isConnected) {
        m_isConnected = true;
        Q_EMIT isConnectedChanged();
        setStatusText(QString("视讯已锁定 (UDP:%1 %2x%3)").arg(m_port).arg(m_frameWidth).arg(m_frameHeight));
    }

    // 重置看门狗
    m_watchdogTimer->start();

    Q_EMIT frameReceived();
    update();
}

void GstVideoReceiver::onWatchdogTimeout()
{
    if (m_isConnected) {
        m_isConnected = false;
        Q_EMIT isConnectedChanged();
        m_fps = 0;
        Q_EMIT fpsChanged();
        setStatusText(QString("视频流信号丢失，正在重试 (UDP:%1)...").arg(m_port));
        update();
    }
}

void GstVideoReceiver::onFpsTimerTimeout()
{
    if (m_fps != m_fpsCounter) {
        m_fps = m_fpsCounter;
        Q_EMIT fpsChanged();
    }
    m_fpsCounter = 0;
}

gboolean GstVideoReceiver::onBusMessage(GstBus *bus, GstMessage *message, gpointer userData)
{
    Q_UNUSED(bus);
    auto *receiver = static_cast<GstVideoReceiver *>(userData);
    switch (GST_MESSAGE_TYPE(message)) {
    case GST_MESSAGE_ERROR: {
        GError *err = nullptr;
        gchar *debug = nullptr;
        gst_message_parse_error(message, &err, &debug);
        qWarning() << "[GstVideoReceiver] 管道总线错误 (端口" << receiver->m_port << "):" << err->message;
        if (debug) {
            g_free(debug);
        }
        g_error_free(err);
        break;
    }
    case GST_MESSAGE_EOS:
        qInfo() << "[GstVideoReceiver] 收到 EOS (端口" << receiver->m_port << ")";
        break;
    default:
        break;
    }
    return TRUE;
}

void GstVideoReceiver::paint(QPainter *painter)
{
    QRectF rect = boundingRect();
    if (rect.isEmpty()) {
        return;
    }

    painter->save();
    painter->setClipRect(rect);

    if (m_isConnected && !m_currentFrame.isNull()) {
        // 工业视讯底色 (深黑)
        painter->fillRect(rect, QColor(0x0a, 0x0a, 0x0c));

        // 计算等比例缩放保持画面纵横比 (KeepAspectRatio)，保证全景视野不被裁切且不越界
        QSizeF imgSize = m_currentFrame.size();
        QSizeF scaledSize = imgSize.scaled(rect.size(), Qt::KeepAspectRatio);
        QRectF targetRect(
            rect.x() + (rect.width() - scaledSize.width()) / 2.0,
            rect.y() + (rect.height() - scaledSize.height()) / 2.0,
            scaledSize.width(),
            scaledSize.height()
        );

        painter->setRenderHint(QPainter::SmoothPixmapTransform, true);
        painter->drawImage(targetRect, m_currentFrame);
    } else {
        // 工业科技深灰质感待机背景
        painter->fillRect(rect, QColor(0x13, 0x13, 0x16));

        // 网格线
        painter->setPen(QPen(QColor(0x1e, 0x1e, 0x24), 0.75, Qt::SolidLine));
        qreal step = 28.0;
        for (qreal x = rect.left(); x <= rect.right(); x += step) {
            painter->drawLine(QPointF(x, rect.top()), QPointF(x, rect.bottom()));
        }
        for (qreal y = rect.top(); y <= rect.bottom(); y += step) {
            painter->drawLine(QPointF(rect.left(), y), QPointF(rect.right(), y));
        }

        // 中心待机准星与提示文字
        qreal cx = rect.center().x();
        qreal cy = rect.center().y();

        painter->setPen(QPen(QColor(0x3e, 0x90, 0xff, 80), 1.0, Qt::DashLine));
        painter->drawEllipse(QPointF(cx, cy), 36, 36);

        painter->setPen(QPen(QColor(0xaa, 0xc7, 0xff), 1.0));
        painter->drawLine(QPointF(cx - 12, cy), QPointF(cx + 12, cy));
        painter->drawLine(QPointF(cx, cy - 12), QPointF(cx, cy + 12));

        // 文字信息
        painter->setPen(QColor(0xaa, 0xc7, 0xff));
        QFont fontTitle("PingFang SC", 10, QFont::Bold);
        painter->setFont(fontTitle);
        QRectF titleRect(rect.left(), cy + 44, rect.width(), 20);
        painter->drawText(titleRect, Qt::AlignCenter, QString("等待视讯信号接入 (UDP:%1)").arg(m_port));

        painter->setPen(QColor(0x8e, 0x8d, 0x92));
        QFont fontSub("PingFang SC", 8);
        painter->setFont(fontSub);
        QRectF subRect(rect.left(), cy + 64, rect.width(), 16);
        painter->drawText(subRect, Qt::AlignCenter, QString("H.264 30FPS · 待机侦听中"));
    }

    painter->restore();
}
