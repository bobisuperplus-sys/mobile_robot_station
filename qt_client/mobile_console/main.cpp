#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQuickWindow>
#include <QTimer>
#include <QDebug>
#include <gst/gst.h>
#include "gst_video_receiver.h"
#include "msh_client.h"

int main(int argc, char *argv[])
{
    gst_init(&argc, &argv);

    QGuiApplication app(argc, argv);

    qmlRegisterType<GstVideoReceiver>("mobile_console", 1, 0, "GstVideoReceiver");
    qmlRegisterType<MshClient>("mobile_console", 1, 0, "MshClient");

    QString screenshotPath;
    int screenshotDelay = 2500;
    int initialMode = 0;
    int initialViewTab = 0;
    for (int i = 1; i < argc; ++i) {
        if (QString(argv[i]) == "--screenshot" && i + 1 < argc) {
            screenshotPath = QString::fromLocal8Bit(argv[i + 1]);
        } else if (QString(argv[i]) == "--delay" && i + 1 < argc) {
            screenshotDelay = QString(argv[i + 1]).toInt();
        } else if (QString(argv[i]) == "--mode" && i + 1 < argc) {
            initialMode = QString(argv[i + 1]).toInt();
        } else if (QString(argv[i]) == "--view-tab" && i + 1 < argc) {
            initialViewTab = QString(argv[i + 1]).toInt();
        }
    }

    QQmlApplicationEngine engine;
    engine.setInitialProperties({
        {"initialMode", initialMode},
        {"initialViewTab", initialViewTab}
    });
    QObject::connect(
        &engine,
        &QQmlApplicationEngine::objectCreationFailed,
        &app,
        []() { QCoreApplication::exit(-1); },
        Qt::QueuedConnection);
    engine.loadFromModule("mobile_console", "Main");

    if (!screenshotPath.isEmpty()) {
        QTimer::singleShot(screenshotDelay, [&engine, screenshotPath, &app]() {
            const auto rootObjects = engine.rootObjects();
            if (!rootObjects.isEmpty()) {
                auto *window = qobject_cast<QQuickWindow *>(rootObjects.first());
                if (window) {
                    QImage image = window->grabWindow();
                    if (image.save(screenshotPath)) {
                        qInfo() << "[Screenshot] 界面快照已成功保存至:" << screenshotPath;
                    } else {
                        qWarning() << "[Screenshot] 无法保存截图至:" << screenshotPath;
                    }
                }
            }
            app.quit();
        });
    }

    return QGuiApplication::exec();
}
