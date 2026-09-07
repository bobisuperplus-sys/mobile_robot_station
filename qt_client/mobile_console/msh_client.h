#pragma once

#include <QObject>
#include <QTcpSocket>
#include <QTimer>
#include <QString>
#include <QJsonObject>
#include <QJsonDocument>
#include <QJsonArray>

/**
 * @brief MSH (Mobile Station Host) TCP JSON-RPC 2.0 客户端数据桥梁
 * 负责管理与 Python 后端 (默认 127.0.0.1:9001) 的长连接、心跳轮询与遥控指令分发。
 * 当后端未开启时，平滑运行高保真演示态数据，保证上位机界面全功能可交互。
 */
class MshClient : public QObject {
    Q_OBJECT

    // 连接与模式
    Q_PROPERTY(bool isConnected READ isConnected NOTIFY isConnectedChanged)
    Q_PROPERTY(int currentMode READ currentMode WRITE setCurrentMode NOTIFY currentModeChanged)
    Q_PROPERTY(bool isEmergencyStopped READ isEmergencyStopped WRITE setEmergencyStopped NOTIFY emergencyStoppedChanged)

    // 大地坐标与高程 (m)
    Q_PROPERTY(double posX READ posX NOTIFY poseChanged)
    Q_PROPERTY(double posY READ posY NOTIFY poseChanged)
    Q_PROPERTY(double posZ READ posZ NOTIFY poseChanged)

    // 姿态欧拉角 (度)
    Q_PROPERTY(double yawDeg READ yawDeg NOTIFY poseChanged)
    Q_PROPERTY(double pitchDeg READ pitchDeg NOTIFY poseChanged)
    Q_PROPERTY(double rollDeg READ rollDeg NOTIFY poseChanged)

    // 速度指标
    Q_PROPERTY(double linearVelocity READ linearVelocity NOTIFY velocityChanged)
    Q_PROPERTY(double angularVelocity READ angularVelocity NOTIFY velocityChanged)
    Q_PROPERTY(double maxLinearVelocity READ maxLinearVelocity WRITE setMaxLinearVelocity NOTIFY settingsChanged)
    Q_PROPERTY(double steeringSensitivity READ steeringSensitivity WRITE setSteeringSensitivity NOTIFY settingsChanged)

    // 电源状态
    Q_PROPERTY(int batteryPct READ batteryPct NOTIFY batteryChanged)
    Q_PROPERTY(double batteryVolt READ batteryVolt NOTIFY batteryChanged)

    // 实时方向意图反馈 ("up", "down", "left", "right", "stop")
    Q_PROPERTY(QString activeDirection READ activeDirection NOTIFY activeDirectionChanged)

    // 自主导航目标与状态
    Q_PROPERTY(double navGoalX READ navGoalX NOTIFY navGoalChanged)
    Q_PROPERTY(double navGoalY READ navGoalY NOTIFY navGoalChanged)
    Q_PROPERTY(bool hasNavGoal READ hasNavGoal NOTIFY navGoalChanged)
    Q_PROPERTY(QString navState READ navState NOTIFY navStateChanged)

    // 激光雷达实测数据
    Q_PROPERTY(QVariantList lidarRanges READ lidarRanges NOTIFY lidarDataChanged)
    Q_PROPERTY(QVariantList localElevationGrid READ localElevationGrid NOTIFY lidarDataChanged)
    Q_PROPERTY(int lidarPointCount READ lidarPointCount NOTIFY lidarDataChanged)
    Q_PROPERTY(bool lidarConnected READ lidarConnected NOTIFY lidarDataChanged)
    Q_PROPERTY(double closestObstacleDist READ closestObstacleDist NOTIFY lidarDataChanged)
    Q_PROPERTY(double closestObstacleAngle READ closestObstacleAngle NOTIFY lidarDataChanged)
    Q_PROPERTY(bool hasObstacle READ hasObstacle NOTIFY lidarDataChanged)

    // 2.5D DEM 地图产物路径
    Q_PROPERTY(QString mapTextureUrl READ mapTextureUrl CONSTANT)
    Q_PROPERTY(QString mapHeatmapUrl READ mapHeatmapUrl CONSTANT)

public:
    explicit MshClient(QObject *parent = nullptr);
    ~MshClient() override;

    bool isConnected() const { return m_isConnected; }
    int currentMode() const { return m_currentMode; }
    void setCurrentMode(int mode);

    bool isEmergencyStopped() const { return m_isEmergencyStopped; }
    void setEmergencyStopped(bool stopped);

    double posX() const { return m_posX; }
    double posY() const { return m_posY; }
    double posZ() const { return m_posZ; }
    double yawDeg() const { return m_yawDeg; }
    double pitchDeg() const { return m_pitchDeg; }
    double rollDeg() const { return m_rollDeg; }

    double linearVelocity() const { return m_linearVelocity; }
    double angularVelocity() const { return m_angularVelocity; }

    double maxLinearVelocity() const { return m_maxLinearVelocity; }
    void setMaxLinearVelocity(double val);

    double steeringSensitivity() const { return m_steeringSensitivity; }
    void setSteeringSensitivity(double val);

    int batteryPct() const { return m_batteryPct; }
    double batteryVolt() const { return m_batteryVolt; }

    QString activeDirection() const { return m_activeDirection; }

    double navGoalX() const { return m_navGoalX; }
    double navGoalY() const { return m_navGoalY; }
    bool hasNavGoal() const { return m_hasNavGoal; }
    QString navState() const { return m_navState; }

    QVariantList lidarRanges() const { return m_lidarRanges; }
    QVariantList localElevationGrid() const { return m_localElevationGrid; }
    int lidarPointCount() const { return m_lidarPointCount; }
    bool lidarConnected() const { return m_lidarConnected; }

    double closestObstacleDist() const { return m_closestObstacleDist; }
    double closestObstacleAngle() const { return m_closestObstacleAngle; }
    bool hasObstacle() const { return m_hasObstacle; }

    QString mapTextureUrl() const;
    QString mapHeatmapUrl() const;

public Q_SLOTS:
    // 遥控与控制接口
    void drive(double linearX, double angularZ);
    void sendDirectionCommand(const QString &dir);
    void emergencyStop();
    void releaseBrake();
    void setMode(int mode);
    void navigateTo(double goalX, double goalY);
    void cancelNavigation();
    void startMapping();
    void stopMapping();
    void startAutoExploration();

Q_SIGNALS:
    void isConnectedChanged();
    void currentModeChanged(int mode);
    void emergencyStoppedChanged(bool stopped);
    void poseChanged();
    void velocityChanged();
    void settingsChanged();
    void batteryChanged();
    void activeDirectionChanged();
    void navGoalChanged();
    void navStateChanged();
    void statusMessageReceived(const QString &message);
    void lidarDataChanged();

private Q_SLOTS:
    void onConnected();
    void onDisconnected();
    void onReadyRead();
    void onPollTimerTimeout();

private:
    void sendRpcRequest(const QString &method, const QJsonObject &params = QJsonObject());
    void handleRpcResponse(const QJsonObject &resp);
    void parseFullTelemetry(const QJsonObject &result);
    void updateActiveDirection(double v, double w);

    QTcpSocket *m_socket = nullptr;
    QTimer *m_pollTimer = nullptr;
    QByteArray m_readBuffer;

    QString m_host = "127.0.0.1";
    quint16 m_port = 9001;
    qint64 m_rpcId = 0;

    bool m_isConnected = false;
    int m_currentMode = 0; // 0: 手动遥控, 1: 实时建图, 2: 自主导航
    bool m_isEmergencyStopped = false;

    double m_posX = 0.0;
    double m_posY = 0.0;
    double m_posZ = 0.0;
    double m_yawDeg = 0.0;
    double m_pitchDeg = 0.0;
    double m_rollDeg = 0.0;

    double m_linearVelocity = 0.0;
    double m_angularVelocity = 0.0;
    double m_maxLinearVelocity = 1.20;
    double m_steeringSensitivity = 75.0;

    int m_batteryPct = 100;
    double m_batteryVolt = 48.0;
    QString m_activeDirection = "";

    double m_navGoalX = 0.0;
    double m_navGoalY = 0.0;
    bool m_hasNavGoal = false;
    QString m_navState = "IDLE";

    // 真实激光雷达数据
    QVariantList m_lidarRanges;
    QVariantList m_localElevationGrid;
    int m_lidarPointCount = 0;
    bool m_lidarConnected = false;
    double m_closestObstacleDist = 0.0;
    double m_closestObstacleAngle = 0.0;
    bool m_hasObstacle = false;
};
