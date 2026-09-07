#include "msh_client.h"
#include <QDebug>
#include <QDir>
#include <QFileInfo>
#include <cmath>

MshClient::MshClient(QObject *parent)
    : QObject(parent)
{
    m_socket = new QTcpSocket(this);

    connect(m_socket, &QTcpSocket::connected, this, &MshClient::onConnected);
    connect(m_socket, &QTcpSocket::disconnected, this, &MshClient::onDisconnected);
    connect(m_socket, &QTcpSocket::readyRead, this, &MshClient::onReadyRead);

    // 100ms 周期轮询 MSH 遥测状态
    m_pollTimer = new QTimer(this);
    m_pollTimer->setInterval(100);
    connect(m_pollTimer, &QTimer::timeout, this, &MshClient::onPollTimerTimeout);
    m_pollTimer->start();

    // 初始尝试连接
    m_socket->connectToHost(m_host, m_port);
}

MshClient::~MshClient()
{
    if (m_socket && m_socket->isOpen()) {
        m_socket->disconnectFromHost();
    }
}

void MshClient::setCurrentMode(int mode)
{
    if (m_currentMode == mode)
        return;

    m_currentMode = mode;
    Q_EMIT currentModeChanged(m_currentMode);

    if (mode == 2) {
        m_linearVelocity = 0.0;
        m_angularVelocity = 0.0;
        m_activeDirection = "stop";
        Q_EMIT velocityChanged();
        Q_EMIT activeDirectionChanged();
    }

    if (m_isConnected) {
        QString modeStr = (mode == 2) ? "NAVIGATION" : (mode == 1 ? "MAPPING" : "MANUAL");
        QJsonObject params;
        params["mode"] = modeStr;
        sendRpcRequest("system.set_mode", params);
    }
}

void MshClient::setEmergencyStopped(bool stopped)
{
    if (m_isEmergencyStopped == stopped)
        return;

    m_isEmergencyStopped = stopped;
    Q_EMIT emergencyStoppedChanged(m_isEmergencyStopped);

    if (m_isEmergencyStopped) {
        emergencyStop();
    }
}

void MshClient::setMaxLinearVelocity(double val)
{
    if (qFuzzyCompare(m_maxLinearVelocity, val))
        return;

    m_maxLinearVelocity = val;
    Q_EMIT settingsChanged();
}

void MshClient::setSteeringSensitivity(double val)
{
    if (qFuzzyCompare(m_steeringSensitivity, val))
        return;

    m_steeringSensitivity = val;
    Q_EMIT settingsChanged();
}

void MshClient::drive(double linearX, double angularZ)
{
    if (m_isEmergencyStopped) {
        qWarning() << "[MshClient] 当前处于急停状态，拒绝执行驱动指令";
        return;
    }

    m_linearVelocity = linearX;
    m_angularVelocity = angularZ;
    Q_EMIT velocityChanged();

    updateActiveDirection(linearX, angularZ);

    if (m_isConnected) {
        QJsonObject params;
        params["linear_x"] = linearX;
        params["angular_z"] = angularZ;
        sendRpcRequest("teleop.drive", params);
    }
}

void MshClient::sendDirectionCommand(const QString &dir)
{
    if (m_isEmergencyStopped)
        return;

    // 自主导航接管中禁止手工点击下发
    if (m_currentMode == 2)
        return;

    double maxV = m_maxLinearVelocity;
    double maxW = (m_steeringSensitivity / 100.0) * 2.5;

    if (dir == "up") {
        drive(maxV, 0.0);
    } else if (dir == "down") {
        drive(-maxV * 0.6, 0.0);
    } else if (dir == "left") {
        drive(0.0, maxW);
    } else if (dir == "right") {
        drive(0.0, -maxW);
    } else if (dir == "stop") {
        drive(0.0, 0.0);
    }
}

void MshClient::emergencyStop()
{
    m_isEmergencyStopped = true;
    m_linearVelocity = 0.0;
    m_angularVelocity = 0.0;
    m_activeDirection = "stop";

    Q_EMIT emergencyStoppedChanged(true);
    Q_EMIT velocityChanged();
    Q_EMIT activeDirectionChanged();

    // 急停强制退出自主导航并重置为手动
    if (m_currentMode != 0) {
        m_currentMode = 0;
        Q_EMIT currentModeChanged(0);
    }

    if (m_isConnected) {
        sendRpcRequest("teleop.emergency_stop");
        QJsonObject params;
        params["mode"] = "MANUAL";
        sendRpcRequest("system.set_mode", params);
    }
}

void MshClient::releaseBrake()
{
    if (!m_isEmergencyStopped)
        return;

    m_isEmergencyStopped = false;
    Q_EMIT emergencyStoppedChanged(false);
}

void MshClient::setMode(int mode)
{
    setCurrentMode(mode);
}

void MshClient::navigateTo(double goalX, double goalY)
{
    if (m_isEmergencyStopped)
        return;

    m_navGoalX = goalX;
    m_navGoalY = goalY;
    m_hasNavGoal = true;
    m_navState = "NAVIGATING";
    Q_EMIT navGoalChanged();
    Q_EMIT navStateChanged();

    setCurrentMode(2); // 自动切换至自主导航

    if (m_isConnected) {
        QJsonObject params;
        params["goal_x"] = goalX;
        params["goal_y"] = goalY;
        sendRpcRequest("nav.navigate_to", params);
    }
}

void MshClient::cancelNavigation()
{
    m_hasNavGoal = false;
    m_navState = "IDLE";
    Q_EMIT navGoalChanged();
    Q_EMIT navStateChanged();

    setCurrentMode(0); // 切回手动

    if (m_isConnected) {
        sendRpcRequest("nav.cancel_goal");
    }
}

void MshClient::startMapping()
{
    setCurrentMode(1); // 切换至建图模式
    if (m_isConnected) {
        sendRpcRequest("slam.start");
        Q_EMIT statusMessageReceived("已启动在线增量 SLAM 实时建图与探索模式");
    }
}

void MshClient::stopMapping()
{
    setCurrentMode(0); // 切回手动模式
    if (m_isConnected) {
        sendRpcRequest("slam.stop");
        Q_EMIT statusMessageReceived("已停止实时建图并转入手动控制模式");
    }
}

void MshClient::startAutoExploration()
{
    startMapping();
    if (m_isConnected) {
        QJsonObject params;
        params["auto_explore"] = true;
        sendRpcRequest("slam.start", params);
        Q_EMIT statusMessageReceived("已启动未知区域全地形自主探索建图序列");
    }
}

void MshClient::onConnected()
{
    m_isConnected = true;
    Q_EMIT isConnectedChanged();
    Q_EMIT statusMessageReceived("已成功建立与 MSH 主控微服务总线 (Port: 9001) 的长连接");

    if (m_currentMode != 0) {
        QString modeStr = (m_currentMode == 2) ? "NAVIGATION" : "MAPPING";
        QJsonObject params;
        params["mode"] = modeStr;
        sendRpcRequest("system.set_mode", params);
    }
}

void MshClient::onDisconnected()
{
    m_isConnected = false;
    m_linearVelocity = 0.0;
    m_angularVelocity = 0.0;
    m_activeDirection = "";
    m_lidarConnected = false;
    m_lidarPointCount = 0;
    m_lidarRanges.clear();
    m_localElevationGrid.clear();

    Q_EMIT isConnectedChanged();
    Q_EMIT velocityChanged();
    Q_EMIT activeDirectionChanged();
    Q_EMIT lidarDataChanged();
    Q_EMIT statusMessageReceived("与 MSH 主控微服务总线断开，系统已安全进入脱机待命状态");
}

void MshClient::onPollTimerTimeout()
{
    if (m_isConnected) {
        // 请求全量遥测状态
        sendRpcRequest("telemetry.get_full_status");
    } else {
        // 断线自愈重连
        if (m_socket->state() == QAbstractSocket::UnconnectedState) {
            m_socket->connectToHost(m_host, m_port);
        }
    }
}

void MshClient::onReadyRead()
{
    m_readBuffer.append(m_socket->readAll());

    while (m_readBuffer.contains('\n')) {
        int newlineIdx = m_readBuffer.indexOf('\n');
        QByteArray line = m_readBuffer.left(newlineIdx).trimmed();
        m_readBuffer.remove(0, newlineIdx + 1);

        if (line.isEmpty())
            continue;

        QJsonParseError err;
        QJsonDocument doc = QJsonDocument::fromJson(line, &err);
        if (err.error == QJsonParseError::NoError && doc.isObject()) {
            handleRpcResponse(doc.object());
        }
    }
}

void MshClient::sendRpcRequest(const QString &method, const QJsonObject &params)
{
    if (!m_isConnected || !m_socket || !m_socket->isOpen())
        return;

    m_rpcId++;
    QJsonObject req;
    req["jsonrpc"] = "2.0";
    req["method"] = method;
    req["params"] = params;
    req["id"] = m_rpcId;

    QByteArray data = QJsonDocument(req).toJson(QJsonDocument::Compact) + "\n";
    m_socket->write(data);
}

void MshClient::handleRpcResponse(const QJsonObject &resp)
{
    if (resp.contains("error") && !resp["error"].isNull()) {
        QJsonObject err = resp["error"].toObject();
        qWarning() << "[MshClient] RPC 错误:" << err["message"].toString();
        return;
    }

    if (!resp.contains("result"))
        return;

    QJsonObject result = resp["result"].toObject();
    // 全量遥测解析
    if (result.contains("pose")) {
        parseFullTelemetry(result);
    }
}

void MshClient::parseFullTelemetry(const QJsonObject &result)
{
    QJsonObject pose = result["pose"].toObject();
    if (pose.contains("position")) {
        QJsonArray posArr = pose["position"].toArray();
        if (posArr.size() >= 3) {
            m_posX = posArr[0].toDouble();
            m_posY = posArr[1].toDouble();
            m_posZ = posArr[2].toDouble();
        }
    }

    if (pose.contains("yaw_deg")) {
        m_yawDeg = pose["yaw_deg"].toDouble();
    }
    if (pose.contains("pitch_deg")) {
        m_pitchDeg = pose["pitch_deg"].toDouble();
    }
    if (pose.contains("roll_deg")) {
        m_rollDeg = pose["roll_deg"].toDouble();
    }
    Q_EMIT poseChanged();

    // 速度与按键意图解析
    if (result.contains("velocity")) {
        QJsonObject vel = result["velocity"].toObject();
        m_linearVelocity = vel["linear"].toDouble();
        m_angularVelocity = vel["angular"].toDouble();
        Q_EMIT velocityChanged();

        updateActiveDirection(m_linearVelocity, m_angularVelocity);
    }

    // 模式同步
    if (result.contains("mode")) {
        QString modeStr = result["mode"].toString();
        int modeVal = (modeStr == "NAVIGATION") ? 2 : ((modeStr == "MAPPING") ? 1 : 0);
        if (m_currentMode != modeVal) {
            m_currentMode = modeVal;
            Q_EMIT currentModeChanged(m_currentMode);
        }
    }

    // 自主导航状态与目标同步
    if (result.contains("navigation")) {
        QJsonObject navObj = result["navigation"].toObject();
        if (navObj.contains("state")) {
            QString state = navObj["state"].toString();
            if (m_navState != state) {
                m_navState = state;
                Q_EMIT navStateChanged();
            }
            // 目标已到达或取消，注销目标星标
            if (state == "ARRIVED" || state == "CANCELLED") {
                if (m_hasNavGoal) {
                    m_hasNavGoal = false;
                    Q_EMIT navGoalChanged();
                }
            }
        }
        if (navObj.contains("goal") && m_navState != "ARRIVED" && m_navState != "CANCELLED") {
            QJsonArray goalArr = navObj["goal"].toArray();
            if (goalArr.size() >= 2) {
                double gx = goalArr[0].toDouble();
                double gy = goalArr[1].toDouble();
                if (std::abs(m_navGoalX - gx) > 0.01 || std::abs(m_navGoalY - gy) > 0.01 || !m_hasNavGoal) {
                    m_navGoalX = gx;
                    m_navGoalY = gy;
                    m_hasNavGoal = true;
                    Q_EMIT navGoalChanged();
                }
            }
        }
    }

    // 激光雷达实测遥测数据解析
    if (result.contains("lidar")) {
        QJsonObject lidarObj = result["lidar"].toObject();
        m_lidarConnected = lidarObj["connected"].toBool(false);
        m_lidarPointCount = lidarObj["points_count"].toInt(0);

        if (lidarObj.contains("ranges")) {
            QJsonArray rArr = lidarObj["ranges"].toArray();
            QVariantList rangesList;
            rangesList.reserve(rArr.size());
            for (const auto &val : rArr) {
                if (val.isNull()) {
                    rangesList.append(QVariant()); // 真实开阔空旷方向
                } else {
                    rangesList.append(val.toDouble());
                }
            }
            m_lidarRanges = rangesList;
        }

        if (lidarObj.contains("closest_obstacle")) {
            QJsonObject obsObj = lidarObj["closest_obstacle"].toObject();
            if (!obsObj["distance"].isNull()) {
                m_closestObstacleDist = obsObj["distance"].toDouble();
                m_closestObstacleAngle = obsObj["angle_deg"].toDouble();
                m_hasObstacle = true;
            } else {
                m_closestObstacleDist = 0.0;
                m_closestObstacleAngle = 0.0;
                m_hasObstacle = false;
            }
        } else {
            m_hasObstacle = false;
        }

        if (lidarObj.contains("local_elevation")) {
            QJsonArray eArr = lidarObj["local_elevation"].toArray();
            QVariantList elevList;
            elevList.reserve(eArr.size());
            for (const auto &val : eArr) {
                if (val.isNull()) {
                    elevList.append(QVariant()); // 空未探测栅格
                } else {
                    elevList.append(val.toDouble());
                }
            }
            m_localElevationGrid = elevList;
        }

        Q_EMIT lidarDataChanged();
    }
}

void MshClient::updateActiveDirection(double v, double w)
{
    QString dir = "";
    if (v > 0.05) {
        dir = "up";
    } else if (v < -0.05) {
        dir = "down";
    } else if (w > 0.08) {
        dir = "left";
    } else if (w < -0.08) {
        dir = "right";
    } else if (std::abs(v) < 0.03 && std::abs(w) < 0.05) {
        dir = "stop";
    }

    if (m_activeDirection != dir) {
        m_activeDirection = dir;
        Q_EMIT activeDirectionChanged();
    }
}

QString MshClient::mapTextureUrl() const
{
    QString standardPath = "/home/yellowtown/Code/RobotProject/mobile_robot_station/maps/urban_world_default/elevation_texture.png";
    if (QFileInfo::exists(standardPath)) {
        return "file://" + standardPath;
    }
    // 相对路径备选
    QString relPath = QDir::current().absoluteFilePath("maps/urban_world_default/elevation_texture.png");
    if (QFileInfo::exists(relPath)) {
        return "file://" + relPath;
    }
    return "file://" + standardPath;
}

QString MshClient::mapHeatmapUrl() const
{
    QString standardPath = "/home/yellowtown/Code/RobotProject/mobile_robot_station/maps/urban_world_default/elevation_heatmap.png";
    if (QFileInfo::exists(standardPath)) {
        return "file://" + standardPath;
    }
    // 相对路径备选
    QString relPath = QDir::current().absoluteFilePath("maps/urban_world_default/elevation_heatmap.png");
    if (QFileInfo::exists(relPath)) {
        return "file://" + relPath;
    }
    return "file://" + standardPath;
}

