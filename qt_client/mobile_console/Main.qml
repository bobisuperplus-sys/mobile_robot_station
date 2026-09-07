import QtQuick
import QtQuick.Layouts
import QtQuick.Controls.Basic
import mobile_console
import "components"
import "views"

ApplicationWindow {
    id: window

    width: 1440
    height: 900
    minimumWidth: 1200
    minimumHeight: 760
    visible: true
    title: qsTr("移动机器人地面监控工作站 - UGV-TWIN OS v4.2")
    color: Theme.bgCanvas

    property int initialMode: 0
    property int initialViewTab: 0

    // 全局 MSH 微服务客户端数据中枢
    MshClient {
        id: mshClient
        currentMode: window.initialMode
        onStatusMessageReceived: function(msg) {
            consoleFooter.addLog("SYSTEM", msg, "[MSH]", 0);
        }
    }

    // 键盘快捷键监听 (处于手动控制模式时，支持键盘按键 W/A/S/D 与空格急停)
    Item {
        anchors.fill: parent
        focus: true

        Keys.onPressed: function(event) {
            if (mshClient.currentMode === 2 || mshClient.isEmergencyStopped)
                return;

            if (event.key === Qt.Key_W) {
                mshClient.sendDirectionCommand("up");
                event.accepted = true;
            } else if (event.key === Qt.Key_S) {
                mshClient.sendDirectionCommand("down");
                event.accepted = true;
            } else if (event.key === Qt.Key_A) {
                mshClient.sendDirectionCommand("left");
                event.accepted = true;
            } else if (event.key === Qt.Key_D) {
                mshClient.sendDirectionCommand("right");
                event.accepted = true;
            } else if (event.key === Qt.Key_Space) {
                mshClient.emergencyStop();
                consoleFooter.addLog("CRITICAL", "键盘快捷空格键触发紧急制动！底盘驻车切断动力", "[TELEOP]", 2);
                event.accepted = true;
            }
        }

        Keys.onReleased: function(event) {
            if (mshClient.currentMode === 2 || mshClient.isEmergencyStopped)
                return;

            if (event.key === Qt.Key_W || event.key === Qt.Key_S || event.key === Qt.Key_A || event.key === Qt.Key_D) {
                mshClient.sendDirectionCommand("stop");
                event.accepted = true;
            }
        }
    }

    ColumnLayout {
        anchors.fill: parent
        spacing: 0

        // 1. 顶部全局系统栏 (三段模式与 MSH 在线状态双向联动)
        TopHeader {
            id: topHeader
            Layout.fillWidth: true
            currentMode: mshClient.currentMode
            isConnected: mshClient.isConnected
            onModeChanged: function(mode) {
                if (mode === 1) {
                    mshClient.startMapping();
                } else if (mode === 2) {
                    mshClient.setMode(2);
                } else {
                    mshClient.stopMapping();
                }
                var modeName = (mode === 2 ? "自主导航 (AUTONOMOUS NAV)" : (mode === 1 ? "实时建图 (3D SLAM 增量探索)" : "手动遥控 (MANUAL)"));
                consoleFooter.addLog("INFO", "全局运行模式已变更为: " + modeName, "[MODE]", 1);
            }
            onEmergencyStopTriggered: function() {
                mshClient.emergencyStop();
                consoleFooter.addLog("CRITICAL", "系统顶栏触发紧急制动！底盘驻车动力切断", "[SAFETY]", 2);
            }
        }

        // 2. 主区域与底栏之间的纵向分割 (SplitView Vertical)
        SplitView {
            id: mainVerticalSplit
            Layout.fillWidth: true
            Layout.fillHeight: true
            Layout.leftMargin: 10
            Layout.rightMargin: 10
            Layout.topMargin: 8
            Layout.bottomMargin: 8
            orientation: Qt.Vertical
            handle: SplitHandleBar { orientation: Qt.Vertical }

            // 2.1 中央三列工作区横向分割 (SplitView Horizontal)
            SplitView {
                id: centralHorizontalSplit
                SplitView.fillWidth: true
                SplitView.fillHeight: true
                orientation: Qt.Horizontal
                handle: SplitHandleBar { orientation: Qt.Horizontal }

                // 2.1.1 左区: 双机位视讯监控 (内部已为 SplitView Vertical)
                CameraPanel {
                    id: cameraPanel
                    SplitView.preferredWidth: 340
                    SplitView.minimumWidth: 260
                    SplitView.maximumWidth: 500
                    SplitView.fillHeight: true
                }

                // 2.1.2 中区: 2.5D DEM 空间态势立体高程主工作区
                SpatialDemView {
                    id: spatialDemView
                    SplitView.fillWidth: true
                    SplitView.minimumWidth: 460
                    SplitView.fillHeight: true
                    currentViewTab: window.initialViewTab
                    mshClient: mshClient

                    onNavigateRequested: function(gx, gy) {
                        mshClient.navigateTo(gx, gy);
                        consoleFooter.addLog("CONTROL", "地图交互选点下发自主寻路: (" + gx.toFixed(2) + ", " + gy.toFixed(2) + ")", "[NAV_GOAL]", 3);
                    }
                }

                // 2.1.3 右区: 任务编排、动力学遥测与底盘控制 (内部使用 SplitView Vertical)
                SplitView {
                    id: rightControlSplit
                    SplitView.preferredWidth: 340
                    SplitView.minimumWidth: 280
                    SplitView.maximumWidth: 500
                    SplitView.fillHeight: true
                    orientation: Qt.Vertical
                    handle: SplitHandleBar { orientation: Qt.Vertical }

                    // 动力学姿态仪表
                    TelemetryPanel {
                        id: telemetryPanel
                        SplitView.preferredHeight: 188
                        SplitView.minimumHeight: 140
                        SplitView.maximumHeight: 260
                        SplitView.fillWidth: true
                        posX: mshClient.posX
                        posY: mshClient.posY
                        posZ: mshClient.posZ
                        pitchDeg: mshClient.pitchDeg
                        rollDeg: mshClient.rollDeg
                        linVel: mshClient.linearVelocity
                        angVel: mshClient.angularVelocity
                        batteryPct: mshClient.batteryPct
                        batteryVolt: mshClient.batteryVolt
                        isEmergencyStopped: mshClient.isEmergencyStopped
                        isConnected: mshClient.isConnected
                    }

                    // 航点与兴趣点 (POI)
                    WaypointPanel {
                        id: waypointPanel
                        SplitView.fillHeight: true
                        SplitView.minimumHeight: 120
                        SplitView.fillWidth: true

                        onWaypointSelected: function(index) {
                            var goals = [
                                [0.00, -6.00], // 1. 起始整备区
                                [-2.50, 0.00], // 2. 环岛西侧通道
                                [0.00, 2.40],  // 3. 登坡入口准备
                                [0.00, 6.00]   // 4. 坡顶立体观景台
                            ];
                            if (index >= 0 && index < goals.length) {
                                var g = goals[index];
                                mshClient.navigateTo(g[0], g[1]);
                                consoleFooter.addLog("CONTROL", "下发自主寻路指令至航点 #" + (index + 1) + ": (" + g[0].toFixed(2) + ", " + g[1].toFixed(2) + ")", "[NAV_GOAL]", 3);
                            }
                        }

                        onSaveCurrentPoseRequested: function() {
                            consoleFooter.addLog("INFO", "已收藏当前实时位姿: (" + mshClient.posX.toFixed(2) + ", " + mshClient.posY.toFixed(2) + ", " + mshClient.posZ.toFixed(2) + ")", "[WAYPOINT]", 1);
                        }
                    }

                    // 手控方向舵与速度限值
                    MotionPanel {
                        id: motionPanel
                        SplitView.preferredHeight: 188
                        SplitView.minimumHeight: 140
                        SplitView.maximumHeight: 250
                        SplitView.fillWidth: true
                        currentMode: mshClient.currentMode
                        isEmergencyStopped: mshClient.isEmergencyStopped
                        activeDirection: mshClient.activeDirection
                        linearVelocity: mshClient.linearVelocity
                        angularVelocity: mshClient.angularVelocity
                        maxLinearVelocity: mshClient.maxLinearVelocity
                        steeringSensitivity: mshClient.steeringSensitivity

                        onModeSwitchRequested: function(newMode) {
                            mshClient.setMode(newMode);
                            var isAuto = (newMode === 2);
                            consoleFooter.addLog("INFO", isAuto ? "自主导航托管已激活：手控锁定，方向盘转为算法实时意图指示" : "手控托管已释放：恢复鼠标/WASD 手动驾驶权限", "[AUTOPILOT]", 1);
                        }
                        onMoveCommand: function(direction) {
                            mshClient.sendDirectionCommand(direction);
                            consoleFooter.addLog("CONTROL", "下发手动操控方向: " + direction.toUpperCase(), "[TELEOP]", 3);
                        }
                        onStopCommand: function() {
                            if (mshClient.isEmergencyStopped) {
                                mshClient.releaseBrake();
                                consoleFooter.addLog("INFO", "已解除驻车制动锁定，恢复待机", "[BRAKE]", 1);
                            } else {
                                mshClient.emergencyStop();
                                consoleFooter.addLog("CRITICAL", "触发底盘制动停机！切断动力", "[SAFETY]", 2);
                            }
                        }
                        onSettingsChanged: function(maxV, sens) {
                            mshClient.maxLinearVelocity = maxV;
                            mshClient.steeringSensitivity = sens;
                        }
                    }
                }
            }

            // 2.2 底部系统事件日志控制台与状态栏
            ConsoleLogFooter {
                id: consoleFooter
                SplitView.preferredHeight: 110
                SplitView.minimumHeight: 52
                SplitView.maximumHeight: 360
                SplitView.fillWidth: true
                posX: mshClient.posX
                posY: mshClient.posY
                posZ: mshClient.posZ
                yawDeg: mshClient.yawDeg
                isConnected: mshClient.isConnected
            }
        }
    }
}
