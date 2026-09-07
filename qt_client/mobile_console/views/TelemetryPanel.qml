import QtQuick
import QtQuick.Layouts
import QtQuick.Controls.Basic
import mobile_console
import "../components"

CardPanel {
    id: root

    cardRadius: Theme.radiusLg
    cardColor: Theme.bgSurface
    borderColor: Theme.borderBase
    implicitHeight: 188

    // 动力学数据源属性 (可外部绑定至 MshClient)
    property real posX: 0.0
    property real posY: 0.0
    property real posZ: 0.0
    property real pitchDeg: 0.0
    property real rollDeg: 0.0
    property real linVel: 0.0
    property real angVel: 0.0
    property int batteryPct: 100
    property real batteryVolt: 48.0
    property bool isEmergencyStopped: false
    property bool isConnected: false

    content: [
        ColumnLayout {
            anchors.fill: parent
            anchors.margins: 12
            spacing: 8

            // 1. 标题行与制动状态徽章
            RowLayout {
                Layout.fillWidth: true

                Row {
                    spacing: 6
                    Rectangle {
                        width: 28
                        height: 16
                        radius: 3
                        color: "#16253b"
                        border.color: "#3e90ff4d"
                        border.width: 1
                        anchors.verticalCenter: parent.verticalCenter
                        Text {
                            anchors.centerIn: parent
                            text: "IMU"
                            font.family: Theme.fontFamilyMono
                            font.bold: true
                            font.pixelSize: 9
                            color: Theme.lightBlue
                        }
                    }
                    Text {
                        text: "动力学姿态仪表"
                        font.family: Theme.fontFamilyNormal
                        font.pixelSize: 11
                        font.bold: true
                        color: Theme.textPrimary
                        anchors.verticalCenter: parent.verticalCenter
                    }
                }

                Item { Layout.fillWidth: true }

                StatusBadge {
                    text: !root.isConnected ? "服务离线 (OFFLINE)" : (root.isEmergencyStopped ? "紧急制动 (BRAKE)" : "制动释放 (READY)")
                    textColor: !root.isConnected ? Theme.textMuted : (root.isEmergencyStopped ? Theme.errorRedLight : Theme.successGreen)
                    badgeColor: !root.isConnected ? "#141720" : (root.isEmergencyStopped ? Theme.errorBg : "#10281b")
                    borderColor: !root.isConnected ? Theme.borderBase : (root.isEmergencyStopped ? "#ffb4ab80" : "#34c7594d")
                    showDot: true
                    dotColor: !root.isConnected ? Theme.textMuted : (root.isEmergencyStopped ? Theme.errorRedLight : Theme.successGreen)
                    dotPulse: root.isEmergencyStopped
                    fontSize: 9
                    height: 18
                }
            }

            Rectangle { Layout.fillWidth: true; height: 1; color: "#242428" }

            // 2. 三列大地坐标卡片
            RowLayout {
                Layout.fillWidth: true
                spacing: 6

                Repeater {
                    model: [
                        { label: "大地坐标 X", val: root.isConnected ? root.posX.toFixed(2) : "--", unit: "m" },
                        { label: "大地坐标 Y", val: root.isConnected ? root.posY.toFixed(2) : "--", unit: "m" },
                        { label: "高程 Z", val: root.isConnected ? root.posZ.toFixed(2) : "--", unit: "m" }
                    ]
                    delegate: Rectangle {
                        Layout.fillWidth: true
                        height: 48
                        radius: Theme.radiusSm
                        color: Theme.bgContainer
                        border.color: Theme.borderSubtle
                        border.width: 1

                        ColumnLayout {
                            anchors.fill: parent
                            anchors.margins: 6
                            spacing: 1

                            Text {
                                text: modelData.label
                                font.family: Theme.fontFamilyMono
                                font.pixelSize: 9
                                color: Theme.textMuted
                            }

                            Row {
                                spacing: 3
                                Text {
                                    text: modelData.val
                                    font.family: Theme.fontFamilyMono
                                    font.pixelSize: 13
                                    font.bold: true
                                    color: Theme.textPrimary
                                }
                                Text {
                                    text: modelData.unit
                                    font.family: Theme.fontFamilyMono
                                    font.pixelSize: 9
                                    color: Theme.textMuted
                                    anchors.bottom: parent.bottom
                                    anchors.bottomMargin: 1
                                }
                            }
                        }
                    }
                }
            }

            // 3. 四列动态参数卡片 (俯仰坡度、翻滚倾角、实时线速、实时角速)
            RowLayout {
                Layout.fillWidth: true
                spacing: 6

                // PITCH 俯仰角 (带坡度状态与自适应预警色)
                Rectangle {
                    Layout.fillWidth: true
                    height: 46
                    radius: Theme.radiusSm
                    color: Theme.bgContainer
                    border.color: Theme.borderSubtle
                    border.width: 1

                    ColumnLayout {
                        anchors.fill: parent
                        anchors.margins: 4
                        spacing: 1

                        Row {
                            spacing: 4
                            Text {
                                text: "PITCH"
                                font.family: Theme.fontFamilyMono
                                font.pixelSize: 8
                                color: Theme.textMuted
                            }
                            Text {
                                text: Math.abs(root.pitchDeg) <= 3.0 ? "平地" : (Math.abs(root.pitchDeg) <= 8.0 ? "缓坡" : "陡坡")
                                font.family: Theme.fontFamilyNormal
                                font.pixelSize: 8
                                color: Math.abs(root.pitchDeg) <= 3.0 ? Theme.successGreen : (Math.abs(root.pitchDeg) <= 8.0 ? Theme.lightBlue : Theme.warningAmber)
                            }
                        }

                        Text {
                            text: !root.isConnected ? "--" : ((root.pitchDeg >= 0 ? "+" : "") + root.pitchDeg.toFixed(1) + "°")
                            font.family: Theme.fontFamilyMono
                            font.pixelSize: 11
                            font.bold: true
                            color: !root.isConnected ? Theme.textMuted : (Math.abs(root.pitchDeg) <= 3.0 ? Theme.successGreen : (Math.abs(root.pitchDeg) <= 8.0 ? Theme.lightBlue : Theme.warningAmber))
                        }
                    }
                }

                // ROLL 横滚倾角
                Rectangle {
                    Layout.fillWidth: true
                    height: 46
                    radius: Theme.radiusSm
                    color: Theme.bgContainer
                    border.color: Theme.borderSubtle
                    border.width: 1

                    ColumnLayout {
                        anchors.fill: parent
                        anchors.margins: 4
                        spacing: 1

                        Text {
                            text: "ROLL"
                            font.family: Theme.fontFamilyMono
                            font.pixelSize: 8
                            color: Theme.textMuted
                        }
                        Text {
                            text: !root.isConnected ? "--" : ((root.rollDeg >= 0 ? "+" : "") + root.rollDeg.toFixed(1) + "°")
                            font.family: Theme.fontFamilyMono
                            font.pixelSize: 11
                            color: !root.isConnected ? Theme.textMuted : Theme.textPrimary
                        }
                    }
                }

                // 实时线速 v
                Rectangle {
                    Layout.fillWidth: true
                    height: 46
                    radius: Theme.radiusSm
                    color: Theme.bgContainer
                    border.color: Theme.borderSubtle
                    border.width: 1

                    ColumnLayout {
                        anchors.fill: parent
                        anchors.margins: 4
                        spacing: 1

                        Text {
                            text: "线速 v"
                            font.family: Theme.fontFamilyMono
                            font.pixelSize: 8
                            color: Theme.textMuted
                        }
                        Text {
                            text: !root.isConnected ? "--" : (Math.abs(root.linVel).toFixed(2) + "m/s")
                            font.family: Theme.fontFamilyMono
                            font.pixelSize: 11
                            font.bold: true
                            color: !root.isConnected ? Theme.textMuted : Theme.lightBlue
                        }
                    }
                }

                // 实时角速 ω
                Rectangle {
                    Layout.fillWidth: true
                    height: 46
                    radius: Theme.radiusSm
                    color: Theme.bgContainer
                    border.color: Theme.borderSubtle
                    border.width: 1

                    ColumnLayout {
                        anchors.fill: parent
                        anchors.margins: 4
                        spacing: 1

                        Text {
                            text: "角速 ω"
                            font.family: Theme.fontFamilyMono
                            font.pixelSize: 8
                            color: Theme.textMuted
                        }
                        Text {
                            text: !root.isConnected ? "--" : (Math.abs(root.angVel).toFixed(2) + "r/s")
                            font.family: Theme.fontFamilyMono
                            font.pixelSize: 11
                            font.bold: true
                            color: !root.isConnected ? Theme.textMuted : Theme.textPrimary
                        }
                    }
                }
            }

            Rectangle { Layout.fillWidth: true; height: 1; color: "#242428" }

            // 4. 底层微指标行 (IMU 状态与电池电量)
            RowLayout {
                Layout.fillWidth: true

                Row {
                    spacing: 5
                    Rectangle {
                        width: 6; height: 6; radius: 3
                        color: root.isConnected ? Theme.successGreen : Theme.textMuted
                        anchors.verticalCenter: parent.verticalCenter
                    }
                    Text {
                        text: root.isConnected ? "IMU 50Hz 采样 · 零偏校准就绪" : "IMU 传感器脱机 · 待机等待"
                        font.family: Theme.fontFamilyNormal
                        font.pixelSize: 9
                        color: root.isConnected ? Theme.textSecondary : Theme.textMuted
                        anchors.verticalCenter: parent.verticalCenter
                    }
                }

                Item { Layout.fillWidth: true }

                Row {
                    spacing: 4
                    Text {
                        text: "PWR"
                        font.family: Theme.fontFamilyMono
                        font.bold: true
                        font.pixelSize: 8
                        color: Theme.successGreen
                        anchors.verticalCenter: parent.verticalCenter
                    }
                    Text {
                        text: "∞ 恒电 (48.0V)"
                        font.family: Theme.fontFamilyMono
                        font.pixelSize: 9
                        font.bold: true
                        color: Theme.textPrimary
                        anchors.verticalCenter: parent.verticalCenter
                    }
                }
            }
        }
    ]
}
