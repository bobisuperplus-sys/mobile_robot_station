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

    // 模式与状态绑定
    property int currentMode: 0 // 0: 手动遥控, 1: 实时建图, 2: 自主导航
    readonly property bool isAutoPilot: currentMode === 2
    property bool isEmergencyStopped: false

    // 动力学反馈
    property string activeDirection: "" // "up", "down", "left", "right", "stop"
    property real linearVelocity: 0.0
    property real angularVelocity: 0.0
    property real maxLinearVelocity: 1.20
    property real steeringSensitivity: 75.0

    // 交互信号
    signal modeSwitchRequested(int newMode)
    signal moveCommand(string direction)
    signal stopCommand()
    signal settingsChanged(real maxVel, real sensitivity)

    content: [
        ColumnLayout {
            anchors.fill: parent
            anchors.margins: 12
            spacing: 6

            // 1. 标题行与托管状态互锁开关
            RowLayout {
                Layout.fillWidth: true

                Row {
                    spacing: 6
                    Rectangle {
                        width: 34
                        height: 16
                        radius: 3
                        color: root.isAutoPilot ? "#0e2d3b" : "#16253b"
                        border.color: root.isAutoPilot ? "#00e5ff66" : "#3e90ff4d"
                        border.width: 1
                        anchors.verticalCenter: parent.verticalCenter
                        Text {
                            anchors.centerIn: parent
                            text: root.isAutoPilot ? "AUTO" : "CTRL"
                            font.family: Theme.fontFamilyMono
                            font.bold: true
                            font.pixelSize: 9
                            color: root.isAutoPilot ? "#00e5ff" : Theme.lightBlue
                        }
                    }
                    Text {
                        text: "手控方向舵与速度限值"
                        font.family: Theme.fontFamilyNormal
                        font.pixelSize: 11
                        font.bold: true
                        color: Theme.textPrimary
                        anchors.verticalCenter: parent.verticalCenter
                    }
                }

                Item { Layout.fillWidth: true }

                // 托管互锁与状态徽章
                Row {
                    spacing: 6
                    Layout.alignment: Qt.AlignVCenter

                    StatusBadge {
                        text: root.isAutoPilot ? "● 算法接管中" : "○ 手动就绪"
                        textColor: root.isAutoPilot ? "#00e5ff" : Theme.successGreen
                        badgeColor: root.isAutoPilot ? "#122a36" : "#10281b"
                        borderColor: root.isAutoPilot ? "#00e5ff4d" : "#34c7594d"
                        fontSize: 9
                        height: 18
                        dotPulse: root.isAutoPilot
                    }

                    Text {
                        text: "托管"
                        font.family: Theme.fontFamilyMono
                        font.pixelSize: 9
                        color: Theme.textMuted
                        anchors.verticalCenter: parent.verticalCenter
                    }

                    Switch {
                        id: autoSwitch
                        checked: root.isAutoPilot
                        implicitWidth: 32
                        implicitHeight: 18
                        anchors.verticalCenter: parent.verticalCenter

                        indicator: Rectangle {
                            implicitWidth: 32
                            implicitHeight: 18
                            radius: 9
                            color: autoSwitch.checked ? Theme.primaryBlue : "#242428"
                            border.color: autoSwitch.checked ? "#3e90ff80" : Theme.borderSubtle
                            border.width: 1

                            Rectangle {
                                x: autoSwitch.checked ? parent.width - width - 2 : 2
                                y: 2
                                width: 14
                                height: 14
                                radius: 7
                                color: "#e4e2e4"
                                Behavior on x { NumberAnimation { duration: 150 } }
                            }
                        }

                        onToggled: {
                            var targetMode = checked ? 2 : 0;
                            root.modeSwitchRequested(targetMode);
                        }
                    }
                }
            }

            Rectangle { Layout.fillWidth: true; height: 1; color: "#242428" }

            // 2. 十字方向键阵列 + 右侧动态双滑块
            RowLayout {
                Layout.fillWidth: true
                spacing: 12

                // 左侧: 十字方向键阵列 (3x3 网格)
                ColumnLayout {
                    spacing: 4

                    Rectangle {
                        width: 90
                        height: 90
                        radius: Theme.radiusMd
                        color: "#121214"
                        border.color: root.isAutoPilot ? "#00e5ff33" : "#26262a"
                        border.width: 1

                        GridLayout {
                            anchors.fill: parent
                            anchors.margins: 4
                            columns: 3
                            rows: 3
                            rowSpacing: 3
                            columnSpacing: 3

                            Item {} // (0,0)

                            // 前进按键 ▲
                            Rectangle {
                                id: btnUp
                                property bool isNavActive: root.isAutoPilot && (root.activeDirection === "up")
                                Layout.fillWidth: true; Layout.fillHeight: true
                                radius: Theme.radiusSm
                                color: btnUp.isNavActive ? "#00e5ff" : (upArea.pressed ? Theme.primaryBlue : (upArea.containsMouse && !root.isAutoPilot ? "#282830" : "#1c1c20"))
                                border.color: btnUp.isNavActive ? "#00e5ff" : "transparent"
                                border.width: 1

                                Text {
                                    anchors.centerIn: parent
                                    text: "▲"
                                    font.pixelSize: 12
                                    color: btnUp.isNavActive ? "#002026" : (upArea.pressed ? "#002957" : Theme.textPrimary)
                                }

                                MouseArea {
                                    id: upArea
                                    anchors.fill: parent
                                    enabled: !root.isAutoPilot && !root.isEmergencyStopped
                                    hoverEnabled: true
                                    cursorShape: root.isAutoPilot ? Qt.ForbiddenCursor : Qt.PointingHandCursor
                                    onPressed: root.moveCommand("up")
                                    onReleased: root.moveCommand("stop")
                                    onCanceled: root.moveCommand("stop")
                                }
                            }

                            Item {} // (0,2)

                            // 左转按键 ◀
                            Rectangle {
                                id: btnLeft
                                property bool isNavActive: root.isAutoPilot && (root.activeDirection === "left")
                                Layout.fillWidth: true; Layout.fillHeight: true
                                radius: Theme.radiusSm
                                color: btnLeft.isNavActive ? "#00e5ff" : (leftArea.pressed ? Theme.primaryBlue : (leftArea.containsMouse && !root.isAutoPilot ? "#282830" : "#1c1c20"))
                                border.color: btnLeft.isNavActive ? "#00e5ff" : "transparent"
                                border.width: 1

                                Text {
                                    anchors.centerIn: parent
                                    text: "◀"
                                    font.pixelSize: 12
                                    color: btnLeft.isNavActive ? "#002026" : (leftArea.pressed ? "#002957" : Theme.textPrimary)
                                }

                                MouseArea {
                                    id: leftArea
                                    anchors.fill: parent
                                    enabled: !root.isAutoPilot && !root.isEmergencyStopped
                                    hoverEnabled: true
                                    cursorShape: root.isAutoPilot ? Qt.ForbiddenCursor : Qt.PointingHandCursor
                                    onPressed: root.moveCommand("left")
                                    onReleased: root.moveCommand("stop")
                                    onCanceled: root.moveCommand("stop")
                                }
                            }

                            // 急停/驻车按键 ■
                            Rectangle {
                                id: btnStop
                                property bool isNavActive: (root.isAutoPilot && root.activeDirection === "stop") || root.isEmergencyStopped
                                Layout.fillWidth: true; Layout.fillHeight: true
                                radius: Theme.radiusSm
                                color: root.isEmergencyStopped ? Theme.errorRed : (btnStop.isNavActive ? "#1c2e3d" : (stopArea.pressed ? Theme.errorRed : (stopArea.containsMouse && !root.isAutoPilot ? "#323238" : "#24242a")))
                                border.color: root.isEmergencyStopped ? Theme.errorRedLight : (btnStop.isNavActive ? "#00e5ff" : "transparent")
                                border.width: 1

                                Text {
                                    anchors.centerIn: parent
                                    text: "■"
                                    font.pixelSize: 10
                                    font.bold: true
                                    color: root.isEmergencyStopped ? "#ffffff" : (btnStop.isNavActive ? "#00e5ff" : Theme.errorRedLight)
                                }

                                MouseArea {
                                    id: stopArea
                                    anchors.fill: parent
                                    hoverEnabled: true
                                    cursorShape: Qt.PointingHandCursor
                                    onClicked: root.stopCommand()
                                }
                            }

                            // 右转按键 ▶
                            Rectangle {
                                id: btnRight
                                property bool isNavActive: root.isAutoPilot && (root.activeDirection === "right")
                                Layout.fillWidth: true; Layout.fillHeight: true
                                radius: Theme.radiusSm
                                color: btnRight.isNavActive ? "#00e5ff" : (rightArea.pressed ? Theme.primaryBlue : (rightArea.containsMouse && !root.isAutoPilot ? "#282830" : "#1c1c20"))
                                border.color: btnRight.isNavActive ? "#00e5ff" : "transparent"
                                border.width: 1

                                Text {
                                    anchors.centerIn: parent
                                    text: "▶"
                                    font.pixelSize: 12
                                    color: btnRight.isNavActive ? "#002026" : (rightArea.pressed ? "#002957" : Theme.textPrimary)
                                }

                                MouseArea {
                                    id: rightArea
                                    anchors.fill: parent
                                    enabled: !root.isAutoPilot && !root.isEmergencyStopped
                                    hoverEnabled: true
                                    cursorShape: root.isAutoPilot ? Qt.ForbiddenCursor : Qt.PointingHandCursor
                                    onPressed: root.moveCommand("right")
                                    onReleased: root.moveCommand("stop")
                                    onCanceled: root.moveCommand("stop")
                                }
                            }

                            Item {} // (2,0)

                            // 后退按键 ▼
                            Rectangle {
                                id: btnDown
                                property bool isNavActive: root.isAutoPilot && (root.activeDirection === "down")
                                Layout.fillWidth: true; Layout.fillHeight: true
                                radius: Theme.radiusSm
                                color: btnDown.isNavActive ? "#00e5ff" : (downArea.pressed ? Theme.primaryBlue : (downArea.containsMouse && !root.isAutoPilot ? "#282830" : "#1c1c20"))
                                border.color: btnDown.isNavActive ? "#00e5ff" : "transparent"
                                border.width: 1

                                Text {
                                    anchors.centerIn: parent
                                    text: "▼"
                                    font.pixelSize: 12
                                    color: btnDown.isNavActive ? "#002026" : (downArea.pressed ? "#002957" : Theme.textPrimary)
                                }

                                MouseArea {
                                    id: downArea
                                    anchors.fill: parent
                                    enabled: !root.isAutoPilot && !root.isEmergencyStopped
                                    hoverEnabled: true
                                    cursorShape: root.isAutoPilot ? Qt.ForbiddenCursor : Qt.PointingHandCursor
                                    onPressed: root.moveCommand("down")
                                    onReleased: root.moveCommand("stop")
                                    onCanceled: root.moveCommand("stop")
                                }
                            }

                            Item {} // (2,2)
                        }
                    }

                    // 模式提示文字
                    Text {
                        Layout.preferredWidth: 90
                        horizontalAlignment: Text.AlignHCenter
                        text: root.isAutoPilot ? "🔒 算法接管只读反馈" : "WASD / 鼠标点按驱动"
                        font.family: Theme.fontFamilyMono
                        font.pixelSize: 8
                        color: root.isAutoPilot ? "#00e5ff" : Theme.textMuted
                    }
                }

                // 右侧: 速度与灵敏度调节 / 算法实时反馈
                ColumnLayout {
                    Layout.fillWidth: true
                    spacing: 4

                    // 线速滑块 (手控可调 vs 自动只读)
                    ColumnLayout {
                        Layout.fillWidth: true
                        spacing: 1

                        RowLayout {
                            Layout.fillWidth: true
                            Text {
                                text: root.isAutoPilot ? "算法巡航线速" : "线速上限"
                                font.family: Theme.fontFamilyMono
                                font.pixelSize: 9
                                color: root.isAutoPilot ? "#00e5ff" : Theme.textMuted
                            }
                            Item { Layout.fillWidth: true }
                            Text {
                                text: root.isAutoPilot
                                      ? (Math.abs(root.linearVelocity).toFixed(2) + " m/s (动态)")
                                      : (velSlider.value.toFixed(2) + " m/s")
                                font.family: Theme.fontFamilyMono
                                font.pixelSize: 10
                                font.bold: true
                                color: root.isAutoPilot ? "#00e5ff" : Theme.lightBlue
                            }
                        }

                        SliderBar {
                            id: velSlider
                            Layout.fillWidth: true
                            enabled: !root.isAutoPilot
                            opacity: root.isAutoPilot ? 0.75 : 1.0
                            activeColor: root.isAutoPilot ? "#00e5ff" : Theme.primaryBlue
                            from: 0.1
                            to: 2.0
                            value: root.isAutoPilot ? Math.min(2.0, Math.max(0.1, Math.abs(root.linearVelocity))) : root.maxLinearVelocity
                            stepSize: 0.05
                            onMoved: {
                                if (!root.isAutoPilot) {
                                    root.maxLinearVelocity = value;
                                    root.settingsChanged(root.maxLinearVelocity, root.steeringSensitivity);
                                }
                            }
                        }
                    }

                    // 转向灵敏度滑块 (手控可调 vs 自动只读)
                    ColumnLayout {
                        Layout.fillWidth: true
                        spacing: 1

                        RowLayout {
                            Layout.fillWidth: true
                            Text {
                                text: root.isAutoPilot ? "转向角速响应" : "转向灵敏度"
                                font.family: Theme.fontFamilyMono
                                font.pixelSize: 9
                                color: root.isAutoPilot ? "#00e5ff" : Theme.textMuted
                            }
                            Item { Layout.fillWidth: true }
                            Text {
                                text: root.isAutoPilot
                                      ? (Math.abs(root.angularVelocity).toFixed(2) + " rad/s")
                                      : (Math.round(steerSlider.value) + "%")
                                font.family: Theme.fontFamilyMono
                                font.pixelSize: 10
                                font.bold: true
                                color: root.isAutoPilot ? "#00e5ff" : Theme.textPrimary
                            }
                        }

                        SliderBar {
                            id: steerSlider
                            Layout.fillWidth: true
                            enabled: !root.isAutoPilot
                            opacity: root.isAutoPilot ? 0.75 : 1.0
                            activeColor: root.isAutoPilot ? "#00e5ff" : Theme.primaryBlue
                            from: 10
                            to: 100
                            value: root.steeringSensitivity
                            stepSize: 5
                            onMoved: {
                                if (!root.isAutoPilot) {
                                    root.steeringSensitivity = value;
                                    root.settingsChanged(root.maxLinearVelocity, root.steeringSensitivity);
                                }
                            }
                        }
                    }

                    // 底层微指标行
                    RowLayout {
                        Layout.fillWidth: true
                        Text {
                            text: root.isAutoPilot ? "手控状态: 安全锁定" : "死区保护: ON"
                            font.family: Theme.fontFamilyMono
                            font.pixelSize: 9
                            color: root.isAutoPilot ? Theme.lightBlue : Theme.textMuted
                        }
                        Item { Layout.fillWidth: true }
                        Text {
                            text: "急停响应: 12ms"
                            font.family: Theme.fontFamilyMono
                            font.pixelSize: 9
                            color: Theme.textMuted
                        }
                    }
                }
            }
        }
    ]
}
