import QtQuick
import QtQuick.Layouts
import QtQuick.Controls.Basic
import mobile_console
import "../components"

Rectangle {
    id: root

    property int currentMode: 0 // 0: 手动遥控, 1: 实时建图, 2: 自主导航
    property bool isConnected: false
    signal modeChanged(int mode)
    signal emergencyStopTriggered()

    height: 56
    color: Theme.bgHeader
    border.color: Theme.borderBase
    border.width: 0

    Rectangle {
        anchors.bottom: parent.bottom
        width: parent.width
        height: 1
        color: Theme.borderBase
    }

    RowLayout {
        anchors.fill: parent
        anchors.leftMargin: 16
        anchors.rightMargin: 16
        spacing: 12

        // 1. 左侧：图标 + 标题 + MSH 状态
        RowLayout {
            spacing: 10

            Rectangle {
                width: 32
                height: 32
                radius: Theme.radiusMd
                color: Theme.bgContainer
                border.color: Theme.borderBase
                border.width: 1

                Text {
                    anchors.centerIn: parent
                    text: "⛭"
                    color: Theme.primaryBlue
                    font.pixelSize: 18
                }
            }

            ColumnLayout {
                spacing: 1

                Text {
                    text: "移动机器人地面监控工作站"
                    font.family: Theme.fontFamilyNormal
                    font.pixelSize: 14
                    font.bold: true
                    color: Theme.textPrimary
                }

                Row {
                    spacing: 6
                    Text {
                        text: "UGV-TWIN OS v4.2"
                        font.family: Theme.fontFamilyMono
                        font.pixelSize: 10
                        color: Theme.textMuted
                    }
                    Text {
                        text: "·"
                        color: Theme.textFaint
                        font.pixelSize: 10
                    }
                    Text {
                        text: "工业数字孪生节点"
                        font.family: Theme.fontFamilyNormal
                        font.pixelSize: 10
                        color: Theme.textMuted
                    }
                }
            }

            Rectangle {
                width: 1
                height: 20
                color: Theme.borderBase
                Layout.leftMargin: 6
                Layout.rightMargin: 6
            }

            StatusBadge {
                text: root.isConnected ? "MSH RPC 在线 · 2ms 极低延迟" : "MSH 待机侦听 (TCP: 9001)"
                textColor: root.isConnected ? Theme.textPrimary : Theme.textMuted
                dotColor: root.isConnected ? Theme.successGreen : Theme.warningAmber
                showDot: true
                dotPulse: root.isConnected
                badgeColor: Theme.bgContainer
                borderColor: Theme.borderBase
                fontSize: 11
            }
        }

        Item { Layout.fillWidth: true }

        // 2. 中央：三段胶囊模式切换器
        Rectangle {
            id: modeCapsule
            height: 34
            implicitWidth: modeRow.implicitWidth + 8
            radius: Theme.radiusFull
            color: "#141416"
            border.color: "#262629"
            border.width: 1

            Row {
                id: modeRow
                anchors.centerIn: parent
                spacing: 4

                Repeater {
                    model: ["手动遥控", "实时建图", "自主导航"]
                    delegate: Rectangle {
                        id: segBtn
                        property bool isSelected: root.currentMode === index
                        height: 26
                        width: 78
                        radius: Theme.radiusFull
                        color: isSelected ? "#2a2a30" : "transparent"
                        border.color: isSelected ? "#3e90ff4d" : "transparent"
                        border.width: 1

                        Text {
                            anchors.centerIn: parent
                            text: modelData
                            font.family: Theme.fontFamilyNormal
                            font.pixelSize: 11
                            font.bold: segBtn.isSelected
                            color: segBtn.isSelected ? Theme.lightBlue : Theme.textMuted
                        }

                        MouseArea {
                            anchors.fill: parent
                            hoverEnabled: true
                            cursorShape: Qt.PointingHandCursor
                            onClicked: {
                                root.currentMode = index;
                                root.modeChanged(index);
                            }
                        }
                    }
                }
            }
        }

        Item { Layout.fillWidth: true }

        // 3. 右侧：急停制动 + 网络/电量 + 时钟 + 用户
        RowLayout {
            spacing: 12

            ActionButton {
                text: "⚠ 急停制动"
                variant: "danger"
                isBold: true
                implicitHeight: 32
                implicitWidth: 98
                onClicked: root.emergencyStopTriggered()
            }

            Rectangle {
                height: 30
                radius: Theme.radiusSm
                color: Theme.bgContainer
                border.color: Theme.borderBase
                border.width: 1
                implicitWidth: statusRow.implicitWidth + 18

                Row {
                    id: statusRow
                    anchors.centerIn: parent
                    spacing: 8

                    Row {
                        spacing: 4
                        Rectangle {
                            width: 6
                            height: 6
                            radius: 3
                            color: root.isConnected ? Theme.successGreen : Theme.textMuted
                            anchors.verticalCenter: parent.verticalCenter
                        }
                        Text { text: "RF"; font.family: Theme.fontFamilyMono; font.bold: true; font.pixelSize: 10; color: Theme.lightBlue; anchors.verticalCenter: parent.verticalCenter }
                        Text { text: "5.8G"; font.family: Theme.fontFamilyMono; font.pixelSize: 11; color: Theme.textPrimary; anchors.verticalCenter: parent.verticalCenter }
                    }

                    Text { text: "|"; color: Theme.textFaint; font.pixelSize: 11; anchors.verticalCenter: parent.verticalCenter }

                    Row {
                        spacing: 4
                        Text { text: "BAT"; font.family: Theme.fontFamilyMono; font.bold: true; font.pixelSize: 9; color: Theme.successGreen; anchors.verticalCenter: parent.verticalCenter }
                        Text { text: "∞"; font.family: Theme.fontFamilyMono; font.bold: true; font.pixelSize: 13; color: Theme.successGreen; anchors.verticalCenter: parent.verticalCenter }
                    }
                }
            }

            Text {
                id: clockText
                text: Qt.formatDateTime(new Date(), "hh:mm:ss") + " UTC+8"
                font.family: Theme.fontFamilyMono
                font.pixelSize: 11
                color: Theme.textMuted

                Timer {
                    interval: 1000
                    running: true
                    repeat: true
                    onTriggered: clockText.text = Qt.formatDateTime(new Date(), "hh:mm:ss") + " UTC+8"
                }
            }

            Rectangle {
                width: 30
                height: 30
                radius: 15
                color: "#26262b"
                border.color: Theme.borderHighlight
                border.width: 1

                Text {
                    anchors.centerIn: parent
                    text: "OP"
                    font.family: Theme.fontFamilyMono
                    font.bold: true
                    font.pixelSize: 11
                    color: Theme.textSecondary
                }
            }
        }
    }
}
