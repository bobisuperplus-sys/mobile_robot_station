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
    Layout.fillHeight: true
    implicitHeight: 200

    signal saveCurrentPoseRequested()
    signal waypointSelected(int index)

    ListModel {
        id: wpModel
        ListElement { name: "1. 起始整备区"; status: "出发点"; statusType: "success"; coords: "(0.00, -6.00, 0.01)"; active: false }
        ListElement { name: "2. 环岛西侧通道"; status: "巡检点"; statusType: "info"; coords: "(-2.50, 0.00, 0.01)"; active: false }
        ListElement { name: "3. 登坡入口准备"; status: "坡底缓冲"; statusType: "info"; coords: "(0.00, 2.40, 0.01)"; active: false }
        ListElement { name: "4. 坡顶立体观景台"; status: "核心目标"; statusType: "active"; coords: "(0.00, 6.00, 0.12)"; active: true }
    }

    content: [
        ColumnLayout {
            anchors.fill: parent
            anchors.margins: 12
            spacing: 8

            // 1. 标题行
            RowLayout {
                Layout.fillWidth: true

                Row {
                    spacing: 6
                    Text { text: "◆"; font.pixelSize: 11; color: Theme.lightBlue; anchors.verticalCenter: parent.verticalCenter }
                    Text { text: "航点与兴趣点 (POI)"; font.family: Theme.fontFamilyNormal; font.pixelSize: 11; font.bold: true; color: Theme.textPrimary; anchors.verticalCenter: parent.verticalCenter }
                }

                Item { Layout.fillWidth: true }

                ActionButton {
                    text: "+ 收藏当前位置"
                    variant: "secondary"
                    implicitHeight: 24
                    implicitWidth: 104
                    customFontSize: 10
                    onClicked: root.saveCurrentPoseRequested()
                }
            }

            Rectangle { Layout.fillWidth: true; height: 1; color: "#242428" }

            // 2. 航点表格列表
            ListView {
                id: wpListView
                Layout.fillWidth: true
                Layout.fillHeight: true
                clip: true
                spacing: 4
                model: wpModel

                delegate: Rectangle {
                    width: wpListView.width
                    height: 40
                    radius: Theme.radiusSm
                    color: model.active ? "#16253b66" : (mouseArea.containsMouse ? "#1c1c20" : "transparent")
                    border.color: model.active ? "#3e90ff33" : "transparent"
                    border.width: 1

                    RowLayout {
                        anchors.fill: parent
                        anchors.leftMargin: 8
                        anchors.rightMargin: 8
                        spacing: 6

                        ColumnLayout {
                            spacing: 1

                            Row {
                                spacing: 6
                                Text {
                                    text: model.name
                                    font.family: Theme.fontFamilyNormal
                                    font.pixelSize: 11
                                    font.bold: true
                                    color: model.active ? Theme.lightBlue : Theme.textPrimary
                                    anchors.verticalCenter: parent.verticalCenter
                                }

                                Rectangle {
                                    height: 16
                                    implicitWidth: statusText.implicitWidth + 8
                                    radius: 3
                                    color: model.statusType === "success" ? "#1e2320" : (model.statusType === "active" ? "#3e90ff33" : "#202028")
                                    border.color: model.statusType === "success" ? "#34c75933" : (model.statusType === "active" ? "#3e90ff66" : "#3e90ff33")
                                    border.width: 1
                                    anchors.verticalCenter: parent.verticalCenter

                                    Text {
                                        id: statusText
                                        anchors.centerIn: parent
                                        text: model.status
                                        font.family: Theme.fontFamilyMono
                                        font.pixelSize: 9
                                        font.bold: model.statusType === "active"
                                        color: model.statusType === "success" ? Theme.successGreen : Theme.lightBlue
                                    }
                                }
                            }

                            Text {
                                text: "XYZ: " + model.coords
                                font.family: Theme.fontFamilyMono
                                font.pixelSize: 9
                                color: Theme.textMuted
                            }
                        }

                        Item { Layout.fillWidth: true }

                        Row {
                            spacing: 4

                            ActionButton {
                                text: model.active ? "进行中" : "前往"
                                variant: model.active ? "primary" : "secondary"
                                isBold: model.active
                                implicitHeight: 22
                                implicitWidth: model.active ? 54 : 44
                                customFontSize: 10
                                onClicked: root.waypointSelected(index)
                            }

                            ActionButton {
                                text: "✕"
                                variant: "outline"
                                implicitHeight: 22
                                implicitWidth: 24
                                customFontSize: 10
                                onClicked: wpModel.remove(index)
                            }
                        }
                    }

                    MouseArea {
                        id: mouseArea
                        anchors.fill: parent
                        hoverEnabled: true
                        propagateComposedEvents: true
                        onClicked: mouse.accepted = false
                    }
                }
            }
        }
    ]
}
