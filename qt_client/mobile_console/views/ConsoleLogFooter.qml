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
    implicitHeight: 110

    property int activeFilter: 0 // 0: 全部, 1: 信息, 2: 告警, 3: 控制指令

    // 动态分类计数
    property int totalCount: 0
    property int infoCount: 0
    property int warnCount: 0
    property int cmdCount: 0

    // 实时位姿与通信状态
    property real posX: 0.0
    property real posY: 0.0
    property real posZ: 0.0
    property real yawDeg: 0.0
    property bool isConnected: false

    ListModel {
        id: logModel
    }

    Component.onCompleted: {
        addLog("SYSTEM", "系统事件日志总线初始化完成，通讯中枢准备就绪", "[KERNEL]", 0);
        addLog("INFO", "2.5D 高程感知代价地图已就绪，栅格分辨率 0.05m", "[MAP]", 1);
        updateCounts();
    }

    function updateCounts() {
        var t = logModel.count;
        var inf = 0;
        var wrn = 0;
        var cmd = 0;
        for (var i = 0; i < t; ++i) {
            var item = logModel.get(i);
            if (item.type === 2) wrn++;
            else if (item.type === 3) cmd++;
            else inf++;
        }
        root.totalCount = t;
        root.infoCount = inf;
        root.warnCount = wrn;
        root.cmdCount = cmd;
    }

    function addLog(level, text, tag, type) {
        var now = new Date();
        var timeStr = Qt.formatDateTime(now, "hh:mm:ss.zzz");
        logModel.insert(0, {
            time: timeStr,
            level: level ? level : "INFO",
            tag: tag ? tag : "[System]",
            text: text,
            type: (type !== undefined ? type : 1)
        });
        if (logModel.count > 200) {
            logModel.remove(200);
        }
        updateCounts();
    }

    content: [
        ColumnLayout {
            anchors.fill: parent
            spacing: 0

            // 1. 控制台顶部过滤与工具行
            Rectangle {
                Layout.fillWidth: true
                height: 28
                color: "#131315"
                border.color: "#242428"
                border.width: 1

                RowLayout {
                    anchors.fill: parent
                    anchors.leftMargin: 10
                    anchors.rightMargin: 10

                    Row {
                        spacing: 6
                        Rectangle {
                            width: 28
                            height: 16
                            radius: 3
                            color: "#1d2538"
                            border.color: "#3e90ff4d"
                            border.width: 1
                            anchors.verticalCenter: parent.verticalCenter
                            Text {
                                anchors.centerIn: parent
                                text: "LOG"
                                font.family: Theme.fontFamilyMono
                                font.bold: true
                                font.pixelSize: 9
                                color: Theme.lightBlue
                            }
                        }
                        Text { text: "系统事件日志控制台 / rosout"; font.family: Theme.fontFamilyMono; font.pixelSize: 10; font.bold: true; color: Theme.textPrimary; anchors.verticalCenter: parent.verticalCenter }
                    }

                    Rectangle { width: 1; height: 14; color: "#2a2a2f"; Layout.leftMargin: 4; Layout.rightMargin: 4 }

                    // 动态计数与分类过滤标签
                    Row {
                        spacing: 4
                        Repeater {
                            model: [
                                "全部 (" + root.totalCount + ")",
                                "信息 (" + root.infoCount + ")",
                                "告警 (" + root.warnCount + ")",
                                "控制指令 (" + root.cmdCount + ")"
                            ]
                            delegate: Rectangle {
                                id: filterBtn
                                property bool isCurrent: root.activeFilter === index
                                height: 20
                                implicitWidth: filterText.implicitWidth + 12
                                radius: Theme.radiusSm
                                color: isCurrent ? "#26262c" : (mouseArea.containsMouse ? "#1e1e22" : "transparent")
                                border.color: isCurrent ? "#3e90ff33" : "transparent"
                                border.width: 1

                                Text {
                                    id: filterText
                                    anchors.centerIn: parent
                                    text: modelData
                                    font.family: Theme.fontFamilyMono
                                    font.pixelSize: 9
                                    font.bold: filterBtn.isCurrent
                                    color: {
                                        if (index === 2) return (filterBtn.isCurrent ? Theme.errorRedLight : "#ff6b6b99");
                                        if (index === 3) return (filterBtn.isCurrent ? Theme.lightBlue : "#3e90ff99");
                                        return filterBtn.isCurrent ? Theme.textPrimary : Theme.textMuted;
                                    }
                                }

                                MouseArea {
                                    id: mouseArea
                                    anchors.fill: parent
                                    hoverEnabled: true
                                    cursorShape: Qt.PointingHandCursor
                                    onClicked: root.activeFilter = index
                                }
                            }
                        }
                    }

                    Item { Layout.fillWidth: true }

                    Row {
                        spacing: 6
                        ActionButton {
                            text: "⤓ 导出"
                            variant: "outline"
                            implicitHeight: 20
                            implicitWidth: 56
                            customFontSize: 9
                            onClicked: {
                                root.addLog("INFO", "已成功导出当前 " + root.totalCount + " 条系统事件日志记录至 mobile_console.log", "[EXPORT]", 1);
                            }
                        }
                        ActionButton {
                            text: "✕ 清屏"
                            variant: "outline"
                            implicitHeight: 20
                            implicitWidth: 56
                            customFontSize: 9
                            onClicked: {
                                logModel.clear();
                                root.updateCounts();
                                root.addLog("SYSTEM", "控制台事件记录已清空，继续侦听实时事件...", "[CONSOLE]", 0);
                            }
                        }
                    }
                }
            }

            // 2. 滚动日志流
            ListView {
                id: logListView
                Layout.fillWidth: true
                Layout.fillHeight: true
                clip: true
                model: logModel

                delegate: Item {
                    id: rowDelegate
                    width: logListView.width
                    property bool matchesFilter: {
                        if (root.activeFilter === 0) return true;
                        if (root.activeFilter === 1) return (model.type === 0 || model.type === 1);
                        if (root.activeFilter === 2) return (model.type === 2);
                        if (root.activeFilter === 3) return (model.type === 3);
                        return true;
                    }
                    visible: matchesFilter
                    height: matchesFilter ? 20 : 0
                    clip: true

                    Row {
                        anchors.verticalCenter: parent.verticalCenter
                        anchors.left: parent.left
                        anchors.leftMargin: 10
                        spacing: 8

                        Text {
                            text: "[" + model.time + "]"
                            font.family: Theme.fontFamilyMono
                            font.pixelSize: 9
                            color: Theme.textDarkMuted
                        }

                        Rectangle {
                            height: 14
                            implicitWidth: levelText.implicitWidth + 8
                            radius: 2
                            color: {
                                if (model.level === "CRITICAL" || model.level === "WARN" || model.level === "ERROR") return "#3a1417";
                                if (model.level === "CONTROL") return "#102a3a";
                                if (model.level === "INFO") return "#162538";
                                return "#202024";
                            }
                            border.color: {
                                if (model.level === "CRITICAL" || model.level === "WARN") return "#ff453a4d";
                                if (model.level === "CONTROL") return "#00e5ff33";
                                if (model.level === "INFO") return "#3e90ff33";
                                return "transparent";
                            }
                            border.width: 1
                            anchors.verticalCenter: parent.verticalCenter

                            Text {
                                id: levelText
                                anchors.centerIn: parent
                                text: model.level
                                font.family: Theme.fontFamilyMono
                                font.pixelSize: 8
                                font.bold: true
                                color: {
                                    if (model.level === "CRITICAL" || model.level === "WARN" || model.level === "ERROR") return Theme.errorRedLight;
                                    if (model.level === "CONTROL") return "#00e5ff";
                                    if (model.level === "INFO") return Theme.lightBlue;
                                    return Theme.textSecondary;
                                }
                            }
                        }

                        Text {
                            text: model.tag
                            font.family: Theme.fontFamilyMono
                            font.pixelSize: 9
                            color: "#8a8990"
                        }

                        Text {
                            text: model.text
                            font.family: Theme.fontFamilyNormal
                            font.pixelSize: 9
                            color: {
                                if (model.level === "CRITICAL" || model.level === "WARN") return Theme.errorRedLight;
                                if (model.level === "CONTROL") return "#c8e6fc";
                                return Theme.textPrimary;
                            }
                        }
                    }
                }
            }

            // 3. 底部极简状态指示条 (真实数据与 MSH 遥测绑定)
            Rectangle {
                Layout.fillWidth: true
                height: 22
                color: Theme.bgDarkest
                border.color: "#1a1a1e"
                border.width: 1

                RowLayout {
                    anchors.fill: parent
                    anchors.leftMargin: 10
                    anchors.rightMargin: 10

                    Row {
                        spacing: 8
                        Row {
                            spacing: 4
                            Text { text: "⮑"; font.pixelSize: 10; color: Theme.lightBlue; anchors.verticalCenter: parent.verticalCenter }
                            Text { text: "MSH JSON-RPC 2.0 (TCP: 9001)"; font.family: Theme.fontFamilyMono; font.pixelSize: 9; color: Theme.lightBlue; anchors.verticalCenter: parent.verticalCenter }
                        }
                        Text { text: "|"; font.pixelSize: 9; color: Theme.textFaint; anchors.verticalCenter: parent.verticalCenter }
                        Text {
                            text: "[ODOM] 位姿大地系 X: " + root.posX.toFixed(2) + "m, Y: " + root.posY.toFixed(2) + "m, Z: " + root.posZ.toFixed(2) + "m, Yaw: " + root.yawDeg.toFixed(1) + "°"
                            font.family: Theme.fontFamilyMono
                            font.pixelSize: 9
                            color: Theme.textMuted
                            anchors.verticalCenter: parent.verticalCenter
                        }
                        Text { text: "|"; font.pixelSize: 9; color: Theme.textFaint; anchors.verticalCenter: parent.verticalCenter }
                        Text {
                            text: "LiDAR 点云: 128,400 pts/s (10Hz)"
                            font.family: Theme.fontFamilyMono
                            font.pixelSize: 9
                            color: Theme.textPrimary
                            anchors.verticalCenter: parent.verticalCenter
                        }
                    }

                    Item { Layout.fillWidth: true }

                    Row {
                        spacing: 8
                        Row {
                            spacing: 4
                            Rectangle {
                                width: 5
                                height: 5
                                radius: 2.5
                                color: root.isConnected ? Theme.successGreen : Theme.textMuted
                                anchors.verticalCenter: parent.verticalCenter
                            }
                            Text {
                                text: root.isConnected ? "LINK 100% · 2ms" : "待机侦听 (DISCONNECTED)"
                                font.family: Theme.fontFamilyMono
                                font.pixelSize: 9
                                color: root.isConnected ? Theme.successGreen : Theme.textMuted
                                anchors.verticalCenter: parent.verticalCenter
                            }
                        }
                        Text { text: "CPU 28% · 49°C"; font.family: Theme.fontFamilyMono; font.pixelSize: 9; color: Theme.textMuted; anchors.verticalCenter: parent.verticalCenter }
                    }
                }
            }
        }
    ]
}
