import QtQuick
import QtQuick.Layouts
import QtQuick.Controls.Basic
import mobile_console
import "../components"

SplitView {
    id: root

    orientation: Qt.Vertical
    handle: SplitHandleBar { orientation: Qt.Vertical }

    // 1. 上视口: 01 车载前视第一视角
    CardPanel {
        SplitView.fillWidth: true
        SplitView.fillHeight: true
        SplitView.preferredHeight: (parent ? parent.height * 0.5 : 240)
        SplitView.minimumHeight: 120
        cardRadius: Theme.radiusLg

        content: [
            // GStreamer 前视低延迟实时视频流 (UDP: 5002)
            GstVideoReceiver {
                id: frontCamReceiver
                anchors.fill: parent
                port: 5002
                autoStart: true
            },

            // 顶部信息条
            Rectangle {
                anchors.top: parent.top
                anchors.left: parent.left
                anchors.right: parent.right
                height: 32
                color: "#141416bf"
                border.color: "#2a2a2d99"
                border.width: 1

                RowLayout {
                    anchors.fill: parent
                    anchors.leftMargin: 10
                    anchors.rightMargin: 10

                    Row {
                        spacing: 6
                        Rectangle { width: 6; height: 6; radius: 3; color: Theme.primaryBlue; anchors.verticalCenter: parent.verticalCenter }
                        Text { text: "01 车载前视视角"; font.family: Theme.fontFamilyNormal; font.pixelSize: 11; font.bold: true; color: Theme.textPrimary; anchors.verticalCenter: parent.verticalCenter }
                    }

                    Item { Layout.fillWidth: true }

                    Row {
                        spacing: 5
                        StatusBadge {
                            text: frontCamReceiver.isConnected ? "LIVE" : "STANDBY"
                            textColor: frontCamReceiver.isConnected ? Theme.errorRedLight : Theme.textMuted
                            dotColor: frontCamReceiver.isConnected ? Theme.errorRedLight : Theme.textFaint
                            showDot: true
                            dotPulse: frontCamReceiver.isConnected
                            badgeColor: frontCamReceiver.isConnected ? Theme.errorBg : "#202024"
                            borderColor: frontCamReceiver.isConnected ? "#ffb4ab4d" : "#2e2e34"
                            fontSize: 9
                            height: 18
                        }
                        StatusBadge {
                            text: frontCamReceiver.isConnected ? (frontCamReceiver.fps + " FPS") : "5002"
                            textColor: frontCamReceiver.isConnected ? Theme.lightBlue : Theme.textMuted
                            badgeColor: "#202024"
                            borderColor: "#2e2e34"
                            fontSize: 9
                            height: 18
                        }
                    }
                }
            },

            // 中央 HUD 瞄准准星与俯仰标尺
            Item {
                anchors.centerIn: parent
                width: 180
                height: 120

                Canvas {
                    anchors.fill: parent
                    onPaint: {
                        var ctx = getContext("2d");
                        ctx.clearRect(0, 0, width, height);
                        var cx = width / 2;
                        var cy = height / 2;

                        // 中心白色微型十字
                        ctx.strokeStyle = "#ffffff";
                        ctx.lineWidth = 1.0;
                        ctx.beginPath();
                        ctx.moveTo(cx - 10, cy); ctx.lineTo(cx + 10, cy);
                        ctx.moveTo(cx, cy - 10); ctx.lineTo(cx, cy + 10);
                        ctx.stroke();

                        // 虚线外环
                        ctx.strokeStyle = "#3e90ff80";
                        ctx.setLineDash([2, 4]);
                        ctx.beginPath();
                        ctx.arc(cx, cy, 34, 0, Math.PI * 2);
                        ctx.stroke();
                        ctx.setLineDash([]);

                        // +5° 与 -5° 俯仰标线
                        ctx.strokeStyle = "#aac7ff";
                        ctx.lineWidth = 0.8;
                        ctx.beginPath();
                        ctx.moveTo(cx - 25, cy - 20); ctx.lineTo(cx - 12, cy - 20);
                        ctx.moveTo(cx + 12, cy - 20); ctx.lineTo(cx + 25, cy - 20);
                        ctx.moveTo(cx - 25, cy + 20); ctx.lineTo(cx - 12, cy + 20);
                        ctx.moveTo(cx + 12, cy + 20); ctx.lineTo(cx + 25, cy + 20);
                        ctx.stroke();

                        // 水平侧标
                        ctx.strokeStyle = "#34c759";
                        ctx.lineWidth = 1.5;
                        ctx.beginPath();
                        ctx.moveTo(cx - 65, cy); ctx.lineTo(cx - 45, cy);
                        ctx.moveTo(cx + 45, cy); ctx.lineTo(cx + 65, cy);
                        ctx.stroke();
                    }
                }

                Text { text: "+5°"; font.family: Theme.fontFamilyMono; font.pixelSize: 8; color: Theme.lightBlue; x: 42; y: 35 }
                Text { text: "-5°"; font.family: Theme.fontFamilyMono; font.pixelSize: 8; color: Theme.lightBlue; x: 42; y: 75 }
            },

            // 底部 HUD 浮动条
            Rectangle {
                anchors.bottom: parent.bottom
                anchors.left: parent.left
                anchors.right: parent.right
                height: 28
                color: "#141416d9"
                border.color: "#2a2a2d99"
                border.width: 1

                RowLayout {
                    anchors.fill: parent
                    anchors.leftMargin: 12
                    anchors.rightMargin: 12

                    Row {
                        spacing: 12
                        Row {
                            spacing: 4
                            Text { text: "SPD"; font.family: Theme.fontFamilyMono; font.pixelSize: 9; color: Theme.textMuted; anchors.verticalCenter: parent.verticalCenter }
                            Text { text: "0.45"; font.family: Theme.fontFamilyMono; font.pixelSize: 13; font.bold: true; color: Theme.lightBlue; anchors.verticalCenter: parent.verticalCenter }
                            Text { text: "m/s"; font.family: Theme.fontFamilyMono; font.pixelSize: 9; color: Theme.textMuted; anchors.verticalCenter: parent.verticalCenter }
                        }

                        Rectangle { width: 1; height: 12; color: Theme.borderSubtle; anchors.verticalCenter: parent.verticalCenter }

                        Row {
                            spacing: 4
                            Text { text: "PITCH"; font.family: Theme.fontFamilyMono; font.pixelSize: 9; color: Theme.textMuted; anchors.verticalCenter: parent.verticalCenter }
                            Text { text: "+3.4° (爬坡)"; font.family: Theme.fontFamilyMono; font.pixelSize: 11; font.bold: true; color: Theme.successGreen; anchors.verticalCenter: parent.verticalCenter }
                        }
                    }

                    Item { Layout.fillWidth: true }

                    Row {
                        spacing: 4
                        Text { text: "RAW"; font.family: Theme.fontFamilyMono; font.bold: true; font.pixelSize: 8; color: Theme.lightBlue; anchors.verticalCenter: parent.verticalCenter }
                        Text { text: "1080P/30fps"; font.family: Theme.fontFamilyMono; font.pixelSize: 9; color: Theme.textMuted; anchors.verticalCenter: parent.verticalCenter }
                    }
                }
            }
        ]
    }

    // 2. 下视口: 02 全局高空透视监控
    CardPanel {
        SplitView.fillWidth: true
        SplitView.fillHeight: true
        SplitView.preferredHeight: (parent ? parent.height * 0.5 : 240)
        SplitView.minimumHeight: 120
        cardRadius: Theme.radiusLg

        content: [
            // GStreamer 全局高空监控实时视频流 (UDP: 5004)
            GstVideoReceiver {
                id: overviewCamReceiver
                anchors.fill: parent
                port: 5004
                autoStart: true
            },

            // 顶部信息条
            Rectangle {
                anchors.top: parent.top
                anchors.left: parent.left
                anchors.right: parent.right
                height: 32
                color: "#141416bf"
                border.color: "#2a2a2d99"
                border.width: 1

                RowLayout {
                    anchors.fill: parent
                    anchors.leftMargin: 10
                    anchors.rightMargin: 10

                    Row {
                        spacing: 6
                        Rectangle { width: 6; height: 6; radius: 3; color: Theme.primaryBlue; anchors.verticalCenter: parent.verticalCenter }
                        Text { text: "02 车载跟随视角 (45° 第三人称)"; font.family: Theme.fontFamilyNormal; font.pixelSize: 11; font.bold: true; color: Theme.textPrimary; anchors.verticalCenter: parent.verticalCenter }
                    }

                    Item { Layout.fillWidth: true }

                    Row {
                        spacing: 5
                        StatusBadge {
                            text: overviewCamReceiver.isConnected ? "LIVE" : "STANDBY"
                            textColor: overviewCamReceiver.isConnected ? Theme.errorRedLight : Theme.textMuted
                            dotColor: overviewCamReceiver.isConnected ? Theme.errorRedLight : Theme.textFaint
                            showDot: true
                            dotPulse: overviewCamReceiver.isConnected
                            badgeColor: overviewCamReceiver.isConnected ? Theme.errorBg : "#202024"
                            borderColor: overviewCamReceiver.isConnected ? "#ffb4ab4d" : "#2e2e34"
                            fontSize: 9
                            height: 18
                        }
                        StatusBadge {
                            text: overviewCamReceiver.isConnected ? (overviewCamReceiver.fps + " FPS") : "5004"
                            textColor: overviewCamReceiver.isConnected ? Theme.lightBlue : Theme.textMuted
                            badgeColor: "#202024"
                            borderColor: "#2e2e34"
                            fontSize: 9
                            height: 18
                        }
                    }
                }
            },


            // 底部信息条
            Rectangle {
                anchors.bottom: parent.bottom
                anchors.left: parent.left
                anchors.right: parent.right
                height: 28
                color: "#141416d9"
                border.color: "#2a2a2d99"
                border.width: 1

                RowLayout {
                    anchors.fill: parent
                    anchors.leftMargin: 12
                    anchors.rightMargin: 12

                    Row {
                        spacing: 8
                        Text { text: "ZOOM: 1.0X"; font.family: Theme.fontFamilyMono; font.pixelSize: 9; color: Theme.textMuted }
                        Text { text: "·"; color: Theme.textFaint; font.pixelSize: 9 }
                        Text { text: "FOV: 55° (第三人称视角)"; font.family: Theme.fontFamilyMono; font.pixelSize: 9; color: Theme.textMuted }
                    }

                    Item { Layout.fillWidth: true }

                    Row {
                        spacing: 5
                        Rectangle { width: 6; height: 6; radius: 3; color: Theme.successGreen; anchors.verticalCenter: parent.verticalCenter }
                        Text { text: "目标实时锁定跟踪"; font.family: Theme.fontFamilyNormal; font.pixelSize: 9; color: Theme.successGreen; anchors.verticalCenter: parent.verticalCenter }
                    }
                }
            }
        ]
    }
}
