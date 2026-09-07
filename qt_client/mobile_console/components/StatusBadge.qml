import QtQuick
import QtQuick.Controls.Basic
import mobile_console

Rectangle {
    id: root

    property string text: ""
    property color textColor: Theme.textPrimary
    property color dotColor: "transparent"
    property bool showDot: false
    property bool dotPulse: false
    property color badgeColor: Theme.bgContainerElevated
    property color borderColor: Theme.borderSubtle
    property int fontSize: 10

    height: 20
    implicitWidth: badgeRow.implicitWidth + 14
    radius: Theme.radiusFull
    color: badgeColor
    border.color: borderColor
    border.width: 1

    Row {
        id: badgeRow
        anchors.centerIn: parent
        spacing: 5

        Rectangle {
            id: dot
            visible: root.showDot
            width: 6
            height: 6
            radius: 3
            color: root.dotColor
            anchors.verticalCenter: parent.verticalCenter

            SequentialAnimation on opacity {
                running: root.showDot && root.dotPulse
                loops: Animation.Infinite
                NumberAnimation { from: 1.0; to: 0.2; duration: 800; easing.type: Easing.InOutQuad }
                NumberAnimation { from: 0.2; to: 1.0; duration: 800; easing.type: Easing.InOutQuad }
            }
        }

        Text {
            text: root.text
            color: root.textColor
            font.family: Theme.fontFamilyMono
            font.pixelSize: root.fontSize
            font.bold: true
            anchors.verticalCenter: parent.verticalCenter
        }
    }
}
