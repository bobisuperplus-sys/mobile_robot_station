import QtQuick
import QtQuick.Controls.Basic
import mobile_console

Slider {
    id: control

    property color trackColor: Theme.borderBase
    property color activeColor: Theme.primaryBlue
    property color handleColor: Theme.textPrimary

    implicitHeight: 20
    implicitWidth: 160

    background: Rectangle {
        x: control.leftPadding
        y: control.topPadding + control.availableHeight / 2 - height / 2
        implicitWidth: 160
        implicitHeight: 4
        width: control.availableWidth
        height: implicitHeight
        radius: 2
        color: control.trackColor

        Rectangle {
            width: control.visualPosition * parent.width
            height: parent.height
            color: control.activeColor
            radius: 2
        }
    }

    handle: Rectangle {
        x: control.leftPadding + control.visualPosition * (control.availableWidth - width)
        y: control.topPadding + control.availableHeight / 2 - height / 2
        implicitWidth: 14
        implicitHeight: 14
        radius: 7
        color: control.pressed ? Qt.lighter(control.handleColor, 1.2) : control.handleColor
        border.color: control.activeColor
        border.width: 1.5

        Behavior on scale { NumberAnimation { duration: 100 } }
    }
}
