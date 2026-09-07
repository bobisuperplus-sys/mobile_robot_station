import QtQuick
import QtQuick.Controls.Basic
import mobile_console

Rectangle {
    id: root

    property alias content: contentContainer.data
    property color cardColor: Theme.bgSurface
    property color borderColor: Theme.borderBase
    property real cardRadius: Theme.radiusMd

    color: cardColor
    radius: cardRadius
    border.color: borderColor
    border.width: 1
    clip: true

    Item {
        id: contentContainer
        anchors.fill: parent
        anchors.margins: 1
    }
}
