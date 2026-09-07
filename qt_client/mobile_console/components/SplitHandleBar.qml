import QtQuick
import QtQuick.Controls.Basic
import mobile_console

Rectangle {
    id: handleRoot

    property int orientation: Qt.Horizontal // Qt.Horizontal: 垂直分割线 (用于左右拖拽) / Qt.Vertical: 水平分割线 (用于上下拖拽)

    implicitWidth: orientation === Qt.Horizontal ? 8 : (parent ? parent.width : 0)
    implicitHeight: orientation === Qt.Vertical ? 8 : (parent ? parent.height : 0)
    color: "transparent"

    // 中间 1px 细分割线
    Rectangle {
        anchors.centerIn: parent
        width: handleRoot.orientation === Qt.Horizontal ? 1 : parent.width
        height: handleRoot.orientation === Qt.Vertical ? 1 : parent.height
        color: SplitHandle.pressed ? Theme.primaryBlue : (SplitHandle.hovered ? Theme.borderHighlight : Theme.borderBase)
        Behavior on color { ColorAnimation { duration: 120 } }
    }

    // 悬浮/按压时平滑显现的居中微型抓手手柄 (Apple Pro / Xcode 质感 Grip)
    Rectangle {
        anchors.centerIn: parent
        width: handleRoot.orientation === Qt.Horizontal ? 3 : 24
        height: handleRoot.orientation === Qt.Horizontal ? 24 : 3
        radius: 1.5
        color: SplitHandle.pressed ? Theme.primaryBlue : Theme.lightBlue
        opacity: SplitHandle.pressed ? 1.0 : (SplitHandle.hovered ? 0.75 : 0.0)
        Behavior on opacity { NumberAnimation { duration: 120 } }
    }
}
