import QtQuick
import QtQuick.Controls.Basic
import mobile_console

Button {
    id: control

    property string variant: "secondary" // "primary", "secondary", "danger", "outline"
    property string customColor: ""
    property string customTextColor: ""
    property real customRadius: Theme.radiusSm
    property int customFontSize: 11
    property bool isBold: false

    implicitHeight: 28
    implicitWidth: Math.max(64, contentItem.implicitWidth + 20)

    font.family: Theme.fontFamilyNormal
    font.pixelSize: customFontSize
    font.bold: isBold

    background: Rectangle {
        implicitWidth: control.implicitWidth
        implicitHeight: control.implicitHeight
        radius: control.customRadius
        color: {
            if (control.customColor !== "") {
                return control.down ? Qt.darker(control.customColor, 1.2) : (control.hovered ? Qt.lighter(control.customColor, 1.15) : control.customColor);
            }
            if (control.variant === "primary") {
                return control.down ? "#2a70d0" : (control.hovered ? "#4a98ff" : Theme.primaryBlue);
            } else if (control.variant === "danger") {
                return control.down ? "#54161b" : (control.hovered ? "#691b22" : "#3d1317");
            } else if (control.variant === "outline") {
                return control.down ? "#222226" : (control.hovered ? "#2a2a30" : "transparent");
            } else {
                return control.down ? "#1c1c20" : (control.hovered ? "#2a2a30" : Theme.bgContainerElevated);
            }
        }
        border.color: {
            if (control.variant === "danger") return "#ffb4ab80";
            if (control.variant === "primary") return "#3e90ff";
            if (control.variant === "outline") return control.hovered ? Theme.borderHighlight : Theme.borderBase;
            return control.hovered ? Theme.borderHighlight : Theme.borderBase;
        }
        border.width: 1

        Behavior on color { ColorAnimation { duration: 120 } }
        Behavior on border.color { ColorAnimation { duration: 120 } }
    }

    contentItem: Text {
        text: control.text
        font: control.font
        color: {
            if (control.customTextColor !== "") return control.customTextColor;
            if (control.variant === "danger") return Theme.errorRedLight;
            if (control.variant === "primary") return "#ffffff";
            return control.hovered ? "#ffffff" : Theme.textPrimary;
        }
        horizontalAlignment: Text.AlignHCenter
        verticalAlignment: Text.AlignVCenter
        elide: Text.ElideRight
    }
}
