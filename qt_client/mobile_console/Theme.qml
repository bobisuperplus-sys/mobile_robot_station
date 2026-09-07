pragma Singleton
import QtQuick

QtObject {
    id: theme

    // 背景色基准
    readonly property color bgCanvas: "#121214"
    readonly property color bgSurface: "#161618"
    readonly property color bgContainer: "#1a1a1d"
    readonly property color bgContainerElevated: "#202024"
    readonly property color bgHeader: "#131315"
    readonly property color bgDarkest: "#0e0e10"

    // 边框色
    readonly property color borderBase: "#2a2a2d"
    readonly property color borderSubtle: "#222226"
    readonly property color borderActive: "#3e90ff"
    readonly property color borderHighlight: "#353438"

    // 语义状态色
    readonly property color primaryBlue: "#3e90ff"
    readonly property color lightBlue: "#aac7ff"
    readonly property color successGreen: "#34c759"
    readonly property color warningAmber: "#f59e0b"
    readonly property color warningLight: "#e5a93c"
    readonly property color errorRed: "#d70015"
    readonly property color errorRedLight: "#ffb4ab"
    readonly property color errorBg: "#3a1417"

    // 文本色
    readonly property color textPrimary: "#f0eff2"
    readonly property color textSecondary: "#aac7ff"
    readonly property color textMuted: "#8e8d92"
    readonly property color textDarkMuted: "#606066"
    readonly property color textFaint: "#46464b"

    // 字体配置
    readonly property string fontFamilyNormal: "PingFang SC, Microsoft YaHei, SF Pro, sans-serif"
    readonly property string fontFamilyMono: "JetBrains Mono, Consolas, monospace"

    // 常用圆角
    readonly property real radiusSm: 4
    readonly property real radiusMd: 8
    readonly property real radiusLg: 12
    readonly property real radiusFull: 9999
}
