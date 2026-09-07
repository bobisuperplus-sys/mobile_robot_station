import QtQuick
import QtQuick.Layouts
import QtQuick.Controls.Basic
import mobile_console
import "../components"

CardPanel {
    id: root

    cardRadius: Theme.radiusLg
    cardColor: "#0b0e14"
    borderColor: Theme.borderBase

    // 外部数据桥梁与接口
    property var mshClient: null
    signal navigateRequested(real goalX, real goalY)

    // 当前视图模式
    // 0: 2.5D 高程热力图 (车体居中动态局部 DEM)
    // 1: 3D 立体高程网格 (3D Wireframe Surface)
    // 2: 动力学收敛曲线 (Telemetry Curves)
    // 3: 2x2 旗舰综合态势 (4 大专业数据图表全景大屏)
    property int currentViewTab: 0

    // 视口感知模式
    property bool robotCentricMode: true  // true: 车体居中随动局部感知窗 [10m x 10m]; false: 全局宏观态势

    // 核心视口与投影参数
    property real viewPitch: 90.0         // 2.5D 正俯视 90°，3D 模式 38°
    property real viewYaw: 0.0            // 航向偏航角 (0° 代表正北朝上)
    property real panX: 0.0               // 视口平移 X (像素)
    property real panY: 0.0               // 视口平移 Y (像素)
    property real zoom: 1.0               // 缩放倍率 (0.4 ~ 4.0)
    property real meterScale: 22.0        // 基准物理比例尺 (1.0m = 22 像素)
    property real zGain: 3.5              // Z 轴高程物理视觉增益

    // 交互与状态
    property real hoverWorldX: 0.0        // 鼠标光标所指大地坐标 X (m)
    property real hoverWorldY: 0.0        // 鼠标光标所指大地坐标 Y (m)
    property real hoverWorldZ: 0.0        // 鼠标光标所指高程 Z (m)
    property bool isHovering: false

    // 交互选点状态
    property bool hasTarget: false        // 是否有目标点
    property real targetX: 0.0            // 目标点大地坐标 X (m)
    property real targetY: 6.0            // 目标点大地坐标 Y (m)
    property real targetZ: 0.12           // 目标点标高 Z (m)

    // 图层可见性
    property bool showTrajectory: true
    property bool showGrid: true
    property real pathDashOffset: 0
    property real radarSweepAngle: 0.0

    // 历史实跑轨迹存储数组 [{x, y, z}, ...]
    property var trailHistory: []

    // 全局增量探索高程栅格字典 (战争迷雾永久探明区域)
    property var globalElevationGrid: ({})
    property int exploredCellCount: 0
    property real exploredAreaM2: 0.0

    // 动力学收敛曲线历史数据 [{t, v, w, pitch, dist}, ...]
    property var telemetryHistory: []
    property real sessionStartTime: 0

    // 小车实时位姿抽取
    readonly property real carX: mshClient ? mshClient.posX : 0.0
    readonly property real carY: mshClient ? mshClient.posY : -6.0
    readonly property real carZ: mshClient ? mshClient.posZ : 0.01
    readonly property real carYaw: mshClient ? mshClient.yawDeg : 90.0
    readonly property real carPitch: mshClient ? mshClient.pitchDeg : 0.0
    readonly property real carLinVel: mshClient ? mshClient.linearVelocity : 0.0
    readonly property real carAngVel: mshClient ? mshClient.angularVelocity : 0.0

    onCurrentViewTabChanged: {
        if (currentViewTab === 0 || currentViewTab === 1) demCanvas.requestPaint();
        if (currentViewTab === 2) curveCanvas.requestPaint();
        if (currentViewTab === 3) {
            dashCanvasTopLeft.requestPaint();
            dashCanvasTopRight.requestPaint();
            dashCanvasBottomLeft.requestPaint();
            dashCanvasBottomRight.requestPaint();
        }
    }

    Component.onCompleted: {
        sessionStartTime = Date.now();
        resetView();
    }

    // 记录轨迹与动力学收敛曲线点
    onCarXChanged: recordTelemetryStep()
    onCarYChanged: recordTelemetryStep()
    onCarZChanged: recordTelemetryStep()
    onCarPitchChanged: recordTelemetryStep()

    function recordTelemetryStep() {
        // 同步增量地图建图
        root.updateGlobalExploration();

        // 1. 轨迹记录
        if (showTrajectory) {
            var last = trailHistory.length > 0 ? trailHistory[trailHistory.length - 1] : null;
            if (!last || Math.hypot(last.x - carX, last.y - carY) > 0.10) {
                trailHistory.push({ x: carX, y: carY, z: carZ });
                if (trailHistory.length > 500) trailHistory.shift();
            }
        }

        // 2. 动力学收敛曲线数据记录 (对应 Python 子图 4)
        var curT = (Date.now() - sessionStartTime) / 1000.0;
        var isStationary = (root.mshClient && (root.mshClient.navState === "ARRIVED" || root.mshClient.navState === "IDLE" || root.mshClient.navState === "BLOCKED" || root.mshClient.navState === "CANCELLED"));
        var targetDist = (root.hasTarget && (!root.mshClient || root.mshClient.hasNavGoal)) 
                         ? Math.hypot(root.targetX - root.carX, root.targetY - root.carY) 
                         : 0.0;
        if (isStationary) {
            targetDist = 0.0;
        }

        var curV = isStationary ? 0.0 : carLinVel;
        var curW = isStationary ? 0.0 : carAngVel;

        var lastTelem = telemetryHistory.length > 0 ? telemetryHistory[telemetryHistory.length - 1] : null;
        if (!lastTelem || (curT - lastTelem.t) >= 0.12) {
            telemetryHistory.push({
                t: curT,
                v: curV,
                w: curW,
                pitch: carPitch,
                dist: targetDist
            });
            if (telemetryHistory.length > 300) telemetryHistory.shift();
        }

        demCanvas.requestPaint();
        if (curveCanvas.visible) curveCanvas.requestPaint();
        if (dashCanvasTopLeft.visible) dashCanvasTopLeft.requestPaint();
        if (dashCanvasTopRight.visible) dashCanvasTopRight.requestPaint();
        if (dashCanvasBottomLeft.visible) dashCanvasBottomLeft.requestPaint();
        if (dashCanvasBottomRight.visible) dashCanvasBottomRight.requestPaint();
    }

    // 监听 MSH 客户端下发的导航目标、激光点云与连接状态
    Connections {
        target: mshClient
        function onNavGoalChanged() {
            if (mshClient && mshClient.hasNavGoal) {
                root.targetX = mshClient.navGoalX;
                root.targetY = mshClient.navGoalY;
                root.targetZ = getElevationAt(root.targetX, root.targetY) || 0.0;
                root.hasTarget = true;
            } else {
                root.hasTarget = false;
            }
            demCanvas.requestPaint();
            if (dashCanvasBottomRight.visible) dashCanvasBottomRight.requestPaint();
        }
        function onNavStateChanged() {
            if (mshClient && (mshClient.navState === "ARRIVED" || mshClient.navState === "CANCELLED")) {
                root.hasTarget = false;
            }
            demCanvas.requestPaint();
            if (dashCanvasBottomRight.visible) dashCanvasBottomRight.requestPaint();
        }
        function onLidarDataChanged() {
            root.updateGlobalExploration();
            demCanvas.requestPaint();
            if (dashCanvasTopLeft.visible) dashCanvasTopLeft.requestPaint();
            if (dashCanvasTopRight.visible) dashCanvasTopRight.requestPaint();
            if (dashCanvasBottomLeft.visible) dashCanvasBottomLeft.requestPaint();
        }
        function onIsConnectedChanged() {
            demCanvas.requestPaint();
            if (curveCanvas.visible) curveCanvas.requestPaint();
            if (dashCanvasTopLeft.visible) dashCanvasTopLeft.requestPaint();
            if (dashCanvasTopRight.visible) dashCanvasTopRight.requestPaint();
            if (dashCanvasBottomLeft.visible) dashCanvasBottomLeft.requestPaint();
            if (dashCanvasBottomRight.visible) dashCanvasBottomRight.requestPaint();
        }
    }

    // 雷达扫描波与动画定时器
    Timer {
        interval: 35
        running: true
        repeat: true
        onTriggered: {
            root.pathDashOffset = (root.pathDashOffset - 1) % 24;
            root.radarSweepAngle = (root.radarSweepAngle + 4.5) % 360;
            if (demCanvas.visible) demCanvas.requestPaint();
            if (dashCanvasBottomLeft.visible) dashCanvasBottomLeft.requestPaint();
        }
    }

    // 增量融合车载 20x20 局部实测高程网格至全局大地图字典 (战争迷雾逐区点亮)
    function updateGlobalExploration() {
        if (!mshClient || !mshClient.isConnected || !mshClient.localElevationGrid) return;
        var elevGrid = mshClient.localElevationGrid;
        if (elevGrid.length !== 400) return;

        var gSize = 20;
        var stepM = 0.50;
        var halfW = 5.0;
        var updated = false;

        for (var r = 0; r < gSize; r++) {
            for (var c = 0; c < gSize; c++) {
                var zVal = elevGrid[r * gSize + c];
                if (zVal !== null && zVal !== undefined) {
                    var dx = -halfW + (c + 0.5) * stepM;
                    var dy = halfW - (r + 0.5) * stepM;
                    var wx = root.carX + dx;
                    var wy = root.carY + dy;

                    var gx = Math.floor((wx + 0.25) / 0.50);
                    var gy = Math.floor((wy + 0.25) / 0.50);
                    var key = gx + "_" + gy;

                    if (root.globalElevationGrid[key] === undefined) {
                        root.globalElevationGrid[key] = Number(zVal);
                        root.exploredCellCount++;
                        updated = true;
                    } else {
                        root.globalElevationGrid[key] = Number(zVal);
                    }
                }
            }
        }
        if (updated) {
            root.exploredAreaM2 = root.exploredCellCount * 0.25; // 0.5m x 0.5m 每单元格
        }
    }

    // 重置已探明地图 (重新开始全区迷雾建图)
    function resetExplorationMap() {
        root.globalElevationGrid = ({});
        root.exploredCellCount = 0;
        root.exploredAreaM2 = 0.0;
        root.trailHistory = [];
        demCanvas.requestPaint();
        if (dashCanvasTopLeft.visible) dashCanvasTopLeft.requestPaint();
    }

    // 基于 3D 激光雷达实测高程读取地表高度 (优先全局已探明网格，回退实时局部网格)
    function getElevationAt(wx, wy) {
        var gx = Math.floor((wx + 0.25) / 0.50);
        var gy = Math.floor((wy + 0.25) / 0.50);
        var key = gx + "_" + gy;
        if (root.globalElevationGrid && root.globalElevationGrid[key] !== undefined) {
            return root.globalElevationGrid[key];
        }

        if (!mshClient || !mshClient.isConnected || !mshClient.localElevationGrid || mshClient.localElevationGrid.length !== 400) {
            return undefined;
        }
        var dx = wx - root.carX;
        var dy = wy - root.carY;
        if (Math.abs(dx) > 5.0 || Math.abs(dy) > 5.0) {
            return undefined;
        }
        var col = Math.floor((dx + 5.0) / 0.5);
        var row = Math.floor((5.0 - dy) / 0.5);
        col = Math.max(0, Math.min(19, col));
        row = Math.max(0, Math.min(19, row));
        var idx = row * 20 + col;
        var val = mshClient.localElevationGrid[idx];
        if (val === undefined || val === null) {
            return undefined;
        }
        return Number(val);
    }

    // 高程对应科学色板映射 (terrain 色谱：深海蓝 -> 青碧 -> 翠绿 -> 黄金 -> 砖褐 -> 浅灰白)
    function getElevationColor(z) {
        if (z <= 0.005) return "#0d1b38";       // 平整路面
        if (z <= 0.03)  return "#154360";       // 坡道起脚
        if (z <= 0.06)  return "#1e8449";       // 缓坡中下段
        if (z <= 0.09)  return "#27ae60";       // 缓坡中上段
        if (z <= 0.125) return "#f39c12";       // 观景高台顶面 (0.12m 核心目标)
        if (z <= 0.22)  return "#d35400";       // 环岛绿化台 (0.20m)
        if (z <= 0.38)  return "#784212";       // 中高障碍物
        return "#ecf0f1";                       // 建筑高墙 / 致命障碍 (0.45m)
    }

    // 坐标投影转换：世界三维坐标 (wx, wy, wz) -> 屏幕平面像素 (sx, sy)
    function worldToScreen(wx, wy, wz) {
        var cx = demCanvas.width / 2.0;
        var cy = demCanvas.height / 2.0;
        var radPitch = root.viewPitch * Math.PI / 180.0;
        var radYaw = root.viewYaw * Math.PI / 180.0;

        // 车体居中模式：以小车实时位置为原点平移
        var refX = root.robotCentricMode ? root.carX : 0.0;
        var refY = root.robotCentricMode ? root.carY : 0.0;

        var dx = wx - refX;
        var dy = wy - refY;

        var rx = dx * Math.cos(radYaw) - dy * Math.sin(radYaw);
        var ry = dx * Math.sin(radYaw) + dy * Math.cos(radYaw);

        var px = rx;
        var py = -ry * Math.sin(radPitch) - wz * Math.cos(radPitch) * root.zGain;

        var sx = cx + root.panX + px * root.meterScale * root.zoom;
        var sy = cy + root.panY + py * root.meterScale * root.zoom;
        return { x: sx, y: sy };
    }

    // 屏幕像素 (sx, sy) -> 世界地面坐标 (wx, wy)
    function screenToWorld(sx, sy) {
        var cx = demCanvas.width / 2.0;
        var cy = demCanvas.height / 2.0;
        var radPitch = root.viewPitch * Math.PI / 180.0;
        var radYaw = root.viewYaw * Math.PI / 180.0;

        var px = (sx - cx - root.panX) / (root.meterScale * root.zoom);
        var py = (sy - cy - root.panY) / (root.meterScale * root.zoom);

        var ry = -py / Math.max(0.1, Math.sin(radPitch));
        var rx = px;

        var dx = rx * Math.cos(-radYaw) - ry * Math.sin(-radYaw);
        var dy = rx * Math.sin(-radYaw) + ry * Math.cos(-radYaw);

        var refX = root.robotCentricMode ? root.carX : 0.0;
        var refY = root.robotCentricMode ? root.carY : 0.0;
        return { x: refX + dx, y: refY + dy };
    }

    // 视口平滑复位
    function resetView() {
        if (root.currentViewTab === 0) {
            root.viewPitch = 90.0;
            root.viewYaw = 0.0;
            root.panX = 0.0;
            root.panY = 0.0;
            root.zoom = 1.0;
        } else if (root.currentViewTab === 1) {
            root.viewPitch = 38.0;
            root.viewYaw = 0.0;
            root.panX = 0.0;
            root.panY = 50.0;
            root.zoom = 1.1;
        }
        demCanvas.requestPaint();
    }

    content: [
        // ======================== 视图 0 / 1: 主 2.5D/3D 画布容器 ========================
        Item {
            id: mainCanvasContainer
            anchors.fill: parent
            visible: root.currentViewTab === 0 || root.currentViewTab === 1

            Canvas {
                id: demCanvas
                anchors.fill: parent
                renderTarget: Canvas.FramebufferObject

                onPaint: {
                    var ctx = getContext("2d");
                    ctx.clearRect(0, 0, width, height);

                    var isOnline = root.mshClient && root.mshClient.isConnected;

                    // 1. 深空底色
                    ctx.fillStyle = "#07090e";
                    ctx.fillRect(0, 0, width, height);

                    // ---------------- 离线待机状态 (MSH 未启动 / 激光雷达离线) ----------------
                    if (!isOnline) {
                        var cx0 = width / 2.0;
                        var cy0 = height / 2.0;

                        // 绘制待机雷达同心圆
                        var standbyRings = [60, 120, 180, 240];
                        ctx.strokeStyle = "#141c2b";
                        ctx.lineWidth = 1.0;
                        ctx.setLineDash([4, 4]);
                        for (var sbi = 0; sbi < standbyRings.length; sbi++) {
                            ctx.beginPath();
                            ctx.arc(cx0, cy0, standbyRings[sbi], 0, Math.PI * 2);
                            ctx.stroke();
                        }
                        ctx.setLineDash([]);

                        // 待机十字刻度轴
                        ctx.strokeStyle = "#1a2538";
                        ctx.beginPath();
                        ctx.moveTo(cx0 - 260, cy0); ctx.lineTo(cx0 + 260, cy0);
                        ctx.moveTo(cx0, cy0 - 260); ctx.lineTo(cx0, cy0 + 260);
                        ctx.stroke();

                        // 旋转扫描波束
                        var swStandbyRad = root.radarSweepAngle * Math.PI / 180.0;
                        ctx.strokeStyle = "#3e90ff44";
                        ctx.lineWidth = 1.5;
                        ctx.beginPath();
                        ctx.moveTo(cx0, cy0);
                        ctx.lineTo(cx0 + 240 * Math.cos(swStandbyRad), cy0 - 240 * Math.sin(swStandbyRad));
                        ctx.stroke();

                        // 居中待机状态卡片
                        var boxW = 380, boxH = 86;
                        ctx.fillStyle = "#0d111ad9";
                        ctx.fillRect(cx0 - boxW / 2, cy0 - boxH / 2, boxW, boxH);
                        ctx.strokeStyle = "#252d3d";
                        ctx.lineWidth = 1.0;
                        ctx.strokeRect(cx0 - boxW / 2, cy0 - boxH / 2, boxW, boxH);

                        ctx.fillStyle = "#ff7b72";
                        ctx.font = "bold 12px sans-serif";
                        ctx.fillText("● MSH 微服务总线离线 (TCP: 9001 待机监听中)", cx0 - boxW / 2 + 24, cy0 - 14);

                        ctx.fillStyle = "#8892a0";
                        ctx.font = "11px sans-serif";
                        ctx.fillText("车载 16 线 3D 激光雷达传感器脱机 · 严禁先验地图作弊", cx0 - boxW / 2 + 24, cy0 + 8);
                        ctx.fillStyle = "#5c6b84";
                        ctx.font = "10px sans-serif";
                        ctx.fillText("请启动 python scripts/run_msh_server.py，建图由实测点云实时生成", cx0 - boxW / 2 + 24, cy0 + 26);
                        return;
                    }

                    // ---------------- 在线状态：基于车载激光雷达实测点云动态绘制局部高程热力图 ----------------
                    var pCar = worldToScreen(root.carX, root.carY, root.carZ);
                    var yawRad = root.carYaw * Math.PI / 180.0;

                    // 1.1 增量探索建图：绘制已探明的全局高程栅格 (战争迷雾已消除区域，永久保留地表高程)
                    var keys = Object.keys(root.globalElevationGrid);
                    var cellM = 0.50;
                    var halfCell = 0.25;

                    for (var k = 0; k < keys.length; k++) {
                        var kStr = keys[k];
                        var pIdx = kStr.indexOf("_");
                        var gx = parseInt(kStr.substring(0, pIdx));
                        var gy = parseInt(kStr.substring(pIdx + 1));
                        var zVal = root.globalElevationGrid[kStr];

                        var wx0 = gx * cellM - halfCell;
                        var wy0 = gy * cellM - halfCell;

                        var sp00 = worldToScreen(wx0, wy0 + cellM, (root.viewPitch < 80 ? zVal : 0));
                        var sp10 = worldToScreen(wx0 + cellM, wy0 + cellM, (root.viewPitch < 80 ? zVal : 0));
                        var sp11 = worldToScreen(wx0 + cellM, wy0, (root.viewPitch < 80 ? zVal : 0));
                        var sp01 = worldToScreen(wx0, wy0, (root.viewPitch < 80 ? zVal : 0));

                        // 视口粗裁剪
                        var minX = Math.min(sp00.x, sp11.x);
                        var maxX = Math.max(sp00.x, sp11.x);
                        var minY = Math.min(sp00.y, sp11.y);
                        var maxY = Math.max(sp00.y, sp11.y);
                        if (maxX < -20 || minX > width + 20 || maxY < -20 || minY > height + 20) {
                            continue;
                        }

                        ctx.fillStyle = getElevationColor(zVal);
                        ctx.beginPath();
                        ctx.moveTo(sp00.x, sp00.y); ctx.lineTo(sp10.x, sp10.y); ctx.lineTo(sp11.x, sp11.y); ctx.lineTo(sp01.x, sp01.y);
                        ctx.closePath();
                        ctx.fill();

                        if (root.showGrid) {
                            ctx.strokeStyle = "#16203033";
                            ctx.lineWidth = 0.4;
                            ctx.stroke();
                        }
                    }

                    // 1.2 当前正在探测的动态感知边界框 (以小车为中心 10m x 10m 虚线发光框)
                    var sb00 = worldToScreen(root.carX - 5.0, root.carY + 5.0, 0);
                    var sb10 = worldToScreen(root.carX + 5.0, root.carY + 5.0, 0);
                    var sb11 = worldToScreen(root.carX + 5.0, root.carY - 5.0, 0);
                    var sb01 = worldToScreen(root.carX - 5.0, root.carY - 5.0, 0);
                    ctx.save();
                    ctx.strokeStyle = "#00e5ff55";
                    ctx.lineWidth = 1.0;
                    ctx.setLineDash([6, 4]);
                    ctx.beginPath();
                    ctx.moveTo(sb00.x, sb00.y); ctx.lineTo(sb10.x, sb10.y); ctx.lineTo(sb11.x, sb11.y); ctx.lineTo(sb01.x, sb01.y);
                    ctx.closePath();
                    ctx.stroke();
                    ctx.restore();

                    // 1.2 车载动态同心警戒测距环
                    if (root.robotCentricMode) {
                        var rings = [
                            { r: 2.0, color: "#34c75955", label: "2.0m 安全" },
                            { r: 4.0, color: "#f59e0b44", label: "4.0m 警戒" },
                            { r: 5.0, color: "#3e90ff33", label: "5.0m 边界" }
                        ];

                        ctx.save();
                        ctx.font = "8px 'JetBrains Mono', monospace";
                        for (var ri = 0; ri < rings.length; ri++) {
                            var ring = rings[ri];
                            var rPix = ring.r * root.meterScale * root.zoom;
                            ctx.strokeStyle = ring.color;
                            ctx.lineWidth = 1.0;
                            ctx.setLineDash([4, 4]);
                            ctx.beginPath();
                            ctx.arc(pCar.x, pCar.y, rPix, 0, Math.PI * 2);
                            ctx.stroke();

                            ctx.fillStyle = ring.color;
                            ctx.fillText(ring.label, pCar.x + rPix + 4, pCar.y + 3);
                        }
                        ctx.restore();
                    }

                    // 1.3 用户鼠标点选的导航目标点 (如有)
                    if (root.hasTarget) {
                        var pGoal = worldToScreen(root.targetX, root.targetY, root.targetZ || 0);
                        ctx.strokeStyle = "#ff1744";
                        ctx.fillStyle = "#ff174455";
                        ctx.lineWidth = 2.0;
                        ctx.beginPath();
                        ctx.arc(pGoal.x, pGoal.y, 9, 0, Math.PI * 2);
                        ctx.stroke();
                        ctx.fillStyle = "#ff1744";
                        ctx.font = "14px sans-serif";
                        ctx.fillText("★", pGoal.x - 6, pGoal.y + 5);
                        ctx.font = "bold 9px 'JetBrains Mono', monospace";
                        ctx.fillText("目标 (" + root.targetX.toFixed(1) + "," + root.targetY.toFixed(1) + ")", pGoal.x + 12, pGoal.y + 3);
                    }

                    // 1.4 实跑黄色轨迹 (真实记录自小车运动)
                    if (root.showTrajectory && root.trailHistory.length > 1) {
                        ctx.save();
                        ctx.strokeStyle = "#ffeb3b";
                        ctx.lineWidth = 2.5;
                        ctx.beginPath();
                        for (var tIdx = 0; tIdx < root.trailHistory.length; tIdx++) {
                            var tp = worldToScreen(root.trailHistory[tIdx].x, root.trailHistory[tIdx].y, root.trailHistory[tIdx].z);
                            if (tIdx === 0) ctx.moveTo(tp.x, tp.y);
                            else ctx.lineTo(tp.x, tp.y);
                        }
                        ctx.stroke();
                        ctx.restore();
                    }

                    // 1.5 车身轮廓与前向雷达扫描视锥
                    var fovDist = 3.6 * root.meterScale * root.zoom;
                    var ang1 = yawRad - (30 * Math.PI / 180.0);
                    var ang2 = yawRad + (30 * Math.PI / 180.0);
                    ctx.fillStyle = "#34c75922";
                    ctx.strokeStyle = "#34c75955";
                    ctx.lineWidth = 1.0;
                    ctx.beginPath();
                    ctx.moveTo(pCar.x, pCar.y);
                    ctx.arc(pCar.x, pCar.y, fovDist, -ang2, -ang1);
                    ctx.closePath();
                    ctx.fill(); ctx.stroke();

                    // 车体外圆
                    var carR = Math.max(8, 0.22 * root.meterScale * root.zoom);
                    ctx.strokeStyle = "#ffeb3b";
                    ctx.lineWidth = 2.0;
                    ctx.fillStyle = "#182030";
                    ctx.beginPath();
                    ctx.arc(pCar.x, pCar.y, carR, 0, Math.PI * 2);
                    ctx.fill(); ctx.stroke();

                    // 车头方向指示线
                    var arrowLen = carR * 1.6;
                    ctx.strokeStyle = "#00e5ff";
                    ctx.lineWidth = 2.5;
                    ctx.beginPath();
                    ctx.moveTo(pCar.x, pCar.y);
                    ctx.lineTo(pCar.x + arrowLen * Math.cos(-yawRad), pCar.y + arrowLen * Math.sin(-yawRad));
                    ctx.stroke();

                    // 质心光斑
                    ctx.fillStyle = "#ffeb3b";
                    ctx.beginPath();
                    ctx.arc(pCar.x, pCar.y, 3, 0, Math.PI * 2);
                    ctx.fill();
                }
            }

            // 鼠标交互控制
            MouseArea {
                id: canvasMouseArea
                anchors.fill: parent
                hoverEnabled: true
                acceptedButtons: Qt.LeftButton | Qt.RightButton | Qt.MiddleButton

                property real lastX: 0
                property real lastY: 0
                property bool isDragging: false

                onPressed: function(mouse) {
                    lastX = mouse.x;
                    lastY = mouse.y;
                    isDragging = true;
                }

                onPositionChanged: function(mouse) {
                    var world = root.screenToWorld(mouse.x, mouse.y);
                    root.hoverWorldX = world.x;
                    root.hoverWorldY = world.y;
                    root.hoverWorldZ = root.getElevationAt(world.x, world.y) || 0.0;
                    root.isHovering = true;

                    if (isDragging) {
                        var dx = mouse.x - lastX;
                        var dy = mouse.y - lastY;
                        if (mouse.buttons & Qt.RightButton) {
                            root.viewYaw = (root.viewYaw + dx * 0.5) % 360;
                            root.viewPitch = Math.max(15, Math.min(90, root.viewPitch - dy * 0.3));
                        } else {
                            root.panX += dx;
                            root.panY += dy;
                        }
                        lastX = mouse.x;
                        lastY = mouse.y;
                        demCanvas.requestPaint();
                    }
                }

                onReleased: function(mouse) { isDragging = false; }
                onExited: { root.isHovering = false; }

                onWheel: function(wheel) {
                    var factor = wheel.angleDelta.y > 0 ? 1.12 : 0.89;
                    root.zoom = Math.max(0.4, Math.min(4.0, root.zoom * factor));
                    demCanvas.requestPaint();
                }

                onClicked: function(mouse) {
                    if (mouse.button === Qt.LeftButton) {
                        var world = root.screenToWorld(mouse.x, mouse.y);
                        root.targetX = world.x;
                        root.targetY = world.y;
                        root.targetZ = root.getElevationAt(world.x, world.y);
                        root.hasTarget = true;
                        root.navigateRequested(world.x, world.y);
                        demCanvas.requestPaint();
                    }
                }
            }
        },

        // ======================== 视图 2: 动力学收敛曲线单屏模式 ========================
        Item {
            id: curveViewContainer
            anchors.fill: parent
            visible: root.currentViewTab === 2

            Canvas {
                id: curveCanvas
                anchors.fill: parent
                renderTarget: Canvas.FramebufferObject

                onPaint: {
                    var ctx = getContext("2d");
                    ctx.clearRect(0, 0, width, height);

                    ctx.fillStyle = "#07090e";
                    ctx.fillRect(0, 0, width, height);

                    var padL = 55, padR = 55, padT = 48, padB = 40;
                    var plotW = width - padL - padR;
                    var plotH = height - padT - padB;
                    if (plotW <= 20 || plotH <= 20) return;

                    ctx.fillStyle = "#05070a";
                    ctx.fillRect(padL, padT, plotW, plotH);
                    ctx.strokeStyle = "#1a2130";
                    ctx.lineWidth = 1.0;
                    ctx.strokeRect(padL, padT, plotW, plotH);

                    // 左 Y 轴刻度 (-15° ~ +2.5°/速度)
                    var yTicks = [-15, -10, -5, 0, 2.5];
                    ctx.font = "9px 'JetBrains Mono', monospace";
                    ctx.fillStyle = "#8892a0";
                    for (var i = 0; i < yTicks.length; i++) {
                        var yVal = yTicks[i];
                        var py = padT + plotH - ((yVal - (-15.0)) / (17.5)) * plotH;
                        ctx.strokeStyle = (yVal === 0) ? "#3e90ff44" : "#141c2b";
                        ctx.lineWidth = (yVal === 0) ? 1.2 : 0.8;
                        ctx.beginPath();
                        ctx.moveTo(padL, py); ctx.lineTo(padL + plotW, py); ctx.stroke();
                        ctx.fillText(yVal.toFixed(1), padL - 34, py + 3);
                    }

                    // 右 Y 轴刻度 (目标距离 0 ~ 12m)
                    ctx.fillStyle = "#ff4081";
                    for (var rD = 0; rD <= 12; rD += 3) {
                        var ry = padT + plotH - (rD / 12.0) * plotH;
                        ctx.fillText(rD + "m", padL + plotW + 8, ry + 3);
                    }

                    var isOnline = root.mshClient && root.mshClient.isConnected;
                    if (!isOnline) {
                        ctx.fillStyle = "#ff7b72";
                        ctx.font = "bold 12px sans-serif";
                        ctx.fillText("● MSH 微服务总线脱机 (离线待机)", width / 2 - 100, height / 2 - 10);
                        ctx.fillStyle = "#5c6b84";
                        ctx.font = "11px sans-serif";
                        ctx.fillText("等待连接服务后采样小车爬坡与动力学收敛曲线", width / 2 - 130, height / 2 + 15);
                        return;
                    }

                    var nPts = root.telemetryHistory.length;
                    if (nPts < 2) {
                        ctx.fillStyle = "#5c6b84";
                        ctx.font = "12px sans-serif";
                        ctx.fillText("等待小车运行，采样遥测动力学收敛数据...", width / 2 - 120, height / 2);
                        return;
                    }

                    var tMin = root.telemetryHistory[0].t;
                    var tMax = Math.max(tMin + 10, root.telemetryHistory[nPts - 1].t);
                    function tToX(t) { return padL + ((t - tMin) / (tMax - tMin)) * plotW; }
                    function leftValToY(v) { return padT + plotH - ((v - (-15.0)) / (17.5)) * plotH; }
                    function rightDistToY(d) { return padT + plotH - (Math.min(12.0, d) / 12.0) * plotH; }

                    // 1. 目标距离收敛折线 (洋红虚线)
                    ctx.save();
                    ctx.strokeStyle = "#ff4081"; ctx.lineWidth = 2.0; ctx.setLineDash([4, 4]);
                    ctx.beginPath();
                    for (var dI = 0; dI < nPts; dI++) {
                        var dx = tToX(root.telemetryHistory[dI].t);
                        var dy = rightDistToY(root.telemetryHistory[dI].dist);
                        if (dI === 0) ctx.moveTo(dx, dy); else ctx.lineTo(dx, dy);
                    }
                    ctx.stroke(); ctx.restore();

                    // 2. 车身爬坡俯仰角 Pitch (亮黄实线)
                    ctx.save();
                    ctx.strokeStyle = "#ffeb3b"; ctx.lineWidth = 2.2;
                    ctx.beginPath();
                    for (var pI = 0; pI < nPts; pI++) {
                        var px = tToX(root.telemetryHistory[pI].t);
                        var py = leftValToY(root.telemetryHistory[pI].pitch);
                        if (pI === 0) ctx.moveTo(px, py); else ctx.lineTo(px, py);
                    }
                    ctx.stroke(); ctx.restore();

                    // 3. 角速度 ω (橙色虚线)
                    ctx.save();
                    ctx.strokeStyle = "#ff9100"; ctx.lineWidth = 1.6; ctx.setLineDash([5, 3]);
                    ctx.beginPath();
                    for (var wI = 0; wI < nPts; wI++) {
                        var wx = tToX(root.telemetryHistory[wI].t);
                        var wy = leftValToY(root.telemetryHistory[wI].w);
                        if (wI === 0) ctx.moveTo(wx, wy); else ctx.lineTo(wx, wy);
                    }
                    ctx.stroke(); ctx.restore();

                    // 4. 线速度 v (青色实线)
                    ctx.save();
                    ctx.strokeStyle = "#00e5ff"; ctx.lineWidth = 2.0;
                    ctx.beginPath();
                    for (var vI = 0; vI < nPts; vI++) {
                        var vx = tToX(root.telemetryHistory[vI].t);
                        var vy = leftValToY(root.telemetryHistory[vI].v);
                        if (vI === 0) ctx.moveTo(vx, vy); else ctx.lineTo(vx, vy);
                    }
                    ctx.stroke(); ctx.restore();
                }
            }
        },

        // ======================== 视图 3: 2x2 旗舰综合态势 (4 大专业数据图表全景大屏) ========================
        GridLayout {
            id: dashboardGrid
            anchors.fill: parent
            anchors.topMargin: 44
            anchors.margins: 8
            columns: 2
            rows: 2
            visible: root.currentViewTab === 3

            // --- 窗口 1 (左上): 2.5D 车载局部动态高程热力图表 ---
            Rectangle {
                Layout.fillWidth: true
                Layout.fillHeight: true
                color: "#080b10"
                border.color: Theme.borderBase
                radius: Theme.radiusSm
                clip: true

                Canvas {
                    id: dashCanvasTopLeft
                    anchors.fill: parent
                    renderTarget: Canvas.FramebufferObject

                    onPaint: {
                        var ctx = getContext("2d");
                        ctx.clearRect(0, 0, width, height);

                        var cx = width / 2.0;
                        var cy = height / 2.0 + 10;

                        var isOnline = root.mshClient && root.mshClient.isConnected;
                        if (!isOnline) {
                            ctx.fillStyle = "#ff7b72";
                            ctx.font = "bold 11px sans-serif";
                            ctx.fillText("● MSH 微服务离线 · 传感器脱机", cx - 85, cy - 10);
                            ctx.fillStyle = "#5c6b84";
                            ctx.font = "10px sans-serif";
                            ctx.fillText("待机中 (等待 3D 激光雷达点云接入)", cx - 85, cy + 12);
                            return;
                        }

                        // 在线实测渲染 20x20 激光点云局部高程网格 (10m x 10m，分辨率 0.5m)
                        var elevGrid = root.mshClient.localElevationGrid;
                        var gSize = 20;
                        var cellSize = Math.min((width - 40) / 20.0, (height - 60) / 20.0);
                        var startX = cx - (gSize * cellSize) / 2.0;
                        var startY = cy - (gSize * cellSize) / 2.0;

                        for (var r = 0; r < gSize; r++) {
                            for (var c = 0; c < gSize; c++) {
                                var zVal = elevGrid ? elevGrid[r * gSize + c] : null;
                                var cellX = startX + c * cellSize;
                                var cellY = startY + r * cellSize;
                                if (zVal !== null && zVal !== undefined) {
                                    ctx.fillStyle = getElevationColor(zVal);
                                    ctx.fillRect(cellX, cellY, cellSize + 0.5, cellSize + 0.5);
                                } else {
                                    ctx.fillStyle = "#0a0e16";
                                    ctx.fillRect(cellX, cellY, cellSize, cellSize);
                                    ctx.strokeStyle = "#121824";
                                    ctx.lineWidth = 0.5;
                                    ctx.strokeRect(cellX, cellY, cellSize, cellSize);
                                }
                            }
                        }

                        // 警戒同心圆 2m, 4m (以小车中心为基准，10m 对应 gSize * cellSize)
                        var mPix = (gSize * cellSize) / 10.0;
                        ctx.strokeStyle = "#34c75944"; ctx.lineWidth = 1.0;
                        ctx.beginPath(); ctx.arc(cx, cy, 2.0 * mPix, 0, Math.PI * 2); ctx.stroke();
                        ctx.strokeStyle = "#f59e0b33";
                        ctx.beginPath(); ctx.arc(cx, cy, 4.0 * mPix, 0, Math.PI * 2); ctx.stroke();

                        // 小车中心与航向
                        var yaw = root.carYaw * Math.PI / 180.0;
                        ctx.fillStyle = "#ffeb3b";
                        ctx.beginPath(); ctx.arc(cx, cy, 5, 0, Math.PI * 2); ctx.fill();
                        ctx.strokeStyle = "#00e5ff"; ctx.lineWidth = 2.0;
                        ctx.beginPath(); ctx.moveTo(cx, cy);
                        ctx.lineTo(cx + 14 * Math.cos(-yaw), cy + 14 * Math.sin(-yaw));
                        ctx.stroke();
                    }
                }

                Text {
                    anchors.top: parent.top; anchors.left: parent.left; anchors.margins: 8
                    text: "2.5D 车载局部动态高程热力图表 (10m x 10m 实测高程)"
                    font.pixelSize: 10; font.bold: true; color: Theme.lightBlue
                }

                Text {
                    anchors.bottom: parent.bottom; anchors.right: parent.right; anchors.margins: 8
                    text: (root.mshClient && root.mshClient.isConnected && root.mshClient.lidarConnected)
                          ? "实测点云: " + root.mshClient.lidarPointCount + " 点 · 栅格 20x20 (0.5m)"
                          : "传感器脱机 · 待机"
                    font.family: Theme.fontFamilyMono; font.pixelSize: 9
                    color: (root.mshClient && root.mshClient.isConnected) ? "#00e5ff" : Theme.textMuted
                }
            }

            // --- 窗口 2 (右上): 3D 立体实测地貌与空间位姿轨迹图表 ---
            Rectangle {
                Layout.fillWidth: true
                Layout.fillHeight: true
                color: "#080b10"
                border.color: Theme.borderBase
                radius: Theme.radiusSm
                clip: true

                Canvas {
                    id: dashCanvasTopRight
                    anchors.fill: parent
                    renderTarget: Canvas.FramebufferObject

                    onPaint: {
                        var ctx = getContext("2d");
                        ctx.clearRect(0, 0, width, height);

                        var cx = width / 2.0;
                        var cy = height / 2.0 + 35;
                        var radP = 36 * Math.PI / 180.0;
                        var sc = 14.0;

                        function p3(x, y, z) {
                            var px = (x - root.carX) * sc;
                            var py = -(y - root.carY) * Math.sin(radP) * sc - (z - root.carZ) * Math.cos(radP) * sc * 3.5;
                            return { x: cx + px, y: cy + py };
                        }

                        var isOnline = root.mshClient && root.mshClient.isConnected;
                        if (!isOnline) {
                            ctx.fillStyle = "#ff7b72";
                            ctx.font = "bold 11px sans-serif";
                            ctx.fillText("● MSH 微服务离线 · 空间轨迹脱机", cx - 90, cy - 25);
                            ctx.fillStyle = "#5c6b84";
                            ctx.font = "10px sans-serif";
                            ctx.fillText("待机中 (无实测数据，严禁先验虚构)", cx - 80, cy - 5);
                            return;
                        }

                        // 绘制实测 3D 局部地形网格 (基于 localElevationGrid 绘制实测坡度与高台)
                        var elevGrid = root.mshClient.localElevationGrid;
                        if (elevGrid && elevGrid.length === 400) {
                            var gSize = 20;
                            var stepM = 0.50;
                            var halfW = 5.0;
                            for (var r = 0; r < gSize - 1; r += 2) {
                                for (var c = 0; c < gSize - 1; c += 2) {
                                    var zVal = elevGrid[r * gSize + c];
                                    if (zVal !== null && zVal !== undefined) {
                                        var x0 = root.carX - halfW + c * stepM;
                                        var y0 = root.carY + halfW - r * stepM;
                                        var p0 = p3(x0, y0, zVal);
                                        var p1 = p3(x0 + stepM * 2, y0, zVal);
                                        var p2 = p3(x0 + stepM * 2, y0 - stepM * 2, zVal);
                                        var p3_ = p3(x0, y0 - stepM * 2, zVal);

                                        ctx.fillStyle = getElevationColor(zVal) + "44";
                                        ctx.strokeStyle = getElevationColor(zVal) + "aa";
                                        ctx.lineWidth = 0.8;
                                        ctx.beginPath();
                                        ctx.moveTo(p0.x, p0.y); ctx.lineTo(p1.x, p1.y); ctx.lineTo(p2.x, p2.y); ctx.lineTo(p3_.x, p3_.y);
                                        ctx.closePath(); ctx.fill(); ctx.stroke();
                                    }
                                }
                            }
                        }

                        // 小车当前 3D 位置与空间爬坡轨迹
                        if (root.trailHistory.length > 1) {
                            ctx.strokeStyle = "#00e5ff"; ctx.lineWidth = 2.2;
                            ctx.beginPath();
                            for (var ti = 0; ti < root.trailHistory.length; ti++) {
                                var pt = p3(root.trailHistory[ti].x, root.trailHistory[ti].y, root.trailHistory[ti].z);
                                if (ti === 0) ctx.moveTo(pt.x, pt.y); else ctx.lineTo(pt.x, pt.y);
                            }
                            ctx.stroke();
                        }

                        var curPt = p3(root.carX, root.carY, root.carZ);
                        ctx.fillStyle = "#ffeb3b"; ctx.beginPath(); ctx.arc(curPt.x, curPt.y, 5, 0, Math.PI * 2); ctx.fill();
                    }
                }

                Text {
                    anchors.top: parent.top; anchors.left: parent.left; anchors.margins: 8
                    text: "3D 立体实测地貌与空间位姿轨迹"
                    font.pixelSize: 10; font.bold: true; color: Theme.lightBlue
                }

                Text {
                    anchors.bottom: parent.bottom; anchors.right: parent.right; anchors.margins: 8
                    text: (root.mshClient && root.mshClient.isConnected)
                          ? "位姿: (" + root.carX.toFixed(2) + ", " + root.carY.toFixed(2) + ", Z:" + root.carZ.toFixed(2) + "m) · 俯仰:" + root.carPitch.toFixed(1) + "°"
                          : "位姿: (0.00, 0.00, Z:0.00m) · 脱机待机"
                    font.family: Theme.fontFamilyMono; font.pixelSize: 9
                    color: (root.mshClient && root.mshClient.isConnected) ? "#00e5ff" : Theme.textMuted
                }
            }

            // --- 窗口 3 (左下): 激光雷达 360° 极坐标测距剖面图表 ---
            Rectangle {
                Layout.fillWidth: true
                Layout.fillHeight: true
                color: "#080b10"
                border.color: Theme.borderBase
                radius: Theme.radiusSm
                clip: true

                Canvas {
                    id: dashCanvasBottomLeft
                    anchors.fill: parent
                    renderTarget: Canvas.FramebufferObject

                    onPaint: {
                        var ctx = getContext("2d");
                        ctx.clearRect(0, 0, width, height);

                        var cx = width / 2.0;
                        var cy = height / 2.0 + 12;
                        var maxR = Math.min(width, height) * 0.38;

                        // 极坐标测距同心圆 (1m, 3m, 5m)
                        var dists = [1.0, 3.0, 5.0];
                        ctx.font = "8px 'JetBrains Mono', monospace";
                        for (var i = 0; i < dists.length; i++) {
                            var rPix = (dists[i] / 5.0) * maxR;
                            ctx.strokeStyle = (i === 0) ? "#d7001555" : ((i === 1) ? "#f59e0b44" : "#1a2130");
                            ctx.lineWidth = 1.0;
                            ctx.beginPath(); ctx.arc(cx, cy, rPix, 0, Math.PI * 2); ctx.stroke();
                            ctx.fillStyle = "#5c6b84";
                            ctx.fillText(dists[i].toFixed(1) + "m", cx + rPix + 3, cy + 3);
                        }

                        // 8 向径向刻度线
                        for (var a = 0; a < 360; a += 45) {
                            var rad = a * Math.PI / 180.0;
                            ctx.strokeStyle = "#141c2b"; ctx.lineWidth = 0.8;
                            ctx.beginPath(); ctx.moveTo(cx, cy);
                            ctx.lineTo(cx + maxR * Math.cos(rad), cy + maxR * Math.sin(rad));
                            ctx.stroke();
                        }

                        var isOnline = root.mshClient && root.mshClient.isConnected;
                        if (!isOnline || !root.mshClient.lidarConnected) {
                            ctx.fillStyle = "#5c6b84";
                            ctx.font = "10px sans-serif";
                            ctx.fillText("雷达脱机待机 (0 束有效回波)", cx - 65, cy + maxR - 10);
                            return;
                        }

                        // 绘制 72 束真实实测激光雷达障碍物
                        var ranges = root.mshClient.lidarRanges;
                        var beamCount = ranges ? ranges.length : 0;

                        if (beamCount > 0) {
                            ctx.save();

                            // 1. 绘制实体障碍物连续轮廓线 (相邻相近回波连线成实体表面)
                            ctx.lineWidth = 2.0;
                            for (var b = 0; b < beamCount; b++) {
                                var d1 = ranges[b];
                                if (d1 === null || d1 === undefined) continue;

                                var nextB = (b + 1) % beamCount;
                                var d2 = ranges[nextB];
                                if (d2 === null || d2 === undefined) continue;

                                if (Math.abs(d1 - d2) < 0.65) {
                                    var ang1 = -Math.PI / 2.0 + (b / beamCount) * Math.PI * 2.0;
                                    var ang2 = -Math.PI / 2.0 + (nextB / beamCount) * Math.PI * 2.0;
                                    var r1 = (Math.min(5.0, d1) / 5.0) * maxR;
                                    var r2 = (Math.min(5.0, d2) / 5.0) * maxR;

                                    var x1 = cx + r1 * Math.cos(ang1);
                                    var y1 = cy + r1 * Math.sin(ang1);
                                    var x2 = cx + r2 * Math.cos(ang2);
                                    var y2 = cy + r2 * Math.sin(ang2);

                                    var avgD = (d1 + d2) * 0.5;
                                    ctx.strokeStyle = avgD < 1.0 ? "#ff1744" : (avgD < 2.5 ? "#f59e0b" : "#00e5ff");
                                    ctx.beginPath();
                                    ctx.moveTo(x1, y1);
                                    ctx.lineTo(x2, y2);
                                    ctx.stroke();
                                }
                            }

                            // 2. 绘制各障碍激光回波点光斑与细微测距光束
                            for (var b = 0; b < beamCount; b++) {
                                var d = ranges[b];
                                if (d === null || d === undefined) continue;

                                var bAng = -Math.PI / 2.0 + (b / beamCount) * Math.PI * 2.0;
                                var rPix = (Math.min(5.0, d) / 5.0) * maxR;
                                var bx = cx + rPix * Math.cos(bAng);
                                var by = cy + rPix * Math.sin(bAng);

                                var ptColor = d < 1.0 ? "#ff1744" : (d < 2.5 ? "#f59e0b" : "#00e5ff");

                                ctx.strokeStyle = d < 1.0 ? "#ff174433" : (d < 2.5 ? "#f59e0b22" : "#00e5ff18");
                                ctx.lineWidth = 0.8;
                                ctx.beginPath();
                                ctx.moveTo(cx, cy);
                                ctx.lineTo(bx, by);
                                ctx.stroke();

                                ctx.fillStyle = ptColor;
                                ctx.beginPath();
                                ctx.arc(bx, by, d < 1.0 ? 3.5 : 2.5, 0, Math.PI * 2);
                                ctx.fill();
                            }
                            ctx.restore();
                        }

                        // 旋转雷达扫描光束
                        var swRad = (root.radarSweepAngle) * Math.PI / 180.0;
                        ctx.strokeStyle = "#34c759cc"; ctx.lineWidth = 1.5;
                        ctx.beginPath(); ctx.moveTo(cx, cy);
                        ctx.lineTo(cx + maxR * Math.cos(swRad), cy - maxR * Math.sin(swRad));
                        ctx.stroke();

                        // 中心小车与前向箭头
                        ctx.fillStyle = "#ffeb3b"; ctx.beginPath(); ctx.arc(cx, cy, 4, 0, Math.PI * 2); ctx.fill();
                        ctx.fillStyle = "#00e5ff"; ctx.font = "9px sans-serif";
                        ctx.fillText("▲ 前向", cx - 12, cy - maxR - 4);

                        // 最近障碍物预警 HUD (置于窗口顶部偏左，与底部点数信息分离)
                        if (root.mshClient && root.mshClient.hasObstacle) {
                            var obsDist = root.mshClient.closestObstacleDist;
                            var obsAng = root.mshClient.closestObstacleAngle;
                            var isDanger = obsDist < 0.8;
                            var isWarn = obsDist < 2.0;

                            ctx.save();
                            ctx.font = "bold 9px 'JetBrains Mono', monospace";
                            ctx.fillStyle = isDanger ? "#ff1744" : (isWarn ? "#f59e0b" : "#00e5ff");
                            var warnText = isDanger ? "【危险】防撞避障" : (isWarn ? "【注意】减速通过" : "【安全】正常通行");
                            ctx.fillText("最近障碍: " + obsDist.toFixed(2) + "m @ " + (obsAng >= 0 ? "+" : "") + obsAng.toFixed(1) + "°  " + warnText, 8, 26);
                            ctx.restore();
                        } else {
                            ctx.save();
                            ctx.font = "9px 'JetBrains Mono', monospace";
                            ctx.fillStyle = "#34c759";
                            ctx.fillText("前方开阔 · 5.0m 内无实体障碍物", 8, 26);
                            ctx.restore();
                        }
                    }
                }

                Text {
                    anchors.top: parent.top; anchors.left: parent.left; anchors.margins: 8
                    text: "激光雷达 360° 极坐标测距与避障剖面"
                    font.pixelSize: 10; font.bold: true; color: Theme.lightBlue
                }

                Text {
                    anchors.bottom: parent.bottom; anchors.right: parent.right; anchors.margins: 8
                    text: (root.mshClient && root.mshClient.isConnected && root.mshClient.lidarConnected)
                          ? "72 扇区实测剖面 · 探测点云: " + root.mshClient.lidarPointCount + " 点"
                          : "传感器脱机 · 待机"
                    font.family: Theme.fontFamilyMono; font.pixelSize: 9
                    color: (root.mshClient && root.mshClient.isConnected) ? "#00e5ff" : Theme.textMuted
                }
            }

            // --- 窗口 4 (右下): 底盘动力学指令、车身爬坡俯仰角与距离收敛曲线图表 ---
            Rectangle {
                Layout.fillWidth: true
                Layout.fillHeight: true
                color: "#080b10"
                border.color: Theme.borderBase
                radius: Theme.radiusSm
                clip: true

                Canvas {
                    id: dashCanvasBottomRight
                    anchors.fill: parent
                    renderTarget: Canvas.FramebufferObject

                    onPaint: {
                        var ctx = getContext("2d");
                        ctx.clearRect(0, 0, width, height);

                        var padL = 38, padR = 38, padT = 32, padB = 24;
                        var plotW = width - padL - padR;
                        var plotH = height - padT - padB;
                        if (plotW <= 20 || plotH <= 20) return;

                        ctx.fillStyle = "#05070a"; ctx.fillRect(padL, padT, plotW, plotH);
                        ctx.strokeStyle = "#1a2130"; ctx.strokeRect(padL, padT, plotW, plotH);

                        // 绘制示波器参考网格与 Y 轴刻度
                        var yTicks = [-15, -10, -5, 0, 2.5];
                        ctx.font = "8px 'JetBrains Mono', monospace";
                        ctx.fillStyle = "#5c6b84";
                        for (var i = 0; i < yTicks.length; i++) {
                            var yVal = yTicks[i];
                            var py = padT + plotH - ((yVal - (-15.0)) / 17.5) * plotH;
                            ctx.strokeStyle = (yVal === 0) ? "#3e90ff33" : "#121824";
                            ctx.lineWidth = 0.8;
                            ctx.beginPath(); ctx.moveTo(padL, py); ctx.lineTo(padL + plotW, py); ctx.stroke();
                            ctx.fillText(yVal.toFixed(0), padL - 22, py + 3);
                        }

                        // 右 Y 轴刻度 (目标距离 0 ~ 12m)
                        ctx.fillStyle = "#ff4081aa";
                        for (var rD = 0; rD <= 12; rD += 4) {
                            var ry = padT + plotH - (rD / 12.0) * plotH;
                            ctx.fillText(rD + "m", padL + plotW + 4, ry + 3);
                        }

                        var isOnline = root.mshClient && root.mshClient.isConnected;
                        if (!isOnline) {
                            ctx.fillStyle = "#ff7b72";
                            ctx.font = "bold 10px sans-serif";
                            ctx.fillText("● MSH 微服务脱机 · 待机中", padL + plotW / 2 - 75, padT + plotH / 2);
                            return;
                        }

                        var nPts = root.telemetryHistory.length;
                        if (nPts < 2) {
                            ctx.fillStyle = "#4a5568";
                            ctx.font = "10px sans-serif";
                            ctx.fillText("等待小车行驶，采样动力学遥测信号...", padL + plotW / 2 - 90, padT + plotH / 2);
                            return;
                        }

                        var tMin = root.telemetryHistory[0].t;
                        var tMax = Math.max(tMin + 8, root.telemetryHistory[nPts - 1].t);
                        function tToX(t) { return padL + ((t - tMin) / (tMax - tMin)) * plotW; }
                        function leftY(v) { return padT + plotH - ((v - (-15.0)) / 17.5) * plotH; }
                        function rightY(d) { return padT + plotH - (Math.min(12.0, d) / 12.0) * plotH; }

                        // 1. 目标残差距离 (洋红虚线)
                        ctx.save(); ctx.strokeStyle = "#ff4081"; ctx.lineWidth = 1.8; ctx.setLineDash([3, 3]);
                        ctx.beginPath();
                        for (var di = 0; di < nPts; di++) {
                            var dx = tToX(root.telemetryHistory[di].t);
                            var dy = rightY(root.telemetryHistory[di].dist);
                            if (di === 0) ctx.moveTo(dx, dy); else ctx.lineTo(dx, dy);
                        }
                        ctx.stroke(); ctx.restore();

                        // 2. 爬坡俯仰角 Pitch (亮黄实线)
                        ctx.save(); ctx.strokeStyle = "#ffeb3b"; ctx.lineWidth = 2.0;
                        ctx.beginPath();
                        for (var pi = 0; pi < nPts; pi++) {
                            var px = tToX(root.telemetryHistory[pi].t);
                            var py = leftY(root.telemetryHistory[pi].pitch);
                            if (pi === 0) ctx.moveTo(px, py); else ctx.lineTo(px, py);
                        }
                        ctx.stroke(); ctx.restore();

                        // 3. 线速度 v (青色实线)
                        ctx.save(); ctx.strokeStyle = "#00e5ff"; ctx.lineWidth = 1.6;
                        ctx.beginPath();
                        for (var vi = 0; vi < nPts; vi++) {
                            var vx = tToX(root.telemetryHistory[vi].t);
                            var vy = leftY(root.telemetryHistory[vi].v);
                            if (vi === 0) ctx.moveTo(vx, vy); else ctx.lineTo(vx, vy);
                        }
                        ctx.stroke(); ctx.restore();
                    }
                }

                Text {
                    anchors.top: parent.top; anchors.left: parent.left; anchors.margins: 8
                    text: "底盘动力学与状态收敛"
                    font.pixelSize: 10; font.bold: true; color: Theme.lightBlue
                }

                Row {
                    anchors.top: parent.top; anchors.right: parent.right; anchors.margins: 8
                    spacing: 8
                    Text { text: "v:" + root.carLinVel.toFixed(2) + "m/s"; font.family: Theme.fontFamilyMono; font.pixelSize: 8; color: "#00e5ff" }
                    Text { text: "Pitch:" + root.carPitch.toFixed(1) + "°"; font.family: Theme.fontFamilyMono; font.pixelSize: 8; color: "#ffeb3b" }
                    Text { text: "Dist:" + (root.hasTarget ? Math.hypot(root.targetX - root.carX, root.targetY - root.carY).toFixed(1) : "0.0") + "m"; font.family: Theme.fontFamilyMono; font.pixelSize: 8; color: "#ff4081" }
                }
            }
        },

        // ======================== 顶部专业态势控制条 (Tab + 工具) ========================
        Rectangle {
            id: topControlBar
            anchors.top: parent.top
            anchors.left: parent.left
            anchors.right: parent.right
            anchors.margins: 8
            height: 30
            radius: Theme.radiusSm
            color: "#14161ef2"
            border.color: Theme.borderBase
            border.width: 1

            RowLayout {
                anchors.fill: parent
                anchors.leftMargin: 4
                anchors.rightMargin: 4
                spacing: 4

                // 左侧：4 个专业视图模式 Tab
                Row {
                    spacing: 4
                    Layout.alignment: Qt.AlignVCenter

                    // Tab 0: 2.5D 高程热力图
                    Rectangle {
                        width: tab0Text.implicitWidth + 12
                        height: 22
                        radius: Theme.radiusSm - 1
                        color: root.currentViewTab === 0 ? Theme.primaryBlue : "transparent"
                        anchors.verticalCenter: parent.verticalCenter
                        Text {
                            id: tab0Text
                            anchors.centerIn: parent
                            text: "2.5D 高程热力图"
                            font.family: Theme.fontFamilyNormal
                            font.pixelSize: 10
                            font.bold: root.currentViewTab === 0
                            color: root.currentViewTab === 0 ? "#ffffff" : Theme.textMuted
                        }
                        MouseArea {
                            anchors.fill: parent
                            cursorShape: Qt.PointingHandCursor
                            onClicked: {
                                root.currentViewTab = 0;
                                root.viewPitch = 90.0;
                                root.resetView();
                            }
                        }
                    }

                    // Tab 1: 3D 立体网格
                    Rectangle {
                        width: tab1Text.implicitWidth + 12
                        height: 22
                        radius: Theme.radiusSm - 1
                        color: root.currentViewTab === 1 ? Theme.primaryBlue : "transparent"
                        anchors.verticalCenter: parent.verticalCenter
                        Text {
                            id: tab1Text
                            anchors.centerIn: parent
                            text: "3D 立体网格"
                            font.family: Theme.fontFamilyNormal
                            font.pixelSize: 10
                            font.bold: root.currentViewTab === 1
                            color: root.currentViewTab === 1 ? "#ffffff" : Theme.textMuted
                        }
                        MouseArea {
                            anchors.fill: parent
                            cursorShape: Qt.PointingHandCursor
                            onClicked: {
                                root.currentViewTab = 1;
                                root.viewPitch = 38.0;
                                root.resetView();
                            }
                        }
                    }

                    // Tab 2: 动力学收敛曲线
                    Rectangle {
                        width: tab2Text.implicitWidth + 12
                        height: 22
                        radius: Theme.radiusSm - 1
                        color: root.currentViewTab === 2 ? Theme.primaryBlue : "transparent"
                        anchors.verticalCenter: parent.verticalCenter
                        Text {
                            id: tab2Text
                            anchors.centerIn: parent
                            text: "动力学收敛曲线"
                            font.family: Theme.fontFamilyNormal
                            font.pixelSize: 10
                            font.bold: root.currentViewTab === 2
                            color: root.currentViewTab === 2 ? "#ffffff" : Theme.textMuted
                        }
                        MouseArea {
                            anchors.fill: parent
                            cursorShape: Qt.PointingHandCursor
                            onClicked: {
                                root.currentViewTab = 2;
                                curveCanvas.requestPaint();
                            }
                        }
                    }

                    // Tab 3: 2x2 旗舰综合态势 (4 大专业图表)
                    Rectangle {
                        width: tab3Text.implicitWidth + 12
                        height: 22
                        radius: Theme.radiusSm - 1
                        color: root.currentViewTab === 3 ? Theme.primaryBlue : "transparent"
                        anchors.verticalCenter: parent.verticalCenter
                        Text {
                            id: tab3Text
                            anchors.centerIn: parent
                            text: "2x2 综合态势大屏"
                            font.family: Theme.fontFamilyNormal
                            font.pixelSize: 10
                            font.bold: root.currentViewTab === 3
                            color: root.currentViewTab === 3 ? "#ffffff" : Theme.textMuted
                        }
                        MouseArea {
                            anchors.fill: parent
                            cursorShape: Qt.PointingHandCursor
                            onClicked: {
                                root.currentViewTab = 3;
                            }
                        }
                    }
                }

                Item { Layout.fillWidth: true }

                // 右侧快捷工具按钮
                Row {
                    spacing: 4
                    Layout.alignment: Qt.AlignVCenter

                    // 车体居中跟随 / 自由漫游 切换
                    ActionButton {
                        text: root.robotCentricMode ? "🎯 锁定车体" : "🌐 自由漫游"
                        variant: root.robotCentricMode ? "primary" : "outline"
                        implicitWidth: 84
                        implicitHeight: 22
                        customFontSize: 9
                        onClicked: {
                            root.robotCentricMode = !root.robotCentricMode;
                            demCanvas.requestPaint();
                        }
                    }

                    // 清空地图 (重新探索)
                    ActionButton {
                        text: "清空地图"
                        variant: "outline"
                        implicitWidth: 58
                        implicitHeight: 22
                        customFontSize: 9
                        onClicked: root.resetExplorationMap()
                    }

                    // 视口复位
                    ActionButton {
                        text: "复位"
                        variant: "ghost"
                        implicitWidth: 40
                        implicitHeight: 22
                        customFontSize: 9
                        onClicked: root.resetView()
                    }
                }
            }
        },

        // ======================== 右侧科学高程标尺 Colorbar (仅在 2.5D 热力图模式展示) ========================
        Rectangle {
            id: colorbarCard
            visible: root.currentViewTab === 0
            anchors.top: parent.top
            anchors.right: parent.right
            anchors.margins: 10
            anchors.topMargin: 46
            width: 72
            height: 220
            radius: Theme.radiusSm
            color: "#10131ae8"
            border.color: Theme.borderBase
            border.width: 1

            Column {
                anchors.fill: parent
                anchors.margins: 6
                spacing: 4

                Text {
                    anchors.horizontalCenter: parent.horizontalCenter
                    text: "高程 Z (m)"
                    font.family: Theme.fontFamilyNormal
                    font.pixelSize: 9
                    font.bold: true
                    color: Theme.lightBlue
                }

                Row {
                    anchors.horizontalCenter: parent.horizontalCenter
                    spacing: 6
                    height: 185

                    // 垂直渐变色彩色条
                    Rectangle {
                        width: 12
                        height: parent.height
                        radius: 2
                        border.color: "#30363d"
                        border.width: 1
                        gradient: Gradient {
                            GradientStop { position: 0.00; color: "#ecf0f1" } // 0.45m 高墙/障碍
                            GradientStop { position: 0.20; color: "#784212" } // 0.35m
                            GradientStop { position: 0.50; color: "#d35400" } // 0.20m 中央环岛
                            GradientStop { position: 0.72; color: "#f39c12" } // 0.12m 登顶高台
                            GradientStop { position: 0.86; color: "#27ae60" } // 0.06m 爬坡中段
                            GradientStop { position: 1.00; color: "#0d1b38" } // 0.00m 基准地面
                        }
                    }

                    // 刻度标签
                    Column {
                        height: parent.height
                        spacing: 12
                        anchors.verticalCenter: parent.verticalCenter

                        Text { text: "0.45m"; font.family: Theme.fontFamilyMono; font.pixelSize: 8; color: Theme.textPrimary }
                        Text { text: "0.35m"; font.family: Theme.fontFamilyMono; font.pixelSize: 8; color: Theme.textMuted }
                        Text { text: "0.20m"; font.family: Theme.fontFamilyMono; font.pixelSize: 8; color: Theme.textMuted }
                        Text { text: "0.12m"; font.family: Theme.fontFamilyMono; font.pixelSize: 8; font.bold: true; color: "#ffeb3b" }
                        Text { text: "0.06m"; font.family: Theme.fontFamilyMono; font.pixelSize: 8; color: "#2ecc71" }
                        Text { text: "0.00m"; font.family: Theme.fontFamilyMono; font.pixelSize: 8; color: Theme.textMuted }
                    }
                }
            }
        },

        // ======================== 左下角悬浮 HUD 状态栏 ========================
        Rectangle {
            visible: root.currentViewTab === 0 || root.currentViewTab === 1
            anchors.bottom: parent.bottom
            anchors.left: parent.left
            anchors.margins: 8
            height: 24
            width: hudText.implicitWidth + 16
            radius: Theme.radiusSm
            color: "#10121ae6"
            border.color: Theme.borderBase
            border.width: 1

            Text {
                id: hudText
                anchors.centerIn: parent
                text: root.isHovering
                      ? "光标坐标: (" + (root.hoverWorldX >= 0 ? "+" : "") + root.hoverWorldX.toFixed(2) + ", "
                        + (root.hoverWorldY >= 0 ? "+" : "") + root.hoverWorldY.toFixed(2) + ") · 估算标高 Z: " + root.hoverWorldZ.toFixed(2) + "m"
                      : (root.mshClient && root.mshClient.isConnected
                         ? "增量 SLAM 战争迷雾探索 · 已探明面积: " + root.exploredAreaM2.toFixed(1) + " m² (" + root.exploredCellCount + " 单元) · 实时感知窗 10m x 10m"
                         : "传感器脱机 · 待机等待 MSH 微服务连接")
                font.family: Theme.fontFamilyMono
                font.pixelSize: 9
                color: root.isHovering ? "#00e5ff" : (root.mshClient && root.mshClient.isConnected ? Theme.successGreen : Theme.textMuted)
            }
        }
    ]
}
