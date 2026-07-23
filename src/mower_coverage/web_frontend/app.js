/**
 * app.js — 自动割草机 Web 前端主逻辑
 *
 * 功能：
 *   1. 显示 Leaflet 卫星地图
 *   2. 在地图上画多边形区域
 *   3. 通过 rosbridge WebSocket 与 ROS2 通信
 *   4. 发送区域到割草机，触发规划和执行
 *   5. 实时显示机器人状态和覆盖率
 *
 * 通信协议（WebSocket ↔ ROS2）:
 *   发送区域: 调用 /multi_area 服务
 *   规划:     调用 /multi_area/plan 服务
 *   执行:     调用 /multi_area/plan_and_start 服务
 *   状态:     订阅 /coverage/statistics 话题
 */

// =============================================================
// 配置
// =============================================================
const CONFIG = {
    rosbridgeUrl: `${window.location.protocol === 'https:' ? 'wss' : 'ws'}://${window.location.hostname || 'localhost'}:9090`,
    mapTileUrl: 'https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png',
    mapTileAttr: '&copy; <a href="https://openstreetmap.org">OSM</a>',
    satelliteTileUrl: 'https://mt1.google.com/vt/lyrs=s&x={x}&y={y}&z={z}',
    initialCenter: [22.5431, 114.0579],  // 深圳坐标（仿真默认 GPS）
    initialZoom: 18,
};

// =============================================================
// 状态
// =============================================================
const state = {
    ros: null,
    connected: false,
    areas: [],           // { id, name, latlngs, color, layer }
    obstacles: [],       // { id, name, latlngs, color, layer }
    robotPose: null,     // { lat, lng }
    robotHeading: null,  // 罗盘角度：北 0°、东 90°
    robotMarker: null,   // Leaflet marker for robot
    pathLine: null,      // Leaflet polyline for coverage path
    captureLine: null,   // 当前遥控采集原始轨迹
    capturePreviewItems: null, // 完成/草稿状态的采集预览
    _capturePreviewSignature: null,
    antennaMarker: null, // 真实 RTK 天线位置（与仿真机器人分开）
    missionItems: null,  // 已确认对象/草稿图层
    coveragePercent: 0,
    robotMode: 'idle',
    execMode: 'idle',        // 'executing' | 'paused' | 'stopped' | 'idle'
    rosSubscribed: false, // 是否已订阅 ROS 话题（防重复订阅）
    _odomReceived: false, // 是否收到过 odom 数据
    captureState: null,
    mission: { objects: [], drafts: [], order: [] },
    planningLoaded: false,
    _missionGeometrySignature: null,
    captureCorrectionMode: false,
    captureCorrectionDraw: null,
    rtkStatus: null,
    realRtkFix: null,
    captureCommandTopic: null,
    teleopTopic: null,
    drawMode: 'area',     // 'area' | 'obstacle'
    _wasExecuting: false, // 上次执行状态（用于检测完成）
};

const COLORS = ['#00ff99', '#4488ff', '#ffcc00', '#ff6600', '#ff4488'];
const OBSTACLE_COLOR = '#ff2222'; // 障碍物的固定颜色
let polygonIdCounter = 0;
let obstacleIdCounter = 0;

// =============================================================
// Leaflet 地图初始化
// =============================================================
const map = L.map('map', {
    center: CONFIG.initialCenter,
    zoom: CONFIG.initialZoom,
    zoomControl: true,
});
// 采集路线放在机器人 marker 之上，避免当前位置标记遮住首尾连接。
map.createPane('captureRoutePane');
map.getPane('captureRoutePane').style.zIndex = 650;

// 卫星图图层
const satelliteLayer = L.tileLayer(CONFIG.satelliteTileUrl, {
    maxZoom: 20,
    attribution: '&copy; Google Maps',
});

// 标准地图图层（备选）
const streetLayer = L.tileLayer(CONFIG.mapTileUrl, {
    maxZoom: 19,
    attribution: CONFIG.mapTileAttr,
});

satelliteLayer.addTo(map);

// 图层切换
L.control.layers({
    '🛰️ 卫星': satelliteLayer,
    '🗺️ 标准': streetLayer,
}).addTo(map);

// =============================================================
// 机器人位置标记（初始在中心点）
// =============================================================
const robotIcon = L.divIcon({
    className: 'robot-marker',
    html: '<div class="robot-heading-icon" title="车头方向未知">'
        + '<div class="robot-heading-arrow"></div>'
        + '<div class="robot-heading-body">M</div>'
        + '</div>',
    iconSize: [34, 34],
    iconAnchor: [17, 17],
});
state.robotMarker = L.marker(CONFIG.initialCenter, {
    icon: robotIcon,
    zIndexOffset: 1000,
}).addTo(map);

const antennaIcon = L.divIcon({
    className: 'rtk-marker',
    html: '<div style="background:#4488ff;color:#fff;border:2px solid #fff;border-radius:12px;padding:3px 6px;box-shadow:0 0 10px rgba(68,136,255,0.8);">RTK</div>',
    iconSize: [42, 24],
    iconAnchor: [21, 12],
});
state.antennaMarker = L.marker(CONFIG.initialCenter, {
    icon: antennaIcon,
    zIndexOffset: 900,
    opacity: 0.0,
}).addTo(map);

// 路径图层（显示规划的全覆盖路径）
state.pathLine = L.polyline([], {
    color: '#ffcc00',
    weight: 3,
    opacity: 0.8,
    dashArray: '8, 6',
}).addTo(map);

state.captureLine = L.polyline([], {
    color: '#00ddff',
    weight: 4,
    opacity: 0.95,
    pane: 'captureRoutePane',
}).addTo(map);
state.missionItems = new L.FeatureGroup();
map.addLayer(state.missionItems);
state.capturePreviewItems = new L.FeatureGroup();
map.addLayer(state.capturePreviewItems);

// =============================================================
// 绘图控制（Leaflet.draw）
// =============================================================
const drawnItems = new L.FeatureGroup();
map.addLayer(drawnItems);

const drawControl = new L.Control.Draw({
    position: 'topright',
    draw: {
        polygon: {
            allowIntersection: false,
            showArea: true,
            shapeOptions: { color: '#00ff99', weight: 2 },
        },
        circle: false,
        rectangle: false,
        marker: false,
        circlemarker: false,
        polyline: false,
    },
    edit: {
        featureGroup: drawnItems,
        remove: true,
    },
});
map.addControl(drawControl);

// 画完多边形后
map.on(L.Draw.Event.CREATED, function (event) {
    const layer = event.layer;

    if (state.captureCorrectionMode) {
        handleCaptureCorrectionLayer(layer);
        return;
    }

    if (state.drawMode === 'obstacle') {
        // ---- 障碍物模式：红色多边形 ----
        const latlngs = layer.getLatLngs()[0].map(ll => [ll.lat, ll.lng]);
        layer.setStyle({
            color: OBSTACLE_COLOR,
            fillColor: OBSTACLE_COLOR,
            fillOpacity: 0.3,
            weight: 3,
            dashArray: '6, 4',
        });
        drawnItems.addLayer(layer);

        const name = `障碍物 ${obstacleIdCounter + 1}`;
        const center = layer.getBounds().getCenter();
        const label = L.marker(center, {
            icon: L.divIcon({
                className: 'area-label',
                html: `<div style="background:${OBSTACLE_COLOR};color:#fff;padding:2px 8px;border-radius:10px;font-size:11px;font-weight:bold;white-space:nowrap;">⛔ ${name}</div>`,
                iconSize: [0, 0],
            }),
            interactive: false,
        });
        drawnItems.addLayer(label);

        state.obstacles.push({ id: name, name, latlngs, color: OBSTACLE_COLOR, layer, label });
        obstacleIdCounter++;
    } else {
        // ---- 区域模式：绿色多边形（原有逻辑） ----
        const latlngs = layer.getLatLngs()[0].map(ll => [ll.lat, ll.lng]);
        const color = COLORS[polygonIdCounter % COLORS.length];
        const name = `区域 ${polygonIdCounter + 1}`;
        polygonIdCounter++;

        layer.setStyle({
            color: color,
            fillColor: color,
            fillOpacity: 0.25,
            weight: 2,
        });
        drawnItems.addLayer(layer);

        const center = layer.getBounds().getCenter();
        const label = L.marker(center, {
            icon: L.divIcon({
                className: 'area-label',
                html: `<div style="background:${color};color:#111;padding:2px 8px;border-radius:10px;font-size:11px;font-weight:bold;white-space:nowrap;">${name}</div>`,
                iconSize: [0, 0],
            }),
            interactive: false,
        });
        drawnItems.addLayer(label);

        state.areas.push({ id: name, name, latlngs, color, layer, label });
    }

    updateAreaList();
    updateSendButton();
});

// 删除多边形时
map.on(L.Draw.Event.DELETED, function () {
    syncAreasFromMap();
    updateAreaList();
    updateSendButton();
});

map.on(L.Draw.Event.EDITED, function () {
    syncAreasFromMap();
});

function syncAreasFromMap() {
    state.areas = [];
    state.obstacles = [];
    drawnItems.eachLayer(layer => {
        if (layer.getLatLngs) {
            const latlngs = layer.getLatLngs()[0].map(ll => [ll.lat, ll.lng]);
            const color = layer.options.color || '#00ff99';
            const isObstacle = color === OBSTACLE_COLOR;
            if (isObstacle) {
                state.obstacles.push({ id: `obs_${state.obstacles.length}`, name: `障碍物`, latlngs, color });
            } else {
                state.areas.push({ id: `area_${state.areas.length}`, name: `区域`, latlngs, color });
            }
        }
    });
}

// =============================================================
// ROS2 通信（rosbridge WebSocket）
// =============================================================
function connectROS() {
    state.ros = new ROSLIB.Ros({ url: CONFIG.rosbridgeUrl });

    state.ros.on('connection', function () {
        state.connected = true;
        document.getElementById('ros-status').innerHTML =
            '<span class="status-dot connected"></span>已连接';
        updateSendButton();
        setupROSSubscribers();
        updateCaptureUI();
    });

    state.ros.on('close', function () {
        state.connected = false;
        state.rosSubscribed = false;  // 下次重连允许重新订阅
        cancelCaptureManualCorrect(false);
        stopTeleop();
        document.getElementById('ros-status').innerHTML =
            '<span class="status-dot disconnected"></span>未连接';
        document.getElementById('btn-send').disabled = true;
        document.getElementById('btn-plan').disabled = true;
        document.getElementById('btn-execute').disabled = true;
        state.planningLoaded = false;
        updateCaptureUI();
    });

    state.ros.on('error', function (err) {
        console.error('ROS连接错误:', err);
        // 5秒后重试
        setTimeout(connectROS, 5000);
    });
}

function setupROSSubscribers() {
    // ★ 防止重复订阅：ROS 重连时不会再次创建新订阅
    if (state.rosSubscribed) return;
    state.rosSubscribed = true;

    state.captureCommandTopic = new ROSLIB.Topic({
        ros: state.ros,
        name: '/mission/capture/command',
        messageType: 'std_msgs/String',
    });
    state.teleopTopic = new ROSLIB.Topic({
        ros: state.ros,
        name: '/teleop/cmd_vel',
        messageType: 'geometry_msgs/Twist',
    });

    // 1. 订阅 GPS 位置 → 显示 GPS 状态和调试坐标（不直接控制标记）
    const gpsTopic = new ROSLIB.Topic({
        ros: state.ros,
        name: '/gps/fix',
        messageType: 'sensor_msgs/NavSatFix',
    });
    gpsTopic.subscribe(msg => {
        const lat = msg.latitude;
        const lng = msg.longitude;
        const status = msg.status?.status;

        // 更新状态文本
        const statusText = status >= 0 ? '🛰️ 仿真GNSS: 有效' : '🛰️ 仿真GNSS: 无定位';
        document.getElementById('gps-status').textContent = statusText;

        // 调试：显示原始 GPS 坐标
        document.getElementById('debug-coords').textContent =
            'GPS: ' + lat.toFixed(6) + ', ' + lng.toFixed(6);

        // 仅当从未收到 odom 时，用 GPS 做初始定位
        if (!state._odomReceived && lat !== 0 && lng !== 0) {
            state.robotPose = { lat, lng };
            state.robotMarker.setLatLng([lat, lng]);
        }
    });

    // 1a. 真实 RTK 天线位置：与仿真机器人位置严格分开显示。
    const realFixTopic = new ROSLIB.Topic({
        ros: state.ros,
        name: '/rtk/gps/fix',
        messageType: 'sensor_msgs/NavSatFix',
    });
    realFixTopic.subscribe(msg => {
        const lat = Number(msg.latitude);
        const lng = Number(msg.longitude);
        const valid = Number.isFinite(lat) && Number.isFinite(lng)
            && Math.abs(lat) > 1e-9 && Math.abs(lng) > 1e-9;
        state.realRtkFix = valid ? { lat, lng } : null;
        if (valid) {
            state.antennaMarker.setLatLng([lat, lng]);
            state.antennaMarker.setOpacity(1.0);
            state.antennaMarker.bindTooltip(
                `RTK 天线 · ${lat.toFixed(7)}, ${lng.toFixed(7)}`);
        } else {
            state.antennaMarker.setOpacity(0.0);
        }
        updateRealRtkStatus();
    });

    const rtkStatusTopic = new ROSLIB.Topic({
        ros: state.ros,
        name: '/rtk/status',
        messageType: 'std_msgs/String',
    });
    rtkStatusTopic.subscribe(msg => {
        try {
            state.rtkStatus = JSON.parse(msg.data);
        } catch (error) {
            state.rtkStatus = { state: 'INVALID', last_error: '状态消息不是 JSON' };
        }
        updateRealRtkStatus();
    });

    const healthTopic = new ROSLIB.Topic({
        ros: state.ros,
        name: '/localization/health',
        messageType: 'std_msgs/String',
    });
    healthTopic.subscribe(msg => {
        try {
            const data = JSON.parse(msg.data);
            const stateText = data.state || '--';
            const reasons = Array.isArray(data.reasons) ? data.reasons.join('; ') : '';
            document.getElementById('localization-status').textContent =
                `🛡️ 定位: ${stateText}${reasons ? ' · ' + reasons : ''}`;
        } catch (error) {
            document.getElementById('localization-status').textContent = '🛡️ 定位: 消息无效';
        }
    });

    const captureStateTopic = new ROSLIB.Topic({
        ros: state.ros,
        name: '/mission/capture/state',
        messageType: 'std_msgs/String',
    });
    captureStateTopic.subscribe(msg => {
        try {
            state.captureState = JSON.parse(msg.data);
            renderCapturePreview();
            updateCaptureUI();
        } catch (error) {
            showToast('❌ 采集状态消息无效', 'error');
        }
    });

    const missionTopic = new ROSLIB.Topic({
        ros: state.ros,
        name: '/mission/capture/mission',
        messageType: 'std_msgs/String',
    });
    missionTopic.subscribe(msg => {
        try {
            const previousSignature = state._missionGeometrySignature;
            state.mission = JSON.parse(msg.data);
            renderMissionItems();
            renderCapturePreview();
            if (previousSignature !== state._missionGeometrySignature) {
                state.planningLoaded = false;
                document.getElementById('btn-plan').disabled = true;
                document.getElementById('btn-execute').disabled = true;
            }
            updateCaptureUI();
        } catch (error) {
            showToast('❌ 任务消息无效', 'error');
        }
    });

    const rawPathTopic = new ROSLIB.Topic({
        ros: state.ros,
        name: '/mission/capture/raw_path',
        messageType: 'nav_msgs/Path',
    });
    rawPathTopic.subscribe(msg => {
        const latlngs = (msg.poses || [])
            .map(p => approxLocalToGPS(p.pose.position.x, p.pose.position.y));
        state.captureLine.setLatLngs(latlngs);
    });

    // 1b. 订阅 /odom → 实时更新机器人位置（odom 随机器人移动而变化）
    const odomTopic = new ROSLIB.Topic({
        ros: state.ros,
        name: '/odom',
        messageType: 'nav_msgs/Odometry',
    });
    odomTopic.subscribe(msg => {
        const x = msg.pose.pose.position.x;
        const y = msg.pose.pose.position.y;
        const [lat, lng] = approxLocalToGPS(x, y);
        const heading = updateRobotHeading(msg.pose.pose.orientation);
        state._odomReceived = true;
        state.robotPose = { lat, lng, heading };
        state.robotMarker.setLatLng([lat, lng]);
        const headingText = heading === null
            ? ' · 车头 --'
            : ` · 车头 ${heading.toFixed(0)}°`;
        document.getElementById('sim-pose-status').textContent =
            `🤖 仿真位姿: ${x.toFixed(2)}, ${y.toFixed(2)}m${headingText}`;
    });

    // 2. 订阅覆盖率统计和执行状态
    const covTopic = new ROSLIB.Topic({
        ros: state.ros,
        name: '/coverage/statistics',
        messageType: 'std_msgs/String',
    });
    covTopic.subscribe(msg => {
        try {
            const data = JSON.parse(msg.data);
            const pct = data.coverage_percent || 0;
            state.coveragePercent = pct;
            document.getElementById('coverage-display').textContent = `📊 ${pct.toFixed(1)}%`;
            // 如果有执行状态信息
            if (data.executing !== undefined) {
                state.robotMode = data.executing ? 'executing' : 'idle';
                if (data.executing) {
                    state.execMode = 'executing';
                    document.getElementById('mode-display').textContent = '▶️ 执行中';
                } else if (pct >= 99.9 && state._wasExecuting) {
                    // 自然完成 → idle
                    state.execMode = 'idle';
                    document.getElementById('mode-display').textContent = '✅ 已完成';
                    showToast('✅ 全覆盖任务执行完毕！', 'success');
                } else if (state.execMode === 'executing') {
                    // coverage 报告停止但前端没主动 stop/pause → 视为完成或异常停止
                    state.execMode = 'idle';
                    document.getElementById('mode-display').textContent = '⚙️ 待机';
                }
                // ★ 更新暂停/继续/停止按钮状态
                updateExecButtons();
                state._wasExecuting = data.executing;
            }
            if (data.safety_stop_reason === 'scan_unavailable') {
                document.getElementById('mode-display').textContent =
                    '⚠️ 安全停车：等待新鲜 /scan';
            } else if (data.safety_stop_reason === 'obstacle') {
                const distance = Number(data.obstacle_distance);
                const suffix = Number.isFinite(distance)
                    ? `（障碍物 ${distance.toFixed(2)}m）` : '';
                document.getElementById('mode-display').textContent =
                    `⚠️ 避障停车${suffix}`;
            }
            if (data.covered && data.total) {
                state._progressStr = `${data.covered}/${data.total}`;
            }
        } catch (e) {
            // 非 JSON 格式，忽略
        }
    });

    // 3. 订阅全覆盖路径 → 在地图上画路径
    // 防抖：首次收到路径才自动回中，后续 1Hz 重发不跳动
    const pathTopic = new ROSLIB.Topic({
        ros: state.ros,
        name: '/coverage/multi_path',
        messageType: 'nav_msgs/Path',
    });
    pathTopic.subscribe(msg => {
        const latlngs = msg.poses
            .filter(p => p.pose.position.x !== 0 || p.pose.position.y !== 0)
            .map(p => {
                return approxLocalToGPS(p.pose.position.x, p.pose.position.y);
            })
            .filter(ll => ll !== null);
        if (latlngs.length > 0) {
            state.pathLine.setLatLngs(latlngs);
            // 首次收到路径才自动回中，后续接收不跳动
            if (state.pathLine.getLatLngs().length === 0 || !state._pathEverFitted) {
                map.fitBounds(state.pathLine.getBounds().pad(0.1));
                state._pathEverFitted = true;
            }
        } else {
            // ★ 收到空路径 → 清除显示的路径线（路径已被清除/重新规划）
            state.pathLine.setLatLngs([]);
        }
    });
}

function normalizeHeadingDegrees(degrees) {
    const normalized = degrees % 360.0;
    return normalized < 0.0 ? normalized + 360.0 : normalized;
}

function quaternionToHeadingDegrees(orientation) {
    if (!orientation) return null;
    const values = [orientation.x, orientation.y, orientation.z, orientation.w]
        .map(Number);
    if (!values.every(Number.isFinite)) return null;
    const norm = Math.hypot(...values);
    if (norm < 1e-9) return null;
    const [x, y, z, w] = values.map(value => value / norm);
    const yaw = Math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    );
    // ROS yaw=0 points +x; the local map conversion uses +x=east,+y=north.
    return normalizeHeadingDegrees(90.0 - yaw * 180.0 / Math.PI);
}

function updateRobotHeading(orientation) {
    const heading = quaternionToHeadingDegrees(orientation);
    state.robotHeading = heading;
    const markerElement = state.robotMarker && state.robotMarker.getElement
        ? state.robotMarker.getElement() : null;
    const headingElement = markerElement
        ? markerElement.querySelector('.robot-heading-icon') : null;
    if (!headingElement) return heading;

    const arrow = headingElement.querySelector('.robot-heading-arrow');
    if (heading === null) {
        headingElement.style.transform = 'rotate(0deg)';
        headingElement.title = '车头方向未知';
        if (arrow) arrow.style.opacity = '0';
        return heading;
    }
    headingElement.style.transform = `rotate(${heading}deg)`;
    headingElement.title = `车头方向 ${heading.toFixed(0)}°`;
    if (arrow) arrow.style.opacity = '1';
    return heading;
}

function updateRealRtkStatus() {
    const el = document.getElementById('rtk-status');
    if (!el) return;
    const status = state.rtkStatus;
    const simulated = status && (
        status.source === 'simulation' || status.simulated === true);
    const label = simulated ? '仿真RTK天线' : '真实RTK天线';
    if (!status) {
        el.textContent = state.realRtkFix
            ? `📡 ${label}: 已收到坐标，等待状态`
            : '📡 RTK天线: 未连接';
        return;
    }
    const solution = status.solution || status.state || 'UNKNOWN';
    const ntrip = status.ntrip ? ` / ${status.ntrip}` : '';
    const trusted = status.global_position_trusted ? ' / 已信任' : '';
    const coordinates = state.realRtkFix
        ? ` (${state.realRtkFix.lat.toFixed(6)},${state.realRtkFix.lng.toFixed(6)})`
        : '';
    el.textContent = `📡 ${label}: ${solution}${ntrip}${trusted}${coordinates}`;
    if (['NO_FIX', 'SERIAL_UNAVAILABLE', 'INVALID'].includes(solution)) {
        state.antennaMarker.setOpacity(0.0);
    } else if (state.realRtkFix) {
        state.antennaMarker.setOpacity(1.0);
    }
}

function publishCaptureCommand(action, extra = {}) {
    if (!state.connected || !state.captureCommandTopic) {
        showToast('❌ 尚未连接仿真采集节点', 'error');
        return false;
    }
    state.captureCommandTopic.publish(new ROSLIB.Message({
        data: JSON.stringify({ action, ...extra }),
    }));
    return true;
}

function captureStart() {
    const type = document.getElementById('capture-type').value;
    const id = document.getElementById('capture-id').value.trim();
    if (publishCaptureCommand('start', { type, id: id || undefined })) {
        showToast(`▶ 已开始采集${captureTypeLabel(type)}`, 'info');
    }
}

function captureFinish() {
    if (publishCaptureCommand('finish')) {
        showToast('⏹ 已完成轨迹采集，请检查几何后确认', 'info');
    }
}

function captureUndo() {
    if (publishCaptureCommand('undo')) showToast('↩ 已撤销最后一个采样点', 'info');
}

function captureSaveDraft() {
    if (publishCaptureCommand('save_draft')) showToast('📝 已保存为草稿', 'success');
}

function suggestRetryId(objectId) {
    const mission = state.mission || {};
    const known = new Set([
        ...(mission.objects || []),
        ...(mission.drafts || []),
    ].map(item => item && item.id).filter(Boolean));
    const base = `${objectId || 'capture'}-retry`;
    let candidate = base;
    let suffix = 2;
    while (known.has(candidate)) candidate = `${base}-${suffix++}`;
    return candidate;
}

function captureRetry() {
    const active = state.captureState && state.captureState.active;
    if (!active) return;
    stopTeleop();
    const nextId = suggestRetryId(active.id);
    if (publishCaptureCommand('save_draft')) {
        const idInput = document.getElementById('capture-id');
        idInput.value = nextId;
        showToast(
            `📝 当前失败轨迹已保留为草稿，请点击“开始采集”创建 ${nextId}`,
            'success',
        );
    }
}

function captureManualCorrect() {
    const active = state.captureState && state.captureState.active;
    if (!active || active.state !== 'draft' || active.type === 'corridor') return;
    if (!L.Draw || !L.Draw.Polygon) {
        showToast('❌ 当前地图不支持手动修正', 'error');
        return;
    }
    state.captureCorrectionMode = true;
    state.captureCorrectionDraw = new L.Draw.Polygon(map, {
        allowIntersection: false,
        showArea: true,
        shapeOptions: {
            color: '#ffcc00',
            weight: 3,
            dashArray: '6,4',
        },
    });
    state.captureCorrectionDraw.enable();
    showToast('✎ 请在地图上重新画一个不自交的闭合边界，双击结束', 'info');
    updateCaptureUI();
}

function cancelCaptureManualCorrect(showMessage = true) {
    if (state.captureCorrectionDraw) state.captureCorrectionDraw.disable();
    state.captureCorrectionDraw = null;
    state.captureCorrectionMode = false;
    if (showMessage) showToast('已取消手动修正', 'info');
    updateCaptureUI();
}

function handleCaptureCorrectionLayer(layer) {
    const active = state.captureState && state.captureState.active;
    const latlngs = layer.getLatLngs && layer.getLatLngs()[0];
    if (map.hasLayer(layer)) map.removeLayer(layer);
    state.captureCorrectionDraw = null;
    state.captureCorrectionMode = false;
    if (!active || !Array.isArray(latlngs) || latlngs.length < 3) {
        updateCaptureUI();
        return;
    }

    const geometry = latlngs.map(point => approxGpsToLocal(point.lat, point.lng));
    if (publishCaptureCommand('manual_geometry', { geometry })) {
        showToast('✎ 手动几何已提交，请检查后点击“确认对象”', 'info');
    }
    updateCaptureUI();
}

function captureCancel() {
    stopTeleop();
    const active = state.captureState && state.captureState.active;
    if (!active) {
        const drafts = state.mission && Array.isArray(state.mission.drafts)
            ? state.mission.drafts.length : 0;
        showToast(
            drafts
                ? '当前没有正在采集的对象，已保存草稿仍保留'
                : '当前没有正在采集的对象',
            'info',
        );
        return;
    }
    if (publishCaptureCommand('cancel')) showToast('✖ 已取消当前采集', 'info');
}

function captureConfirm() {
    const active = state.captureState && state.captureState.active;
    const extra = {};
    if (active && active.type === 'corridor') {
        const width = Number(document.getElementById('corridor-width').value);
        const from = document.getElementById('corridor-from').value.trim();
        const to = document.getElementById('corridor-to').value.trim();
        if (!Number.isFinite(width) || width <= 0 || !from || !to || from === to) {
            showToast('❌ 请填写有效的通道宽度和两个不同作业区', 'error');
            return;
        }
        extra.corridor = {
            width,
            from_work_area_id: from,
            to_work_area_id: to,
            bidirectional: document.getElementById('corridor-bidirectional').checked,
        };
    }
    if (publishCaptureCommand('confirm', extra)) {
        stopTeleop();
        showToast('✅ 对象已确认并加入任务', 'success');
    }
}

function captureTypeLabel(type) {
    return ({ work_area: '作业区', no_go_zone: '禁区', corridor: '通道' })[type] || type;
}

function formatCapturePoint(point) {
    if (!point || !Number.isFinite(Number(point.x))
        || !Number.isFinite(Number(point.y))) return '--';
    return `(${Number(point.x).toFixed(2)}, ${Number(point.y).toFixed(2)})m`;
}

function findSelfIntersectionPoint(active) {
    const issues = active && Array.isArray(active.issues) ? active.issues : [];
    const number = '[-+]?(?:\\d+(?:\\.\\d*)?|\\.\\d+)(?:[eE][-+]?\\d+)?';
    const pattern = new RegExp(
        `self-intersection\\s*\\[\\s*(${number})\\s+(${number})\\s*\\]`,
        'i',
    );
    for (const issue of issues) {
        const match = String(issue).match(pattern);
        if (match) return { x: Number(match[1]), y: Number(match[2]) };
    }
    return null;
}

function renderCaptureClosureFeedback(active) {
    const element = document.getElementById('capture-closure-feedback');
    if (!element) return;
    if (!active) {
        element.textContent = '起点 -- · 终点 -- · 闭合距离 -- · 闭合阈值 --';
        return;
    }
    const distance = Number(active.closure_distance);
    const tolerance = Number(active.closure_tolerance);
    const distanceText = Number.isFinite(distance) ? `${distance.toFixed(2)}m` : '--';
    const toleranceText = Number.isFinite(tolerance) ? `${tolerance.toFixed(2)}m` : '--';
    element.textContent = [
        `起点 ${formatCapturePoint(active.start_point)}`,
        `终点 ${formatCapturePoint(active.end_point)}`,
        `闭合距离 ${distanceText}`,
        `闭合阈值 ${toleranceText}`,
    ].join(' · ');
}

function updateCaptureUI() {
    const active = state.captureState && state.captureState.active;
    const captureState = state.captureState || {};
    const activeState = captureState.state || 'idle';
    const hasActive = !!active;
    const setDisabled = (id, disabled) => {
        const el = document.getElementById(id);
        if (el) el.disabled = disabled;
    };
    const canFinish = activeState === 'capturing' || activeState === 'draft';
    setDisabled('btn-capture-start', !state.connected || hasActive);
    setDisabled('btn-capture-finish', !hasActive || !canFinish);
    setDisabled('btn-capture-undo', !hasActive || !(active.raw_trajectory || []).length);
    setDisabled('btn-capture-draft', !hasActive);
    setDisabled(
        'btn-capture-confirm',
        !hasActive || activeState !== 'ready'
            || captureState.health_state !== 'GREEN');
    // 保持可点击，让保存草稿后再次点“取消”也能得到明确反馈。
    setDisabled('btn-capture-cancel', !state.connected);
    setDisabled(
        'btn-capture-load-plan',
        !state.connected || hasActive || !(state.mission.objects || []).length);

    const typeSelect = document.getElementById('capture-type');
    typeSelect.disabled = hasActive;
    document.getElementById('capture-id').disabled = hasActive;
    const activeType = hasActive ? active.type : typeSelect.value;
    document.getElementById('corridor-fields').hidden = activeType !== 'corridor';

    const stateText = hasActive
        ? `${captureTypeLabel(active.type)} · ${activeState} · ${(active.raw_trajectory || []).length} 点`
        : '未开始采集';
    const issues = hasActive && Array.isArray(active.issues)
        ? active.issues.map(issue => String(issue)) : [];
    const hasClosureMetrics = active
        && Number.isFinite(Number(active.closure_distance))
        && Number.isFinite(Number(active.closure_tolerance));
    const closureIssue = hasClosureMetrics
        ? Number(active.closure_distance) > Number(active.closure_tolerance)
        : issues.some(issue => issue.includes('not close to its start'));
    const invalidPolygon = issues.some(issue =>
        issue.includes('does not form a valid polygon'));
    const selfIntersection = issues.some(issue =>
        /self-intersection/i.test(issue));
    const intersectionPoint = selfIntersection
        ? findSelfIntersectionPoint(active) : null;
    const canRecover = hasActive
        && activeState === 'draft'
        && invalidPolygon;
    const geometryHint = closureIssue
        ? '⚠️ 轨迹未闭合，请回到起点附近后继续采集，再次点击完成'
        : invalidPolygon
            ? selfIntersection
                ? '⚠️ 轨迹存在自交，请沿边界单向绕行后再次点击完成'
                : '⚠️ 轨迹不构成有效多边形，请检查路线后再次点击完成'
            : activeState === 'ready'
                ? '✅ 已闭合，请点击“确认对象”后载入规划'
                : '';
    const health = captureState.health_state ? ` · 定位 ${captureState.health_state}` : '';
    const draftCount = Array.isArray(captureState.drafts)
        ? captureState.drafts.length : 0;
    const draftHint = !hasActive && draftCount
        ? ` · 已保留 ${draftCount} 个草稿` : '';
    const lastError = typeof captureState.last_error === 'string'
        ? captureState.last_error.trim() : '';
    const errorHint = lastError ? ` · ❌ ${lastError}` : '';
    const movementHint = captureState.drive_allowed
        ? '🟢 移动可用' : '🔴 移动锁定';
    const samplingHint = captureState.sampling_allowed
        ? '🔴 正在记录采样' : '⚪ 未记录采样';
    document.getElementById('capture-state-text').textContent =
        [
            stateText,
            movementHint,
            samplingHint,
            geometryHint,
            intersectionPoint ? `自交点 ${formatCapturePoint(intersectionPoint)}` : '',
            health,
            draftHint,
            errorHint,
        ]
            .filter(Boolean).join(' · ');
    renderCaptureClosureFeedback(active);

    const recovery = document.getElementById('capture-recovery');
    const recoveryMessage = document.getElementById('capture-recovery-message');
    const retryButton = document.getElementById('btn-capture-retry');
    const manualButton = document.getElementById('btn-capture-manual-correct');
    const correctionCancelButton = document.getElementById(
        'btn-capture-correction-cancel');
    const showRecovery = canRecover || state.captureCorrectionMode;
    recovery.hidden = !showRecovery;
    if (state.captureCorrectionMode) {
        recoveryMessage.textContent = '请在地图上画出新的不自交边界';
    } else if (canRecover) {
        recoveryMessage.textContent = intersectionPoint
            ? `检测到自交，交点 ${formatCapturePoint(intersectionPoint)}；当前仍为草稿`
            : '检测到无效多边形；当前仍为草稿';
    } else {
        recoveryMessage.textContent = '';
    }
    setDisabled(
        'btn-capture-retry',
        !canRecover || state.captureCorrectionMode,
    );
    setDisabled(
        'btn-capture-manual-correct',
        !canRecover || state.captureCorrectionMode
            || (active && active.type === 'corridor'),
    );
    correctionCancelButton.hidden = !state.captureCorrectionMode;
    manualButton.hidden = state.captureCorrectionMode;
    retryButton.hidden = state.captureCorrectionMode;

    const teleopEnabled = !!captureState.drive_allowed && state.connected;
    const joystick = document.getElementById('teleop-joystick');
    const stopButton = document.getElementById('teleop-stop');
    joystick.classList.toggle('disabled', !teleopEnabled);
    joystick.setAttribute('aria-disabled', String(!teleopEnabled));
    stopButton.disabled = !state.connected;
    if (!teleopEnabled && joystickPointerId !== null) stopTeleop();
    renderMissionList();
}

function deleteMissionObject(item) {
    if (!item || !item.id) return;
    if (state.captureState && state.captureState.active) {
        showToast('❌ 请先完成、保存或取消当前采集，再删除任务对象', 'error');
        return;
    }
    if (state.planningLoaded || ['executing', 'paused'].includes(state.execMode)) {
        showToast('❌ 请先停止并清除已载入规划，再删除任务对象', 'error');
        return;
    }
    const label = `${captureTypeLabel(item.type)} ${item.id}`;
    const kind = item._draft ? '草稿' : '已确认对象';
    if (!window.confirm(`确定删除${kind}“${label}”吗？此操作会同步删除持久化任务记录。`)) {
        return;
    }
    if (publishCaptureCommand('delete', { id: item.id })) {
        showToast(`🗑️ 已请求删除${label}`, 'info');
    }
}

function renderMissionList() {
    const container = document.getElementById('mission-object-list');
    if (!container) return;
    const objects = (state.mission.objects || []).map(item => ({ ...item, _draft: false }));
    const drafts = (state.mission.drafts || []).map(item => ({ ...item, _draft: true }));
    const all = objects.concat(drafts);
    const active = !!(state.captureState && state.captureState.active);
    const missionMutationBlocked = state.planningLoaded
        || ['executing', 'paused'].includes(state.execMode);
    const signature = JSON.stringify({
        items: all.map(item => [item.id, item.type, item._draft]),
        connected: state.connected,
        active,
        missionMutationBlocked,
    });
    if (container.dataset.renderSignature === signature) return;
    container.dataset.renderSignature = signature;
    if (!all.length) {
        container.textContent = '暂无已确认对象或草稿';
        return;
    }
    container.replaceChildren();
    all.forEach(item => {
        const entry = document.createElement('span');
        entry.className = 'mission-object-entry';

        const label = document.createElement('span');
        label.textContent = `${item._draft ? '📝' : '✅'}${captureTypeLabel(item.type)}:${item.id}`;
        entry.appendChild(label);

        const deleteButton = document.createElement('button');
        deleteButton.type = 'button';
        deleteButton.className = 'btn btn-danger mission-object-delete';
        deleteButton.textContent = '删除';
        deleteButton.disabled = !state.connected || active || missionMutationBlocked;
        deleteButton.setAttribute('aria-label', `删除${captureTypeLabel(item.type)} ${item.id}`);
        deleteButton.addEventListener('click', () => deleteMissionObject(item));
        entry.appendChild(deleteButton);
        container.appendChild(entry);
    });
}

function renderMissionItems() {
    if (!state.missionItems) return;
    state.missionItems.clearLayers();
    const objects = (state.mission.objects || []).map(item => ({ ...item, _draft: false }));
    const drafts = (state.mission.drafts || []).map(item => ({ ...item, _draft: true }));
    const allItems = objects.concat(drafts);
    const signature = JSON.stringify(allItems.map(item => ({
        id: item.id,
        type: item.type,
        status: item.status,
        draft: item._draft,
        geometry: item.geometry || [],
        raw_trajectory: (item.geometry || []).length
            ? [] : (item.raw_trajectory || []),
    })));
    let hasRenderableGeometry = false;
    allItems.forEach(item => {
        const geometry = item.geometry || [];
        const rawTrajectory = item.raw_trajectory || [];
        const rawPoints = rawTrajectory
            .filter(point => Number.isFinite(Number(point.x))
                && Number.isFinite(Number(point.y)))
            .map(point => [Number(point.x), Number(point.y)]);
        const points = geometry.length >= 2 ? geometry : rawPoints;
        if (points.length < 2) return;
        const latlngs = points.map(point => approxLocalToGPS(point[0], point[1]));
        if (!latlngs.every(([lat, lng]) => Number.isFinite(lat) && Number.isFinite(lng))) return;
        const color = item._draft ? '#aaaaaa' : (
            item.type === 'no_go_zone' ? '#ff2222' :
            item.type === 'corridor' ? '#ff9900' : '#00ddff'
        );
        const layer = geometry.length < 2 || item.type === 'corridor'
            ? L.polyline(latlngs, {
                color,
                weight: 4,
                dashArray: item._draft ? '5,5' : null,
                pane: 'captureRoutePane',
            })
            : L.polygon(latlngs, {
                color,
                fillColor: color,
                fillOpacity: item._draft ? 0.08 : 0.2,
                dashArray: item._draft ? '5,5' : null,
                pane: 'captureRoutePane',
            });
        const label = geometry.length < 2 ? '原始轨迹草稿' : (
            item._draft ? '草稿' : '已确认');
        layer.bindTooltip(`${label} · ${captureTypeLabel(item.type)} · ${item.id}`);
        state.missionItems.addLayer(layer);
        hasRenderableGeometry = true;

        const intersectionPoint = findSelfIntersectionPoint(item);
        if (intersectionPoint) {
            const [intersectionLat, intersectionLng] = approxLocalToGPS(
                intersectionPoint.x,
                intersectionPoint.y,
            );
            const marker = L.circleMarker(
                [intersectionLat, intersectionLng],
                {
                    radius: 9,
                    color: '#fff',
                    weight: 2,
                    fillColor: '#ff2244',
                    fillOpacity: 1.0,
                    pane: 'captureRoutePane',
                },
            );
            marker.bindTooltip(
                `⚠️ 自交点 · ${formatCapturePoint(intersectionPoint)}`,
            );
            state.missionItems.addLayer(marker);
        }
    });
    if (hasRenderableGeometry && signature !== state._missionGeometrySignature) {
        const bounds = state.missionItems.getBounds();
        if (bounds.isValid()) map.fitBounds(bounds.pad(0.25), { maxZoom: 19 });
    }
    state._missionGeometrySignature = signature;
}

function renderCapturePreview() {
    if (!state.capturePreviewItems) return;
    state.capturePreviewItems.clearLayers();
    const active = state.captureState && state.captureState.active;
    if (!active) {
        state._capturePreviewSignature = null;
        return;
    }

    const geometry = active.geometry || [];
    const rawTrajectory = active.raw_trajectory || [];
    const rawPoints = rawTrajectory
        .filter(point => Number.isFinite(Number(point.x))
            && Number.isFinite(Number(point.y)))
        .map(point => [Number(point.x), Number(point.y)]);
    const points = geometry.length >= 2 ? geometry : rawPoints;
    if (points.length < 2) return;

    const latlngs = points.map(point => approxLocalToGPS(point[0], point[1]));
    if (!latlngs.every(([lat, lng]) => Number.isFinite(lat) && Number.isFinite(lng))) return;
    const signature = JSON.stringify({
        id: active.id,
        state: active.state,
        geometry,
        raw_trajectory: geometry.length >= 2 ? [] : rawTrajectory,
        start_point: active.start_point,
        end_point: active.end_point,
        closure_distance: active.closure_distance,
        closure_tolerance: active.closure_tolerance,
    });
    const color = active.type === 'no_go_zone' ? '#ff2222' : (
        active.type === 'corridor' ? '#ff9900' : '#00ff99');
    const layer = geometry.length < 2 || active.type === 'corridor'
        ? L.polyline(latlngs, {
            color,
            weight: 4,
            dashArray: '8,5',
            pane: 'captureRoutePane',
        })
        : L.polygon(latlngs, {
            color,
            fillColor: color,
            fillOpacity: 0.12,
            weight: 3,
            dashArray: '8,5',
            pane: 'captureRoutePane',
        });
    layer.bindTooltip(
        `预览 · ${captureTypeLabel(active.type)} · ${active.id} · ${active.state}`,
    );
    state.capturePreviewItems.addLayer(layer);
    [
        { point: active.start_point, label: '起点', color: '#00ff99' },
        { point: active.end_point, label: '终点', color: '#ffcc00' },
    ].forEach(({ point, label, color }) => {
        if (!point || !Number.isFinite(Number(point.x))
            || !Number.isFinite(Number(point.y))) return;
        const [lat, lng] = approxLocalToGPS(Number(point.x), Number(point.y));
        const marker = L.circleMarker([lat, lng], {
            radius: 5,
            color: '#fff',
            weight: 2,
            fillColor: color,
            fillOpacity: 1.0,
            pane: 'captureRoutePane',
        });
        marker.bindTooltip(`${label} · ${formatCapturePoint(point)}`);
        state.capturePreviewItems.addLayer(marker);
    });
    const intersectionPoint = findSelfIntersectionPoint(active);
    if (intersectionPoint) {
        const [lat, lng] = approxLocalToGPS(
            intersectionPoint.x,
            intersectionPoint.y,
        );
        const marker = L.circleMarker([lat, lng], {
            radius: 9,
            color: '#fff',
            weight: 2,
            fillColor: '#ff2244',
            fillOpacity: 1.0,
            pane: 'captureRoutePane',
        });
        marker.bindTooltip(`⚠️ 自交点 · ${formatCapturePoint(intersectionPoint)}`);
        state.capturePreviewItems.addLayer(marker);
    }
    if (active.state === 'ready'
            && signature !== state._capturePreviewSignature) {
        const bounds = state.capturePreviewItems.getBounds();
        if (bounds.isValid()) map.fitBounds(bounds.pad(0.25), { maxZoom: 19 });
    }
    state._capturePreviewSignature = signature;
}

/**
 * 将局部坐标 (x, y) 近似转换为 GPS 坐标
 * 使用 GPS 原点 (22.5431, 114.0579) 做等距圆柱投影反算
 */
const GPS_ORIGIN_LAT = 22.5431;
const GPS_ORIGIN_LNG = 114.0579;
const DEG_TO_RAD = Math.PI / 180.0;
const GPS_ORIGIN_LAT_RAD = GPS_ORIGIN_LAT * DEG_TO_RAD;

function approxLocalToGPS(x, y) {
    // 从前端发过去的 GPS 坐标 → 后端用 _gps_to_local 转成局部坐标
    // 这里反算回来: lng = x / (111320 * cos(lat_rad)) + origin_lng
    //              lat = y / 110540 + origin_lat
    const lng = x / (111320.0 * Math.cos(GPS_ORIGIN_LAT_RAD)) + GPS_ORIGIN_LNG;
    const lat = y / 110540.0 + GPS_ORIGIN_LAT;
    return [lat, lng];
}

function approxGpsToLocal(lat, lng) {
    const x = (lng - GPS_ORIGIN_LNG)
        * 111320.0 * Math.cos(GPS_ORIGIN_LAT_RAD);
    const y = (lat - GPS_ORIGIN_LAT) * 110540.0;
    return [x, y];
}

// =============================================================
// 绘图模式切换（区域 / 障碍物）
// =============================================================
function setDrawMode(mode) {
    state.drawMode = mode;
    const btnArea = document.getElementById('btn-mode-area');
    const btnObs = document.getElementById('btn-mode-obstacle');
    btnArea.className = 'btn btn-sm' + (mode === 'area' ? ' btn-active' : '');
    btnObs.className = 'btn btn-sm' + (mode === 'obstacle' ? ' btn-obstacle-active' : '');
    showToast(mode === 'area' ? '📍 区域绘制模式' : '⛔ 障碍物绘制模式（红色虚线）', 'info');
}

// =============================================================
// 暂停 / 继续 / 停止 — 调用 ROS2 服务
// =============================================================
function triggerPause() {
    if (!state.connected) { showToast('❌ 未连接到 ROS2', 'error'); return; }
    const svc = new ROSLIB.Service({
        ros: state.ros,
        name: '/multi_area/pause',
        serviceType: 'std_srvs/srv/Trigger',
    });
    svc.callService(new ROSLIB.ServiceRequest({}), result => {
        if (result.success) {
            state.execMode = 'paused';
            document.getElementById('mode-display').textContent = '⏸️ 已暂停';
            updateExecButtons();
            showToast('⏸️ 已暂停执行', 'info');
        } else {
            showToast('⏸️ 暂停失败: ' + (result.message || ''), 'error');
        }
    }, error => showToast('❌ 暂停服务调用失败', 'error'));
}

function triggerResume() {
    if (!state.connected) { showToast('❌ 未连接到 ROS2', 'error'); return; }
    const svc = new ROSLIB.Service({
        ros: state.ros,
        name: '/multi_area/resume',
        serviceType: 'std_srvs/srv/Trigger',
    });
    svc.callService(new ROSLIB.ServiceRequest({}), result => {
        if (result.success) {
            state.execMode = 'executing';
            document.getElementById('mode-display').textContent = '▶️ 执行中';
            updateExecButtons();
            showToast('▶️ 已恢复执行', 'success');
        } else {
            showToast('▶️ 恢复失败: ' + (result.message || ''), 'error');
        }
    }, error => showToast('❌ 恢复服务调用失败', 'error'));
}

function triggerStop() {
    if (!state.connected) { showToast('❌ 未连接到 ROS2', 'error'); return; }
    const svc = new ROSLIB.Service({
        ros: state.ros,
        name: '/multi_area/stop',
        serviceType: 'std_srvs/srv/Trigger',
    });
    svc.callService(new ROSLIB.ServiceRequest({}), result => {
        if (result.success) {
            state.execMode = 'stopped';
            document.getElementById('mode-display').textContent = '⏹️ 已停止';
            updateExecButtons();
            showToast('⏹️ 已停止执行', 'info');
        } else {
            showToast('⏹️ 停止失败: ' + (result.message || ''), 'error');
        }
    }, error => showToast('❌ 停止服务调用失败: ' + error, 'error'));
}

// =============================================================
// 发送区域到 ROS2（含障碍物）
// =============================================================
function publishPlannerSelection(selection) {
    if (!state.connected) return false;
    const topic = new ROSLIB.Topic({
        ros: state.ros,
        name: '/web/mission',
        messageType: 'std_msgs/String',
    });
    topic.publish(new ROSLIB.Message({
        data: JSON.stringify(selection),
    }));
    return true;
}

function publishPlannerAreas(areaData, description) {
    if (!state.connected || !areaData.length) return false;

    publishPlannerSelection({ action: 'use_legacy_areas' });
    const areaTopic = new ROSLIB.Topic({
        ros: state.ros,
        name: '/web/areas',
        messageType: 'std_msgs/String',
    });
    areaTopic.publish(new ROSLIB.Message({
        data: JSON.stringify({ action: 'set_areas', areas: areaData }),
    }));

    state.planningLoaded = true;
    document.getElementById('btn-plan').disabled = false;
    document.getElementById('btn-execute').disabled = true;
    document.getElementById('mode-display').textContent = '⚙️ 已载入规划区域';
    showToast(`📤 ${description}`, 'success');
    return true;
}

function loadCaptureMissionForPlanning() {
    const mission = state.mission || {};
    const objects = Array.isArray(mission.objects) ? mission.objects : [];
    const drafts = Array.isArray(mission.drafts) ? mission.drafts : [];
    if (!objects.length) {
        showToast('❌ 没有已确认的采集对象', 'error');
        return;
    }
    if (drafts.length) {
        showToast('❌ 仍有草稿对象，请先确认或取消草稿', 'error');
        return;
    }
    const workAreas = objects.filter(item => item.type === 'work_area');
    const corridors = objects.filter(item => item.type === 'corridor');
    if (!workAreas.length) {
        showToast('❌ 任务至少需要一个已确认作业区', 'error');
        return;
    }
    const order = Array.isArray(mission.order) ? mission.order : [];
    const executableIds = new Set([
        ...workAreas.map(item => item.id),
        ...corridors.map(item => item.id),
    ]);
    if (order.length !== executableIds.size
        || new Set(order).size !== executableIds.size
        || order.some(id => !executableIds.has(id))) {
        showToast('❌ 采集任务顺序不完整，不能载入规划', 'error');
        return;
    }

    try {
        if (!publishPlannerSelection({
            action: 'set_mission',
            mission,
        })) return;
        state.planningLoaded = true;
        document.getElementById('btn-plan').disabled = false;
        document.getElementById('btn-execute').disabled = true;
        document.getElementById('mode-display').textContent = '⚙️ 已载入规划任务';
        showToast(
            `📤 已载入 ${workAreas.length} 个作业区、`
            + `${corridors.length} 个通道和 `
            + `${objects.filter(item => item.type === 'no_go_zone').length} 个禁区；现在可以规划`,
            'success',
        );
    } catch (error) {
        showToast(`❌ 载入规划失败: ${error.message}`, 'error');
    }
}

function sendAreasToROS() {
    if (!state.connected || (state.areas.length === 0 && state.obstacles.length === 0)) return;

    // ★ 新方案：将区域和障碍物分开发送，由后端定义器用 Shapely 判断归属
    const areaData = [
        // 所有割草区域（不带 inner_rings）
        ...state.areas.map(area => ({
            name: area.name,
            points: area.latlngs.map(([lat, lng]) => ({ x: lng, y: lat, z: 0.0 })),
            cutting_angle: 0.0,
            max_speed: 1.0,
        })),
        // 所有障碍物（单独发送，max_speed=0）
        ...state.obstacles.map(obs => ({
            name: obs.name,
            points: obs.latlngs.map(([lat, lng]) => ({ x: lng, y: lat, z: 0.0 })),
            cutting_angle: 0.0,
            max_speed: 0.0,
            is_obstacle: true,
        })),
    ];

    console.log('[发送] areaData:', JSON.stringify(areaData, null, 2));

    publishPlannerAreas(
        areaData,
        `已发送 ${state.areas.length} 个区域 + ${state.obstacles.length} 个障碍物；现在可以规划`,
    );
}

/** 射线法判断点是否在多边形内 */
function pointInPolygon(point, polygon) {
    const [x, y] = point;
    let inside = false;
    for (let i = 0, j = polygon.length - 1; i < polygon.length; j = i++) {
        const [xi, yi] = polygon[i];
        const [xj, yj] = polygon[j];
        if ((yi > y) !== (yj > y) && x < (xj - xi) * (y - yi) / (yj - yi) + xi) inside = !inside;
    }
    return inside;
}

/** 检查一个多边形是否完全在另一个多边形内部 */
function isPolygonInside(innerLatLngs, outerLatLngs) {
    if (!innerLatLngs.length || !outerLatLngs.length) return false;
    for (const ll of innerLatLngs) {
        if (!pointInPolygon(ll, outerLatLngs)) return false;
    }
    return true;
}

function triggerPlan() {
    if (!state.connected) {
        showToast('❌ 未连接到 ROS2', 'error');
        return;
    }
    if (!state.planningLoaded) {
        showToast('❌ 请先点击“载入规划”或“发送”', 'error');
        return;
    }
    const btn = document.getElementById('btn-plan');
    btn.disabled = true;
    btn.textContent = '⏳ 规划中...';
    const svc = new ROSLIB.Service({
        ros: state.ros,
        name: '/multi_area/plan',
        serviceType: 'std_srvs/srv/Trigger',
    });
    svc.callService(new ROSLIB.ServiceRequest({}), result => {
        btn.disabled = false;
        btn.textContent = '📋 规划路径';
        if (result.success) {
            document.getElementById('btn-execute').disabled = false;
            document.getElementById('mode-display').textContent = '📋 已规划';
            showToast('✅ 路径规划完成！黄色虚线为覆盖路径', 'success');
        } else {
            showToast('❌ 规划失败: ' + (result.message || '未知错误'), 'error');
        }
    }, error => {
        btn.disabled = false;
        btn.textContent = '📋 规划路径';
        showToast('❌ 规划服务调用失败: ' + error, 'error');
    });
}

function triggerExecute() {
    if (!state.connected) {
        showToast('❌ 未连接到 ROS2', 'error');
        return;
    }
    const btn = document.getElementById('btn-execute');
    btn.disabled = true;
    btn.textContent = '⏳ 执行中...';
    const svc = new ROSLIB.Service({
        ros: state.ros,
        name: '/multi_area/plan_and_start',
        serviceType: 'std_srvs/srv/Trigger',
    });
    svc.callService(new ROSLIB.ServiceRequest({}), result => {
        btn.disabled = false;
        btn.textContent = '▶️ 开始执行';
        if (result.success) {
            state.execMode = 'executing';
            document.getElementById('mode-display').textContent = '▶️ 执行中';
            updateExecButtons();
            showToast('✅ 割草机开始执行！', 'success');
        } else {
            showToast('❌ 执行失败: ' + (result.message || '未知错误'), 'error');
        }
    }, error => {
        btn.disabled = false;
        btn.textContent = '▶️ 开始执行';
        showToast('❌ 执行服务调用失败: ' + error, 'error');
    });
}

// =============================================================
// Toast 提示
// =============================================================
function showToast(msg, type) {
    const container = document.getElementById('toast-container');
    if (!container) return;
    const el = document.createElement('div');
    el.className = 'toast toast-' + (type || 'info');
    el.textContent = msg;
    container.appendChild(el);
    // 3秒后自动消失
    setTimeout(() => {
        el.style.opacity = '0';
        setTimeout(() => el.remove(), 300);
    }, 3000);
}

// =============================================================
// UI 更新函数
// =============================================================
function updateAreaList() {
    const container = document.getElementById('area-list');
    // 列出所有区域 + 障碍物
    const items = [
        ...state.areas.map((a, i) => ({ ...a, _type: 'area', _idx: i })),
        ...state.obstacles.map((o, i) => ({ ...o, _type: 'obstacle', _idx: i })),
    ];
    container.innerHTML = items.map(item =>
        `<span class="area-tag" style="${item._type === 'obstacle' ? 'border:1px solid #f44;' : ''}">
            <span style="display:inline-block;width:8px;height:8px;border-radius:50%;background:${item.color};"></span>
            ${item._type === 'obstacle' ? '⛔ ' : ''}${item.name}
            <span class="remove" data-type="${item._type}" data-index="${item._idx}">×</span>
        </span>`
    ).join('');

    container.querySelectorAll('.remove').forEach(el => {
        el.addEventListener('click', function () {
            const type = this.dataset.type;
            const idx = parseInt(this.dataset.index);
            if (type === 'obstacle') removeObstacle(idx);
            else removeArea(idx);
        });
    });
}

function removeArea(index) {
    const area = state.areas[index];
    if (area && area.layer && drawnItems.hasLayer(area.layer)) {
        drawnItems.removeLayer(area.layer);
    }
    if (area && area.label && drawnItems.hasLayer(area.label)) {
        drawnItems.removeLayer(area.label);
    }
    syncAreasFromMap();
    updateAreaList();
    updateSendButton();
}

function removeObstacle(index) {
    const obs = state.obstacles[index];
    if (obs && obs.layer && drawnItems.hasLayer(obs.layer)) {
        drawnItems.removeLayer(obs.layer);
    }
    if (obs && obs.label && drawnItems.hasLayer(obs.label)) {
        drawnItems.removeLayer(obs.label);
    }
    syncAreasFromMap();
    updateAreaList();
    updateSendButton();
}

function updateSendButton() {
    const btn = document.getElementById('btn-send');
    const hasContent = state.areas.length > 0 || state.obstacles.length > 0;
    btn.disabled = !hasContent || !state.connected;
    let label = '📤 请先在地图画区域';
    if (hasContent) {
        const parts = [];
        if (state.areas.length > 0) parts.push(`${state.areas.length} 个区域`);
        if (state.obstacles.length > 0) parts.push(`${state.obstacles.length} 个障碍物`);
        label = `📤 发送 ${parts.join(' + ')} 到割草机`;
    }
    btn.textContent = label;
}

// =============================================================
// 按钮事件绑定
// =============================================================
let teleopInterval = null;
let joystickPointerId = null;
let joystickAxes = { x: 0, y: 0 };
let joystickCommand = { linear: 0, angular: 0 };
let teleopSpeed = Number(document.getElementById('teleop-speed').value) || 0.6;
const TELEOP_MAX_ANGULAR = 1.2;
const JOYSTICK_DEADZONE = 0.12;

function publishTeleop(linear, angular) {
    if (!state.connected || !state.teleopTopic) return;
    state.teleopTopic.publish(new ROSLIB.Message({
        linear: { x: linear, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: angular },
    }));
}

function publishJoystickCommand() {
    joystickCommand = {
        linear: -joystickAxes.y * teleopSpeed,
        angular: -joystickAxes.x * TELEOP_MAX_ANGULAR,
    };
    publishTeleop(joystickCommand.linear, joystickCommand.angular);
}

function resetJoystickVisual() {
    const joystick = document.getElementById('teleop-joystick');
    const stick = document.getElementById('teleop-stick');
    joystick.classList.remove('active');
    stick.style.transform = 'translate(-50%, -50%)';
}

function stopTeleop() {
    if (teleopInterval) {
        clearInterval(teleopInterval);
        teleopInterval = null;
    }
    joystickPointerId = null;
    joystickAxes = { x: 0, y: 0 };
    joystickCommand = { linear: 0, angular: 0 };
    resetJoystickVisual();
    publishTeleop(0, 0);
}

function updateJoystickFromEvent(event) {
    const joystick = document.getElementById('teleop-joystick');
    const stick = document.getElementById('teleop-stick');
    const rect = joystick.getBoundingClientRect();
    const centerX = rect.left + rect.width / 2;
    const centerY = rect.top + rect.height / 2;
    const radius = Math.max(1, Math.min(rect.width, rect.height) / 2
        - stick.offsetWidth / 2 - 4);
    let dx = event.clientX - centerX;
    let dy = event.clientY - centerY;
    const distance = Math.hypot(dx, dy);
    if (distance > radius) {
        const scale = radius / distance;
        dx *= scale;
        dy *= scale;
    }
    stick.style.transform = `translate(calc(-50% + ${dx}px), calc(-50% + ${dy}px))`;

    const rawX = dx / radius;
    const rawY = dy / radius;
    const rawMagnitude = Math.hypot(rawX, rawY);
    if (rawMagnitude <= JOYSTICK_DEADZONE) {
        joystickAxes = { x: 0, y: 0 };
    } else {
        const activeMagnitude = (rawMagnitude - JOYSTICK_DEADZONE)
            / (1 - JOYSTICK_DEADZONE);
        const scale = activeMagnitude / rawMagnitude;
        joystickAxes = { x: rawX * scale, y: rawY * scale };
    }
    publishJoystickCommand();
}

function startJoystick(event) {
    event.preventDefault();
    event.stopPropagation();
    const joystick = event.currentTarget;
    if (joystick.classList.contains('disabled')
        || !state.captureState || !state.captureState.drive_allowed) return;
    stopTeleop();
    joystickPointerId = event.pointerId;
    joystick.classList.add('active');
    joystick.setPointerCapture(event.pointerId);
    updateJoystickFromEvent(event);
    teleopInterval = setInterval(publishJoystickCommand, 100);
}

function moveJoystick(event) {
    if (event.pointerId !== joystickPointerId) return;
    event.preventDefault();
    event.stopPropagation();
    updateJoystickFromEvent(event);
}

document.getElementById('btn-capture-start').addEventListener('click', captureStart);
document.getElementById('btn-capture-finish').addEventListener('click', captureFinish);
document.getElementById('btn-capture-undo').addEventListener('click', captureUndo);
document.getElementById('btn-capture-draft').addEventListener('click', captureSaveDraft);
document.getElementById('btn-capture-retry').addEventListener('click', captureRetry);
document.getElementById('btn-capture-manual-correct').addEventListener(
    'click', captureManualCorrect);
document.getElementById('btn-capture-correction-cancel').addEventListener(
    'click', () => cancelCaptureManualCorrect());
document.getElementById('btn-capture-confirm').addEventListener('click', captureConfirm);
document.getElementById('btn-capture-cancel').addEventListener('click', captureCancel);
document.getElementById('btn-capture-load-plan').addEventListener(
    'click', loadCaptureMissionForPlanning);
document.getElementById('capture-type').addEventListener('change', updateCaptureUI);
const joystick = document.getElementById('teleop-joystick');
joystick.addEventListener('pointerdown', startJoystick);
joystick.addEventListener('pointermove', moveJoystick);
joystick.addEventListener('pointerup', stopTeleop);
joystick.addEventListener('pointercancel', stopTeleop);
joystick.addEventListener('lostpointercapture', stopTeleop);
document.getElementById('teleop-stop').addEventListener('pointerdown', event => {
    event.preventDefault();
    event.stopPropagation();
    stopTeleop();
});
document.getElementById('teleop-speed').addEventListener('input', event => {
    teleopSpeed = Number(event.target.value);
    document.getElementById('teleop-speed-value').textContent =
        `${teleopSpeed.toFixed(2)} m/s`;
    if (joystickPointerId !== null) publishJoystickCommand();
});
window.addEventListener('blur', stopTeleop);
document.addEventListener('visibilitychange', () => {
    if (document.hidden) stopTeleop();
});

// 绘图模式切换
document.getElementById('btn-mode-area').addEventListener('click', () => setDrawMode('area'));
document.getElementById('btn-mode-obstacle').addEventListener('click', () => setDrawMode('obstacle'));

// 操作按钮
document.getElementById('btn-send').addEventListener('click', sendAreasToROS);
document.getElementById('btn-plan').addEventListener('click', function() {
    showToast('📋 正在规划全覆盖路径...', 'info');
    triggerPlan();
});
document.getElementById('btn-execute').addEventListener('click', function() {
    showToast('▶️ 正在开始执行...', 'info');
    triggerExecute();
});
document.getElementById('btn-pause').addEventListener('click', triggerPause);
document.getElementById('btn-resume').addEventListener('click', triggerResume);
document.getElementById('btn-stop').addEventListener('click', triggerStop);

document.getElementById('btn-clear').addEventListener('click', function () {
    cancelCaptureManualCorrect(false);
    stopTeleop();
    // 1. 调用后端清除服务
    if (state.connected) {
        // 1a. 清除执行器路径（停止执行 + 清空 waypoints）
        const clearSvc = new ROSLIB.Service({
            ros: state.ros,
            name: '/multi_area/clear',
            serviceType: 'std_srvs/srv/Trigger',
        });
        clearSvc.callService(new ROSLIB.ServiceRequest({}));

        // 1b. 清除规划器路径（停止 1Hz 重发）
        const clearPathSvc = new ROSLIB.Service({
            ros: state.ros,
            name: '/multi_area/clear_path',
            serviceType: 'std_srvs/srv/Trigger',
        });
        clearPathSvc.callService(new ROSLIB.ServiceRequest({}));

        // 1c. ★ 清除定义器的所有区域 + 删除 YAML 文件
        const clearAllSvc = new ROSLIB.Service({
            ros: state.ros,
            name: '/multi_area/clear_all',
            serviceType: 'std_srvs/srv/Trigger',
        });
        clearAllSvc.callService(new ROSLIB.ServiceRequest({}));
    }

    // 2. 清除地图上的图形和路径线
    drawnItems.clearLayers();
    state.pathLine.setLatLngs([]);   // ← 立即清除黄色虚线路径

    // 3. 重置状态（包括计数器，确保新区域从 区域1/障碍物1 开始）
    polygonIdCounter = 0;
    obstacleIdCounter = 0;
    state.areas = [];
    state.obstacles = [];
    state.execMode = 'idle';
    state.planningLoaded = false;
    state._missionGeometrySignature = null;
    updateAreaList();
    updateSendButton();
    document.getElementById('btn-plan').disabled = true;
    document.getElementById('btn-execute').disabled = true;
    document.getElementById('mode-display').textContent = '⚙️ 待机';
    updateExecButtons();
    showToast('🗑️ 已清除所有区域、路径和状态', 'info');
});

// =============================================================
// 更新暂停/继续/停止按钮状态（基于 execMode）
// =============================================================
function updateExecButtons() {
    const mode = state.execMode;
    document.getElementById('btn-pause').disabled = mode !== 'executing';
    document.getElementById('btn-resume').disabled = mode !== 'paused';
    document.getElementById('btn-stop').disabled = (mode !== 'executing' && mode !== 'paused');
}
// 初始禁用
updateExecButtons();
updateCaptureUI();

// =============================================================
// 启动
// =============================================================
// 延迟连接，让页面先加载完
setTimeout(connectROS, 1000);
