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
    rosbridgeUrl: 'ws://localhost:9090',
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
    robotMarker: null,   // Leaflet marker for robot
    pathLine: null,      // Leaflet polyline for coverage path
    coveragePercent: 0,
    robotMode: 'idle',
    execMode: 'idle',        // 'executing' | 'paused' | 'stopped' | 'idle'
    rosSubscribed: false, // 是否已订阅 ROS 话题（防重复订阅）
    _odomReceived: false, // 是否收到过 odom 数据
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
    html: `<div style="
        width:32px;height:32px;background:#ff2222;
        border:3px solid white;border-radius:50%;
        box-shadow:0 0 12px rgba(255,34,34,0.9), 0 0 24px rgba(255,34,34,0.4);
        display:flex;align-items:center;justify-content:center;
        font-size:16px;font-weight:bold;color:white;
        animation:pulse-robot 1.5s ease-in-out infinite;">M</div>`,
    iconSize: [32, 32],
    iconAnchor: [16, 16],
});
state.robotMarker = L.marker(CONFIG.initialCenter, {
    icon: robotIcon,
    zIndexOffset: 1000,
}).addTo(map);

// 路径图层（显示规划的全覆盖路径）
state.pathLine = L.polyline([], {
    color: '#ffcc00',
    weight: 3,
    opacity: 0.8,
    dashArray: '8, 6',
}).addTo(map);

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
        document.getElementById('btn-send').disabled = false;
        setupROSSubscribers();
    });

    state.ros.on('close', function () {
        state.connected = false;
        state.rosSubscribed = false;  // 下次重连允许重新订阅
        document.getElementById('ros-status').innerHTML =
            '<span class="status-dot disconnected"></span>未连接';
        document.getElementById('btn-send').disabled = true;
        document.getElementById('btn-plan').disabled = true;
        document.getElementById('btn-execute').disabled = true;
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
        const statusText = status === 2 ? '🛰️ RTK固定解' :
                          status === 1 ? '🛰️ RTK浮点解' :
                          status === 0 ? '🛰️ 单点定位' : '🛰️ 无定位';
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
        state._odomReceived = true;
        state.robotPose = { lat, lng };
        state.robotMarker.setLatLng([lat, lng]);
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

    const areaTopic = new ROSLIB.Topic({
        ros: state.ros,
        name: '/web/areas',
        messageType: 'std_msgs/String',
    });
    const msg = new ROSLIB.Message({
        data: JSON.stringify({ action: 'set_areas', areas: areaData }),
    });
    areaTopic.publish(msg);

    document.getElementById('btn-plan').disabled = false;
    showToast(`📤 已发送 ${state.areas.length} 个区域 + ${state.obstacles.length} 个障碍物`, 'success');
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

// =============================================================
// 启动
// =============================================================
// 延迟连接，让页面先加载完
setTimeout(connectROS, 1000);
