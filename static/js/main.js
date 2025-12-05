
// Global State
let isRecording = false;
let recordedData = [];
let currentTheme = 'dark';
const socket = io();

// Client-side Max Value Tracking
let maxValues = {
    velocity: 0,
    acceleration: 0
};

// DOM Elements
const els = {
    status: document.getElementById('connection-status'),
    themeBtn: document.getElementById('theme-toggle'),
    recordBtn: document.getElementById('record-btn'),
    exportBtn: document.getElementById('export-btn'),
    missionTime: document.getElementById('mission-time'),
    values: {
        altitude: document.getElementById('val-altitude'),
        maxAlt: document.getElementById('val-max-alt'),
        velocity: document.getElementById('val-velocity'),
        maxVel: document.getElementById('val-max-vel'),
        accel: document.getElementById('val-accel'),
        maxAccel: document.getElementById('val-max-accel'),
        temp: document.getElementById('val-temp'),
        time: document.getElementById('val-time'),
        pitch: document.getElementById('val-pitch'),
        yaw: document.getElementById('val-yaw'),
        roll: document.getElementById('val-roll'),
    },
    states: document.querySelectorAll('.state-node')
};

// Theme Toggle
els.themeBtn.addEventListener('click', () => {
    currentTheme = currentTheme === 'dark' ? 'light' : 'dark';
    document.documentElement.setAttribute('data-theme', currentTheme);
    const icon = els.themeBtn.querySelector('i');
    icon.className = currentTheme === 'dark' ? 'fa-solid fa-sun' : 'fa-solid fa-moon';

    // Update Chart Colors
    updateChartTheme(currentTheme);
});

// Socket Connection (Server Status)
socket.on('connect', () => {
    // We don't show "Connected" here anymore, we wait for serial_status
    console.log('Connected to UI Server');
});

socket.on('disconnect', () => {
    setConnectionStatus(false, 'Server Disconnected');
});

// Serial Status Handling
socket.on('serial_status', (status) => {
    if (status.connected) {
        setConnectionStatus(true, `Connected(${status.port})`);
    } else {
        setConnectionStatus(false, status.message || 'Searching for Device...');
    }
});

function setConnectionStatus(isConnected, text) {
    if (isConnected) {
        els.status.classList.add('connected');
    } else {
        els.status.classList.remove('connected');
    }
    els.status.querySelector('.text').textContent = text;
}

// Data Handling
socket.on('telemetry_data', (data) => {
    // Update Max Values Client-Side
    if (Math.abs(data.velocity) > maxValues.velocity) maxValues.velocity = Math.abs(data.velocity);
    if (Math.abs(data.acceleration) > maxValues.acceleration) maxValues.acceleration = Math.abs(data.acceleration);

    // Inject calculated maxes into data object for display/recording
    data.max_velocity = maxValues.velocity;
    data.max_acceleration = maxValues.acceleration;

    updateUI(data);
    updateCharts(data);
    updateRocketOrientation(data.orientation);

    if (isRecording) {
        recordedData.push(data);
    }
});

function updateUI(data) {
    // Update Numerical Values
    els.values.altitude.textContent = data.altitude.toFixed(1);
    els.values.maxAlt.textContent = data.max_altitude.toFixed(1);
    els.values.velocity.textContent = data.velocity.toFixed(1);
    els.values.maxVel.textContent = data.max_velocity.toFixed(1);
    els.values.accel.textContent = data.acceleration.toFixed(1);
    els.values.maxAccel.textContent = data.max_acceleration.toFixed(1);
    els.values.temp.textContent = data.temperature.toFixed(1);

    // Use packet count or estimated time
    els.values.time.textContent = data.time.toFixed(1);

    els.values.pitch.textContent = (data.orientation.pitch * (180 / Math.PI)).toFixed(2) + '°';
    els.values.yaw.textContent = (data.orientation.yaw * (180 / Math.PI)).toFixed(2) + '°';
    els.values.roll.textContent = (data.orientation.roll * (180 / Math.PI)).toFixed(2) + '°';

    // Update Mission Timer (Format seconds to MM:SS.ss)
    const totalSeconds = data.time;
    const minutes = Math.floor(totalSeconds / 60).toString().padStart(2, '0');
    const seconds = (totalSeconds % 60).toFixed(2).padStart(5, '0');
    els.missionTime.textContent = `${minutes}:${seconds} `;

    // Update State Timeline
    els.states.forEach(node => {
        if (node.dataset.state === data.state) {
            node.classList.add('active');
        } else {
            node.classList.remove('active');
        }
    });
}

// Recording Logic
els.recordBtn.addEventListener('click', () => {
    if (!isRecording) {
        // Start Recording
        isRecording = true;
        recordedData = [];
        maxValues = { velocity: 0, acceleration: 0 }; // Reset maxes on new recording? Optional.
        els.recordBtn.innerHTML = '<i class="fa-solid fa-stop"></i> Stop Recording';
        els.recordBtn.classList.add('recording');
        els.exportBtn.disabled = true;
    } else {
        // Stop Recording
        isRecording = false;
        els.recordBtn.innerHTML = '<i class="fa-solid fa-circle"></i> Start Recording';
        els.recordBtn.classList.remove('recording');
        els.exportBtn.disabled = false;
    }
});

// Export Logic
els.exportBtn.addEventListener('click', () => {
    if (recordedData.length === 0) return;

    const rows = recordedData.map(row => {
        // Flatten structure
        return [
            row.time,
            row.packet_count,
            row.state,
            row.altitude,
            row.max_altitude,
            row.velocity,
            row.max_velocity,
            row.acceleration,
            row.max_acceleration,
            row.temperature,
            row.orientation.pitch,
            row.orientation.yaw,
            row.orientation.roll,
            row.raw.ax, row.raw.ay, row.raw.az,
            row.raw.gx, row.raw.gy, row.raw.gz
        ].join(',');
    });

    const header = "time,packet_count,state,altitude,max_altitude,velocity,max_velocity,acceleration,max_acceleration,temperature,pitch,yaw,roll,ax,ay,az,gx,gy,gz";

    const csvContent = "data:text/csv;charset=utf-8," + header + "\n" + rows.join("\n");

    const encodedUri = encodeURI(csvContent);
    const link = document.createElement("a");
    link.setAttribute("href", encodedUri);
    link.setAttribute("download", `flight_data_${new Date().toISOString().slice(0, 19).replace(/:/g, "-")}.csv`);
    document.body.appendChild(link);
    link.click();
    document.body.removeChild(link);
});

