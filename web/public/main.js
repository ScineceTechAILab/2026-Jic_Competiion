
class DOMBuilder {
    static el(tag, classes = '', children = [], props = {}) {
        const element = document.createElement(tag);
        if (classes) element.className = classes;
        
        children.forEach(child => {
            if (typeof child === 'string') {
                element.appendChild(document.createTextNode(child));
            } else {
                element.appendChild(child);
            }
        });

        Object.entries(props).forEach(([key, value]) => {
            if (key === 'dataset') {
                Object.entries(value).forEach(([k, v]) => element.dataset[k] = v);
            } else {
                element[key] = value;
            }
        });

        return element;
    }
}

class App {
    constructor() {
        this.API_BASE = '/api';
        this.activeTab = 'chassis'; // chassis, imu, lidar
        this.pollInterval = null;
        
        // Chassis State
        this.currentConfig = {
            left_scale: 1.0,
            right_scale: 1.0
        };

        const root = document.getElementById('app');
        if (!root) throw new Error('Root element #app not found');
        this.appRoot = root;

        this.render();
    }

    render() {
        // Cleanup previous listeners
        if (this.tabAbortController) {
            this.tabAbortController.abort();
        }
        this.tabAbortController = new AbortController();

        this.appRoot.innerHTML = '';
        
        // Build Header
        const header = DOMBuilder.el('div', 'bg-slate-800 p-6 rounded-t-xl', [
            DOMBuilder.el('h1', 'text-2xl font-bold text-white text-center', ['Robot Control Panel']),
            DOMBuilder.el('p', 'text-gray-400 text-center text-sm mt-1', ['Chassis, IMU, and LiDAR Debugging'])
        ]);

        // Build Tabs
        const tabs = this.buildTabs();

        // Build Content Area
        const contentArea = DOMBuilder.el('div', 'p-8 bg-white min-h-[400px]');
        this.contentArea = contentArea;

        // Build Status Bar
        this.statusDiv = DOMBuilder.el('div', 'text-center p-3 rounded-lg text-sm font-medium text-gray-500 bg-gray-100 transition-all duration-300 opacity-0', ['Ready']);

        // Main Container
        const container = DOMBuilder.el('div', 'bg-white rounded-xl shadow-lg overflow-hidden', [
            header,
            tabs,
            contentArea,
            DOMBuilder.el('div', 'p-4 border-t', [this.statusDiv])
        ]);

        this.appRoot.appendChild(container);
        
        this.loadTab(this.activeTab);
    }

    buildTabs() {
        const tabs = [
            { id: 'chassis', label: 'Chassis Tuning' },
            { id: 'imu', label: 'IMU Debug' },
            { id: 'lidar', label: 'LiDAR View' },
            { id: 'camera', label: 'Camera' }
        ];

        return DOMBuilder.el('div', 'flex border-b bg-gray-50', tabs.map(tab => {
            const isActive = this.activeTab === tab.id;
            const btn = DOMBuilder.el('button', 
                `flex-1 py-4 px-6 text-sm font-medium transition-colors ${isActive ? 'text-blue-600 border-b-2 border-blue-600 bg-white' : 'text-gray-500 hover:text-gray-700 hover:bg-gray-100'}`, 
                [tab.label]
            );
            btn.onclick = () => this.switchTab(tab.id);
            return btn;
        }));
    }

    switchTab(tabId) {
        if (this.activeTab === tabId) return;
        this.activeTab = tabId;
        
        // Stop any active polling
        if (this.pollInterval) {
            clearInterval(this.pollInterval);
            this.pollInterval = null;
        }

        this.render(); // Re-render whole app to update tabs state
    }

    loadTab(tabId) {
        this.contentArea.innerHTML = '';
        if (tabId === 'chassis') {
            this.initChassisView();
        } else if (tabId === 'imu') {
            this.initIMUView();
        } else if (tabId === 'lidar') {
            this.initLidarView();
        } else if (tabId === 'camera') {
            this.initCameraView();
        }
    }

    // ==========================================
    // Chassis View
    // ==========================================
    initChassisView() {
        const batterySection = this.buildBatterySection();
        const controlSettingsSection = this.buildControlSettingsSection();
        const speedSection = this.buildSpeedSection();
        const controlsSection = this.buildControlsSection();
        
        this.contentArea.appendChild(DOMBuilder.el('div', 'space-y-8', [
            batterySection,
            controlSettingsSection,
            speedSection,
            controlsSection
        ]));

        this.bindChassisEvents();
        this.fetchChassisConfig();
        
        // Poll battery every 2 seconds
        this.pollBattery();
        this.pollInterval = setInterval(() => this.pollBattery(), 2000);
    }

    buildControlSettingsSection() {
        // Linear Speed Slider
        this.linearSpeedDisplay = DOMBuilder.el('span', 'bg-blue-100 text-blue-800 text-xs font-mono px-2 py-1 rounded', ['0.50']);
        this.linearSpeedInput = DOMBuilder.el('input', 'w-full h-2 bg-gray-200 rounded-lg appearance-none cursor-pointer accent-blue-600 hover:accent-blue-700 transition-all', [], {
            type: 'range', min: '0.1', max: '1.0', step: '0.05', value: '0.5'
        });

        const linearControl = DOMBuilder.el('div', '', [
            DOMBuilder.el('div', 'flex justify-between items-center mb-2', [
                DOMBuilder.el('label', 'text-sm font-medium text-gray-700', ['Target Linear Speed (m/s)']),
                this.linearSpeedDisplay
            ]),
            this.linearSpeedInput
        ]);

        // Angular Speed Slider
        this.angularSpeedDisplay = DOMBuilder.el('span', 'bg-blue-100 text-blue-800 text-xs font-mono px-2 py-1 rounded', ['1.50']);
        this.angularSpeedInput = DOMBuilder.el('input', 'w-full h-2 bg-gray-200 rounded-lg appearance-none cursor-pointer accent-blue-600 hover:accent-blue-700 transition-all', [], {
            type: 'range', min: '0.5', max: '3.0', step: '0.1', value: '1.5'
        });

        const angularControl = DOMBuilder.el('div', '', [
            DOMBuilder.el('div', 'flex justify-between items-center mb-2', [
                DOMBuilder.el('label', 'text-sm font-medium text-gray-700', ['Target Angular Speed (rad/s)']),
                this.angularSpeedDisplay
            ]),
            this.angularSpeedInput
        ]);

        return DOMBuilder.el('div', 'bg-gray-50 rounded-lg p-6 border border-gray-200', [
            DOMBuilder.el('h3', 'text-lg font-semibold text-gray-700 mb-4 border-b pb-2', ['Manual Control Settings']),
            DOMBuilder.el('div', 'space-y-6', [linearControl, angularControl])
        ]);
    }

    buildBatterySection() {
        this.batteryVoltageDisplay = DOMBuilder.el('div', 'text-3xl font-bold text-gray-800', ['-- V']);
        this.batteryStatusText = DOMBuilder.el('div', 'text-sm font-medium text-gray-500', ['Checking...']);
        
        // Battery Icon (Simple CSS representation)
        const batteryIcon = DOMBuilder.el('div', 'w-16 h-8 border-4 border-gray-600 rounded-lg relative flex items-center p-1', [
            DOMBuilder.el('div', 'h-full bg-green-500 rounded w-full transition-all duration-500', [], { id: 'battery-level' }),
            DOMBuilder.el('div', 'absolute -right-3 top-1/2 transform -translate-y-1/2 w-2 h-4 bg-gray-600 rounded-r-sm')
        ]);
        this.batteryLevelBar = batteryIcon.firstChild;

        return DOMBuilder.el('div', 'bg-white rounded-lg p-6 border border-gray-200 shadow-sm flex items-center justify-between', [
            DOMBuilder.el('div', '', [
                DOMBuilder.el('h3', 'text-lg font-semibold text-gray-700 mb-1', ['Battery Status']),
                this.batteryStatusText
            ]),
            DOMBuilder.el('div', 'flex items-center gap-4', [
                this.batteryVoltageDisplay,
                batteryIcon
            ])
        ]);
    }

    async pollBattery() {
        try {
            const response = await fetch(`${this.API_BASE}/battery`);
            if (!response.ok) throw new Error('Failed to fetch battery');
            const data = await response.json();
            
            const voltage = data.voltage;
            this.batteryVoltageDisplay.textContent = `${voltage.toFixed(2)} V`;
            
            // Assume 12V battery system: 
            // Max ~12.6V (100%), Min ~10.0V (0%) for 3S LiPo? 
            // Or if it's a 2S... let's assume 3S for a typical robot or 12V lead acid?
            // The driver code mentions 0x26 which is a Waveshare Motor Driver HAT usually used with 2x18650 (approx 7.4V-8.4V) or 3S.
            // Let's guess based on the "11.1V" or similar standard. 
            // If it's the Waveshare Motor Driver HAT for Pi, it usually takes 6-12V.
            // Let's set a generic range for now, maybe 9V-12.6V?
            // Wait, looking at chasis_mov.py output, it just prints the voltage.
            // Let's just map it linearly for visualization 9V to 12V for now or just show voltage.
            
            // Heuristic for 3S LiPo (11.1V nominal)
            // 12.6V = 100%, 10.0V = 0%
            let percentage = (voltage - 10.0) / (12.6 - 10.0) * 100;
            percentage = Math.max(0, Math.min(100, percentage));
            
            this.batteryLevelBar.style.width = `${percentage}%`;
            
            if (percentage < 20) {
                this.batteryLevelBar.className = 'h-full bg-red-500 rounded transition-all duration-500';
                this.batteryStatusText.textContent = 'Low Battery!';
                this.batteryStatusText.className = 'text-sm font-medium text-red-600';
            } else if (percentage < 50) {
                this.batteryLevelBar.className = 'h-full bg-yellow-500 rounded transition-all duration-500';
                this.batteryStatusText.textContent = 'Medium';
                this.batteryStatusText.className = 'text-sm font-medium text-yellow-600';
            } else {
                this.batteryLevelBar.className = 'h-full bg-green-500 rounded transition-all duration-500';
                this.batteryStatusText.textContent = 'Good';
                this.batteryStatusText.className = 'text-sm font-medium text-green-600';
            }

        } catch (e) {
            // console.error(e);
            this.batteryVoltageDisplay.textContent = '-- V';
            this.batteryStatusText.textContent = 'Error';
        }
    }

    buildSpeedSection() {
        // Left Motor
        this.leftValDisplay = DOMBuilder.el('span', 'bg-blue-100 text-blue-800 text-xs font-mono px-2 py-1 rounded', ['1.00']);
        this.leftScaleInput = DOMBuilder.el('input', 'w-full h-2 bg-gray-200 rounded-lg appearance-none cursor-pointer accent-blue-600 hover:accent-blue-700 transition-all', [], {
            type: 'range', min: '0.5', max: '1.5', step: '0.01', value: '1.0'
        });

        const leftControl = DOMBuilder.el('div', '', [
            DOMBuilder.el('div', 'flex justify-between items-center mb-2', [
                DOMBuilder.el('label', 'text-sm font-medium text-gray-700', ['Left Motor Scale (Left Wheel)']),
                this.leftValDisplay
            ]),
            this.leftScaleInput
        ]);

        // Right Motor
        this.rightValDisplay = DOMBuilder.el('span', 'bg-blue-100 text-blue-800 text-xs font-mono px-2 py-1 rounded', ['1.00']);
        this.rightScaleInput = DOMBuilder.el('input', 'w-full h-2 bg-gray-200 rounded-lg appearance-none cursor-pointer accent-blue-600 hover:accent-blue-700 transition-all', [], {
            type: 'range', min: '0.5', max: '1.5', step: '0.01', value: '1.0'
        });

        const rightControl = DOMBuilder.el('div', '', [
            DOMBuilder.el('div', 'flex justify-between items-center mb-2', [
                DOMBuilder.el('label', 'text-sm font-medium text-gray-700', ['Right Motor Scale (Right Wheel)']),
                this.rightValDisplay
            ]),
            this.rightScaleInput
        ]);

        // Save Button
        this.saveBtn = DOMBuilder.el('button', 'bg-blue-600 hover:bg-blue-700 text-white font-medium py-2 px-6 rounded-lg shadow transition-colors flex items-center gap-2', [
            DOMBuilder.el('span', '', ['Save Configuration'])
        ]);

        return DOMBuilder.el('div', 'bg-gray-50 rounded-lg p-6 border border-gray-200', [
            DOMBuilder.el('h3', 'text-lg font-semibold text-gray-700 mb-4 border-b pb-2', ['Speed Correction Factors']),
            DOMBuilder.el('div', 'space-y-6', [leftControl, rightControl]),
            DOMBuilder.el('div', 'mt-6 flex justify-center', [this.saveBtn])
        ]);
    }

    buildControlsSection() {
        // Forward Button
        this.fwdBtn = DOMBuilder.el('button', 'bg-blue-600 hover:bg-blue-700 active:bg-blue-800 text-white font-medium py-3 px-6 rounded-lg shadow transition-all transform active:scale-95 flex items-center gap-2 select-none w-full justify-center', [
            'Forward'
        ]);

        // Backward Button
        this.bwdBtn = DOMBuilder.el('button', 'bg-blue-600 hover:bg-blue-700 active:bg-blue-800 text-white font-medium py-3 px-6 rounded-lg shadow transition-all transform active:scale-95 flex items-center gap-2 select-none w-full justify-center', [
            'Backward'
        ]);

        // CW Button
        this.cwBtn = DOMBuilder.el('button', 'bg-green-600 hover:bg-green-700 active:bg-green-800 text-white font-medium py-3 px-6 rounded-lg shadow transition-all transform active:scale-95 flex items-center gap-2 select-none w-full justify-center', [
            'Rotate CW (Right)'
        ]);

        // CCW Button
        this.ccwBtn = DOMBuilder.el('button', 'bg-green-600 hover:bg-green-700 active:bg-green-800 text-white font-medium py-3 px-6 rounded-lg shadow transition-all transform active:scale-95 flex items-center gap-2 select-none w-full justify-center', [
            'Rotate CCW (Left)'
        ]);

        // Stop Button
        this.stopBtn = DOMBuilder.el('button', 'bg-red-500 hover:bg-red-600 active:bg-red-700 text-white font-medium py-3 px-8 rounded-lg shadow transition-all transform active:scale-95 flex items-center gap-2 col-span-2', [
            'STOP'
        ]);

        return DOMBuilder.el('div', 'bg-gray-50 rounded-lg p-6 border border-gray-200', [
            DOMBuilder.el('h3', 'text-lg font-semibold text-gray-700 mb-4 border-b pb-2', ['Test Controls']),
            DOMBuilder.el('p', 'text-sm text-gray-600 mb-6', ['Press and hold the buttons below to control the chassis.']),
            DOMBuilder.el('div', 'grid grid-cols-2 gap-4', [
                this.fwdBtn, this.bwdBtn,
                this.ccwBtn, this.cwBtn,
                this.stopBtn
            ])
        ]);
    }

    bindChassisEvents() {
        this.leftScaleInput.addEventListener('input', (e) => {
            this.leftValDisplay.textContent = parseFloat(e.target.value).toFixed(2);
        });

        this.rightScaleInput.addEventListener('input', (e) => {
            this.rightValDisplay.textContent = parseFloat(e.target.value).toFixed(2);
        });
        
        this.linearSpeedInput.addEventListener('input', (e) => {
            this.linearSpeedDisplay.textContent = parseFloat(e.target.value).toFixed(2);
        });

        this.angularSpeedInput.addEventListener('input', (e) => {
            this.angularSpeedDisplay.textContent = parseFloat(e.target.value).toFixed(2);
        });

        this.saveBtn.addEventListener('click', () => this.saveChassisConfig());

        this.bindPressHold(this.fwdBtn, 'move_forward');
        this.bindPressHold(this.bwdBtn, 'move_backward');
        this.bindPressHold(this.cwBtn, 'rotate_cw');
        this.bindPressHold(this.ccwBtn, 'rotate_ccw');
        this.stopBtn.addEventListener('click', () => this.sendControl('stop'));
        
        this.bindKeyboardControls();
    }

    bindKeyboardControls() {
        this.activeKey = null;
        const signal = this.tabAbortController.signal;
        
        const keyMap = {
            'w': 'move_forward',
            's': 'move_backward',
            'a': 'rotate_ccw',
            'd': 'rotate_cw',
            'arrowup': 'move_forward',
            'arrowdown': 'move_backward',
            'arrowleft': 'rotate_ccw',
            'arrowright': 'rotate_cw'
        };

        document.addEventListener('keydown', (e) => {
            if (e.repeat) return;
            if (e.target.tagName === 'INPUT') return;
            
            const key = e.key.toLowerCase();
            const action = keyMap[key];
            
            if (action && !this.activeKey) {
                this.activeKey = key;
                this.sendControl(action);
                this.highlightButton(action, true);
            }
        }, { signal });

        document.addEventListener('keyup', (e) => {
            const key = e.key.toLowerCase();
            if (key === this.activeKey) {
                this.activeKey = null;
                this.sendControl('stop');
                
                const action = keyMap[key];
                if (action) this.highlightButton(action, false);
            }
        }, { signal });
    }

    highlightButton(action, active) {
        let btn;
        switch(action) {
            case 'move_forward': btn = this.fwdBtn; break;
            case 'move_backward': btn = this.bwdBtn; break;
            case 'rotate_cw': btn = this.cwBtn; break;
            case 'rotate_ccw': btn = this.ccwBtn; break;
        }
        
        if (btn) {
            if (active) {
                btn.classList.add('ring-4', 'ring-blue-300', 'transform', 'scale-95');
            } else {
                btn.classList.remove('ring-4', 'ring-blue-300', 'transform', 'scale-95');
            }
        }
    }

    bindPressHold(btn, action) {
        const start = () => this.sendControl(action);
        const end = () => this.sendControl('stop');

        btn.addEventListener('mousedown', start);
        btn.addEventListener('mouseup', end);
        btn.addEventListener('mouseleave', end);
        
        btn.addEventListener('touchstart', (e) => {
            e.preventDefault();
            start();
        });
        btn.addEventListener('touchend', end);
    }

    async fetchChassisConfig() {
        try {
            const response = await fetch(`${this.API_BASE}/config`);
            if (!response.ok) throw new Error('Failed to fetch config');
            
            const data = await response.json();
            this.currentConfig = data;
            
            // Update UI
            if (this.leftScaleInput && this.rightScaleInput) {
                this.leftScaleInput.value = this.currentConfig.left_scale;
                this.rightScaleInput.value = this.currentConfig.right_scale;
                this.leftValDisplay.textContent = parseFloat(this.currentConfig.left_scale).toFixed(2);
                this.rightValDisplay.textContent = parseFloat(this.currentConfig.right_scale).toFixed(2);
            }
        } catch (e) {
            this.showStatus(`Error loading config: ${e}`, 'error');
        }
    }

    async saveChassisConfig() {
        const newConfig = {
            left_scale: parseFloat(this.leftScaleInput.value),
            right_scale: parseFloat(this.rightScaleInput.value)
        };

        this.setLoading(true);

        try {
            const response = await fetch(`${this.API_BASE}/config`, {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify(newConfig)
            });
            
            if (!response.ok) throw new Error('Failed to save');
            
            await response.json();
            this.showStatus('Configuration saved successfully!', 'success');
            this.currentConfig = newConfig;
        } catch (e) {
            this.showStatus(`Error saving config: ${e}`, 'error');
        } finally {
            this.setLoading(false);
        }
    }

    async sendControl(action) {
        try {
            const payload = { action };
            
            // Add speed parameters if inputs exist (Chassis tab), otherwise use defaults
            if (this.linearSpeedInput) {
                payload.linear_speed = parseFloat(this.linearSpeedInput.value);
            }
            if (this.angularSpeedInput) {
                payload.angular_speed = parseFloat(this.angularSpeedInput.value);
            }

            const response = await fetch(`${this.API_BASE}/control`, {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify(payload)
            });
            
            if (!response.ok) throw new Error('Command failed');
            
            if (action === 'stop') {
                this.showStatus('Stopped', 'info');
            } else {
                this.showStatus(`Running: ${action.replace('_', ' ').toUpperCase()}`, 'success');
            }
        } catch (e) {
            this.showStatus(`Error executing command: ${e}`, 'error');
        }
    }

    // ==========================================
    // Camera View
    // ==========================================
    initCameraView() {
        console.log('Initializing Camera View');
        const streamUrl = `${this.API_BASE}/camera/stream?t=${Date.now()}`;
        
        const videoElement = DOMBuilder.el('img', 'w-full h-[500px] object-contain bg-gray-100 rounded-lg border border-gray-200', [], {
            src: streamUrl,
            alt: 'Camera Stream'
        });

        // Add error handling for the image
        videoElement.onerror = () => {
            console.error('Camera stream failed to load');
            // We can show a placeholder or error message here if needed
            // For now, let's keep the gray background which is already set
        };

        const container = DOMBuilder.el('div', 'flex flex-col items-center gap-4 w-full', [
            DOMBuilder.el('h3', 'text-lg font-semibold text-gray-700', ['USB Camera Stream']),
            videoElement,
            DOMBuilder.el('p', 'text-sm text-gray-500', ['Real-time video feed from USB camera'])
        ]);

        this.contentArea.appendChild(container);
    }


    // ==========================================
    // IMU View
    // ==========================================
    initIMUView() {
        const createCard = (title, id) => {
            return DOMBuilder.el('div', 'bg-gray-50 p-4 rounded-lg border', [
                DOMBuilder.el('h3', 'text-sm font-semibold text-gray-500 uppercase mb-2', [title]),
                DOMBuilder.el('div', 'font-mono text-sm space-y-1', [], { id: id })
            ]);
        };

        const orientationCard = createCard('Orientation (Quaternion)', 'imu-orient');
        const gyroCard = createCard('Angular Velocity (rad/s)', 'imu-gyro');
        const accelCard = createCard('Linear Acceleration (m/s²)', 'imu-accel');

        this.contentArea.appendChild(DOMBuilder.el('div', 'grid grid-cols-1 md:grid-cols-3 gap-4', [
            orientationCard, gyroCard, accelCard
        ]));

        this.pollInterval = setInterval(() => this.pollIMU(), 200); // 5Hz
    }

    async pollIMU() {
        try {
            const response = await fetch(`${this.API_BASE}/imu`);
            if (!response.ok) throw new Error('Failed to fetch IMU');
            const data = await response.json();

            const format = (obj) => {
                return Object.entries(obj)
                    .map(([k, v]) => `${k}: ${v.toFixed(3)}`)
                    .join('<br>');
            };

            const orientEl = document.getElementById('imu-orient');
            const gyroEl = document.getElementById('imu-gyro');
            const accelEl = document.getElementById('imu-accel');

            if (orientEl) orientEl.innerHTML = format(data.orientation);
            if (gyroEl) gyroEl.innerHTML = format(data.angular_velocity);
            if (accelEl) accelEl.innerHTML = format(data.linear_acceleration);

        } catch (e) {
            // this.showStatus(`IMU Error: ${e}`, 'error');
        }
    }

    // ==========================================
    // LiDAR View
    // ==========================================
    initLidarView() {
        const canvas = DOMBuilder.el('canvas', 'w-full h-[500px] bg-black rounded-lg shadow', [], {
            width: 800,
            height: 800
        });
        this.lidarCanvas = canvas;
        this.lidarCtx = canvas.getContext('2d');

        const container = DOMBuilder.el('div', 'flex flex-col items-center gap-4', [
            DOMBuilder.el('h3', 'text-lg font-semibold text-gray-700', ['LiDAR Scan (Top Down)']),
            canvas,
            DOMBuilder.el('p', 'text-sm text-gray-500', ['Center is robot position. Scale: 100px = 1m'])
        ]);

        this.contentArea.appendChild(container);

        this.pollInterval = setInterval(() => this.pollLidar(), 200); // 5Hz
    }

    async pollLidar() {
        try {
            const response = await fetch(`${this.API_BASE}/lidar`);
            if (!response.ok) throw new Error('Failed to fetch LiDAR');
            const data = await response.json();
            
            this.drawLidar(data);
        } catch (e) {
            // this.showStatus(`LiDAR Error: ${e}`, 'error');
        }
    }

    drawLidar(data) {
        const ctx = this.lidarCtx;
        const width = this.lidarCanvas.width;
        const height = this.lidarCanvas.height;
        const cx = width / 2;
        const cy = height / 2;
        const scale = 100; // 100 pixels per meter

        // Clear
        ctx.fillStyle = '#000000';
        ctx.fillRect(0, 0, width, height);

        // Draw Grid
        ctx.strokeStyle = '#333333';
        ctx.lineWidth = 1;
        ctx.beginPath();
        for (let i = 0; i < width; i += scale) {
            ctx.moveTo(i, 0); ctx.lineTo(i, height);
            ctx.moveTo(0, i); ctx.lineTo(width, i);
        }
        ctx.stroke();

        // Draw Robot Center
        ctx.fillStyle = '#FF0000';
        ctx.beginPath();
        ctx.arc(cx, cy, 5, 0, 2 * Math.PI);
        ctx.fill();

        // Draw Points
        ctx.fillStyle = '#00FF00';
        const ranges = data.ranges;
        const angleIncrement = (data.angle_max - data.angle_min) / ranges.length;

        for (let i = 0; i < ranges.length; i++) {
            const dist = ranges[i];
            if (dist < data.range_min || dist > data.range_max) continue;

            const angle = data.angle_min + i * angleIncrement;
            
            // Convert polar to cartesian
            // Canvas Y is down, so we flip Y
            const x = cx + (dist * Math.cos(angle)) * scale;
            const y = cy - (dist * Math.sin(angle)) * scale;

            ctx.fillRect(x, y, 2, 2);
        }
    }


    // ==========================================
    // Shared Utilities
    // ==========================================
    showStatus(msg, type = 'info') {
        if (!this.statusDiv) return;
        
        this.statusDiv.textContent = msg;
        this.statusDiv.className = 'text-center p-3 rounded-lg text-sm font-medium transition-all duration-300 opacity-100';
        
        if (type === 'success') {
            this.statusDiv.classList.add('bg-green-100', 'text-green-800');
        } else if (type === 'error') {
            this.statusDiv.classList.add('bg-red-100', 'text-red-800');
        } else {
            this.statusDiv.classList.add('bg-gray-100', 'text-gray-600');
        }

        if (type !== 'error') {
            setTimeout(() => {
                this.statusDiv.classList.remove('opacity-100');
                this.statusDiv.classList.add('opacity-0');
            }, 3000);
        }
    }

    setLoading(isLoading) {
        if (!this.saveBtn) return;
        
        if (isLoading) {
            this.saveBtn.disabled = true;
            this.saveBtn.classList.add('opacity-50', 'cursor-not-allowed');
            this.saveBtn.textContent = 'Saving...';
        } else {
            this.saveBtn.disabled = false;
            this.saveBtn.classList.remove('opacity-50', 'cursor-not-allowed');
            this.saveBtn.textContent = 'Save Configuration';
        }
    }
}

document.addEventListener('DOMContentLoaded', () => {
    new App();
});
