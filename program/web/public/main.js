class DOMBuilder {
    static el(tag, classes = '', children = [], props = {}) {
        const element = document.createElement(tag);
        if (classes) element.className = classes;

        children.forEach((child) => {
            if (typeof child === 'string') {
                element.appendChild(document.createTextNode(child));
            } else if (child) {
                element.appendChild(child);
            }
        });

        Object.entries(props).forEach(([key, value]) => {
            if (key === 'dataset') {
                Object.entries(value).forEach(([k, v]) => {
                    element.dataset[k] = v;
                });
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
        this.currentConfig = {
            left_scale: 1.0,
            right_scale: 1.0,
        };

        this.pollTimers = [];
        this.activeKey = null;

        const root = document.getElementById('app');
        if (!root) throw new Error('Root element #app not found');
        this.appRoot = root;

        this.render();
        this.init();
    }

    render() {
        this.appRoot.innerHTML = '';

        const shell = DOMBuilder.el('div', 'min-h-screen text-slate-100', [
            DOMBuilder.el('div', 'mx-auto max-w-7xl px-4 py-6 sm:px-6 lg:px-8 lg:py-10', [
                DOMBuilder.el('div', 'rounded-[2rem] border border-white/10 bg-slate-950/90 shadow-2xl shadow-slate-950/40 backdrop-blur overflow-hidden', [
                    this.buildHero(),
                    this.buildQuickNav(),
                    DOMBuilder.el('main', 'space-y-8 bg-slate-50 px-4 py-6 text-slate-900 sm:px-6 lg:px-8 lg:py-8', [
                        this.buildChassisSection(),
                        this.buildIMUSection(),
                        this.buildLidarSection(),
                        this.buildCameraSection(),
                        this.buildFooterStatus(),
                    ]),
                ]),
            ]),
        ]);

        this.appRoot.appendChild(shell);
    }

    buildHero() {
        return DOMBuilder.el('header', 'relative overflow-hidden bg-slate-950 px-5 py-6 text-white sm:px-8 sm:py-8', [
            DOMBuilder.el('div', 'absolute inset-0 opacity-30', []),
            DOMBuilder.el('div', 'relative flex flex-col gap-5 lg:flex-row lg:items-end lg:justify-between', [
                DOMBuilder.el('div', 'space-y-3', [
                    DOMBuilder.el('div', 'inline-flex items-center gap-2 rounded-full border border-cyan-400/30 bg-cyan-400/10 px-3 py-1 text-xs font-semibold tracking-[0.24em] text-cyan-200 uppercase', [
                        'Robot Control Panel',
                    ]),
                    DOMBuilder.el('div', 'space-y-2', [
                        DOMBuilder.el('h1', 'text-3xl font-bold tracking-tight sm:text-4xl lg:text-5xl', [
                            'One-page chassis, IMU, LiDAR, and camera dashboard',
                        ]),
                        DOMBuilder.el('p', 'max-w-3xl text-sm leading-6 text-slate-300 sm:text-base', [
                            'A responsive control surface that keeps chassis tuning, inertial telemetry, laser scan data, and camera preview visible together on desktop and mobile.',
                        ]),
                    ]),
                ]),
                DOMBuilder.el('div', 'grid grid-cols-2 gap-3 sm:grid-cols-3 lg:min-w-[320px]', [
                    this.buildHeroBadge('Battery', '-- V', 'battery-hero-badge'),
                    this.buildHeroBadge('Left speed', '-- m/s', 'left-speed-hero-badge'),
                    this.buildHeroBadge('Right speed', '-- m/s', 'right-speed-hero-badge'),
                ]),
            ]),
        ]);
    }

    buildHeroBadge(label, value, id) {
        const valueNode = DOMBuilder.el('div', 'text-lg font-semibold text-white sm:text-xl', [value], { id });
        return DOMBuilder.el('div', 'rounded-2xl border border-white/10 bg-white/5 px-4 py-3 backdrop-blur-sm', [
            DOMBuilder.el('div', 'text-[0.72rem] font-semibold uppercase tracking-[0.22em] text-slate-400', [label]),
            valueNode,
        ]);
    }

    buildQuickNav() {
        const links = [
            ['Chassis tuning', '#chassis'],
            ['IMU debug', '#imu'],
            ['LiDAR view', '#lidar'],
            ['Camera', '#camera'],
        ];

        return DOMBuilder.el('nav', 'border-t border-b border-white/10 bg-slate-900 px-4 py-3 sm:px-8', [
            DOMBuilder.el('div', 'flex flex-wrap gap-2', links.map(([label, href]) => (
                DOMBuilder.el('a', 'rounded-full border border-slate-700 bg-slate-800 px-4 py-2 text-sm font-medium text-slate-200 transition hover:border-cyan-400 hover:text-white hover:shadow-lg hover:shadow-cyan-500/10', [label], {
                    href,
                })
            ))),
        ]);
    }

    buildSectionShell(id, title, description, accent, contentNode) {
        return DOMBuilder.el('section', 'scroll-mt-6 rounded-3xl border border-slate-200 bg-white shadow-lg shadow-slate-200/70', [
            DOMBuilder.el('div', 'border-b border-slate-100 px-5 py-5 sm:px-6', [
                DOMBuilder.el('div', 'flex flex-col gap-2', [
                    DOMBuilder.el('div', 'inline-flex items-center self-start rounded-full px-3 py-1 text-xs font-semibold uppercase tracking-[0.24em] ' + accent.badge, [accent.label]),
                    DOMBuilder.el('div', 'space-y-1', [
                        DOMBuilder.el('h2', 'text-2xl font-bold tracking-tight text-slate-900 sm:text-[1.75rem]', [title]),
                        DOMBuilder.el('p', 'max-w-4xl text-sm leading-6 text-slate-600 sm:text-base', [description]),
                    ]),
                ]),
            ]),
            DOMBuilder.el('div', 'px-5 py-5 sm:px-6 sm:py-6', [contentNode]),
        ], { id });
    }

    buildChassisSection() {
        this.batteryVoltageDisplay = DOMBuilder.el('div', 'text-3xl font-bold tracking-tight text-slate-900 sm:text-4xl', ['-- V']);
        this.batteryStatusText = DOMBuilder.el('div', 'text-sm font-medium text-slate-500', ['Checking...']);
        this.batteryLevelBar = DOMBuilder.el('div', 'h-full w-full rounded-full bg-emerald-500 transition-all duration-500', [], { id: 'battery-level' });

        const batteryIcon = DOMBuilder.el('div', 'relative flex h-9 w-20 items-center rounded-xl border-4 border-slate-600 p-1 sm:h-10 sm:w-24', [
            this.batteryLevelBar,
            DOMBuilder.el('div', 'absolute -right-3 top-1/2 h-4 w-2 -translate-y-1/2 rounded-r-sm bg-slate-600', []),
        ]);

        const batteryCard = DOMBuilder.el('div', 'rounded-2xl border border-slate-200 bg-slate-50 p-5 shadow-sm', [
            DOMBuilder.el('div', 'flex flex-col gap-4 sm:flex-row sm:items-center sm:justify-between', [
                DOMBuilder.el('div', 'space-y-1', [
                    DOMBuilder.el('h3', 'text-lg font-semibold text-slate-900', ['Battery Status']),
                    this.batteryStatusText,
                ]),
                DOMBuilder.el('div', 'flex items-center gap-4', [
                    this.batteryVoltageDisplay,
                    batteryIcon,
                ]),
            ]),
        ]);

        this.leftSpeedDisplay = DOMBuilder.el('div', 'text-2xl font-bold text-slate-900', ['-- m/s']);
        this.rightSpeedDisplay = DOMBuilder.el('div', 'text-2xl font-bold text-slate-900', ['-- m/s']);

        const speedCards = DOMBuilder.el('div', 'grid gap-4 sm:grid-cols-2', [
            DOMBuilder.el('div', 'rounded-2xl border border-slate-200 bg-white p-5 shadow-sm', [
                DOMBuilder.el('div', 'text-sm font-medium text-slate-500', ['Left Wheel Speed']),
                this.leftSpeedDisplay,
            ]),
            DOMBuilder.el('div', 'rounded-2xl border border-slate-200 bg-white p-5 shadow-sm', [
                DOMBuilder.el('div', 'text-sm font-medium text-slate-500', ['Right Wheel Speed']),
                this.rightSpeedDisplay,
            ]),
        ]);

        const correctionSection = this.buildSpeedCorrectionPanel();
        const motionSection = this.buildMotionControlPanel();

        const layout = DOMBuilder.el('div', 'grid gap-6 xl:grid-cols-[minmax(0,1.15fr)_minmax(320px,0.85fr)]', [
            DOMBuilder.el('div', 'space-y-6', [batteryCard, speedCards]),
            DOMBuilder.el('div', 'space-y-6', [correctionSection, motionSection]),
        ]);

        const section = this.buildSectionShell(
            'chassis',
            'Chassis Tuning',
            'Live battery, wheel speed, and motor control live in the same page so tuning on desktop or on a phone stays fast.',
            { label: 'Drive train', badge: 'bg-amber-50 text-amber-700' },
            layout,
        );

        return section;
    }

    buildSpeedCorrectionPanel() {
        this.leftValDisplay = DOMBuilder.el('span', 'rounded-full bg-cyan-50 px-2.5 py-1 text-xs font-mono font-semibold text-cyan-700', ['1.00']);
        this.leftScaleInput = DOMBuilder.el('input', 'w-full accent-cyan-600', [], {
            type: 'range',
            min: '0.5',
            max: '1.5',
            step: '0.01',
            value: '1.0',
        });

        this.rightValDisplay = DOMBuilder.el('span', 'rounded-full bg-cyan-50 px-2.5 py-1 text-xs font-mono font-semibold text-cyan-700', ['1.00']);
        this.rightScaleInput = DOMBuilder.el('input', 'w-full accent-cyan-600', [], {
            type: 'range',
            min: '0.5',
            max: '1.5',
            step: '0.01',
            value: '1.0',
        });

        this.saveBtn = DOMBuilder.el('button', 'inline-flex min-h-11 items-center justify-center rounded-xl bg-slate-950 px-5 py-3 text-sm font-semibold text-white transition hover:bg-slate-800 disabled:cursor-not-allowed disabled:opacity-50', [
            'Save configuration',
        ]);

        return DOMBuilder.el('div', 'rounded-2xl border border-slate-200 bg-white p-5 shadow-sm', [
            DOMBuilder.el('div', 'mb-4 flex items-center justify-between', [
                DOMBuilder.el('h3', 'text-lg font-semibold text-slate-900', ['Speed Correction']),
                DOMBuilder.el('span', 'rounded-full bg-slate-100 px-3 py-1 text-xs font-semibold uppercase tracking-[0.2em] text-slate-500', ['Fine tune']),
            ]),
            DOMBuilder.el('div', 'space-y-5', [
                DOMBuilder.el('div', 'space-y-2', [
                    DOMBuilder.el('div', 'flex items-center justify-between gap-3', [
                        DOMBuilder.el('label', 'text-sm font-medium text-slate-700', ['Left motor scale']),
                        this.leftValDisplay,
                    ]),
                    this.leftScaleInput,
                ]),
                DOMBuilder.el('div', 'space-y-2', [
                    DOMBuilder.el('div', 'flex items-center justify-between gap-3', [
                        DOMBuilder.el('label', 'text-sm font-medium text-slate-700', ['Right motor scale']),
                        this.rightValDisplay,
                    ]),
                    this.rightScaleInput,
                ]),
                DOMBuilder.el('div', 'pt-1', [this.saveBtn]),
            ]),
        ]);
    }

    buildMotionControlPanel() {
        this.linearSpeedDisplay = DOMBuilder.el('span', 'rounded-full bg-slate-100 px-2.5 py-1 text-xs font-mono font-semibold text-slate-700', ['0.50']);
        this.linearSpeedInput = DOMBuilder.el('input', 'w-full accent-cyan-600', [], {
            type: 'range',
            min: '0.1',
            max: '1.0',
            step: '0.05',
            value: '0.5',
        });

        this.angularSpeedDisplay = DOMBuilder.el('span', 'rounded-full bg-slate-100 px-2.5 py-1 text-xs font-mono font-semibold text-slate-700', ['1.50']);
        this.angularSpeedInput = DOMBuilder.el('input', 'w-full accent-cyan-600', [], {
            type: 'range',
            min: '0.5',
            max: '3.0',
            step: '0.1',
            value: '1.5',
        });

        this.fwdBtn = DOMBuilder.el('button', 'inline-flex min-h-11 items-center justify-center rounded-xl bg-cyan-600 px-4 py-3 text-sm font-semibold text-white transition hover:bg-cyan-700 active:scale-[0.99] touch-none', ['Forward']);
        this.bwdBtn = DOMBuilder.el('button', 'inline-flex min-h-11 items-center justify-center rounded-xl bg-cyan-600 px-4 py-3 text-sm font-semibold text-white transition hover:bg-cyan-700 active:scale-[0.99] touch-none', ['Backward']);
        this.cwBtn = DOMBuilder.el('button', 'inline-flex min-h-11 items-center justify-center rounded-xl bg-emerald-600 px-4 py-3 text-sm font-semibold text-white transition hover:bg-emerald-700 active:scale-[0.99] touch-none', ['Rotate CW']);
        this.ccwBtn = DOMBuilder.el('button', 'inline-flex min-h-11 items-center justify-center rounded-xl bg-emerald-600 px-4 py-3 text-sm font-semibold text-white transition hover:bg-emerald-700 active:scale-[0.99] touch-none', ['Rotate CCW']);
        this.stopBtn = DOMBuilder.el('button', 'inline-flex min-h-11 items-center justify-center rounded-xl bg-rose-500 px-5 py-3 text-sm font-semibold text-white transition hover:bg-rose-600 active:scale-[0.99] touch-none', ['STOP']);

        return DOMBuilder.el('div', 'rounded-2xl border border-slate-200 bg-white p-5 shadow-sm', [
            DOMBuilder.el('div', 'mb-4 flex items-center justify-between', [
                DOMBuilder.el('h3', 'text-lg font-semibold text-slate-900', ['Manual Control']),
                DOMBuilder.el('span', 'rounded-full bg-slate-100 px-3 py-1 text-xs font-semibold uppercase tracking-[0.2em] text-slate-500', ['Hold or tap']),
            ]),
            DOMBuilder.el('div', 'space-y-5', [
                DOMBuilder.el('div', 'space-y-2', [
                    DOMBuilder.el('div', 'flex items-center justify-between gap-3', [
                        DOMBuilder.el('label', 'text-sm font-medium text-slate-700', ['Linear speed']),
                        this.linearSpeedDisplay,
                    ]),
                    this.linearSpeedInput,
                ]),
                DOMBuilder.el('div', 'space-y-2', [
                    DOMBuilder.el('div', 'flex items-center justify-between gap-3', [
                        DOMBuilder.el('label', 'text-sm font-medium text-slate-700', ['Angular speed']),
                        this.angularSpeedDisplay,
                    ]),
                    this.angularSpeedInput,
                ]),
                DOMBuilder.el('div', 'grid grid-cols-1 gap-3 sm:grid-cols-2', [
                    this.fwdBtn,
                    this.bwdBtn,
                    this.ccwBtn,
                    this.cwBtn,
                ]),
                DOMBuilder.el('div', 'pt-1', [this.stopBtn]),
            ]),
        ]);
    }

    buildIMUSection() {
        this.imuConnEl = DOMBuilder.el('div', 'whitespace-pre-line font-mono text-sm text-slate-700', ['connection: --']);
        this.imuTimeEl = DOMBuilder.el('div', 'whitespace-pre-line font-mono text-sm text-slate-700', ['timestamp: --']);
        this.imuCalibEl = DOMBuilder.el('div', 'whitespace-pre-line font-mono text-sm text-slate-700', ['--']);
        this.imuOrientEl = DOMBuilder.el('div', 'whitespace-pre-line font-mono text-sm text-slate-700', ['--']);
        this.imuGyroEl = DOMBuilder.el('div', 'whitespace-pre-line font-mono text-sm text-slate-700', ['--']);
        this.imuAccelEl = DOMBuilder.el('div', 'whitespace-pre-line font-mono text-sm text-slate-700', ['--']);

        const content = DOMBuilder.el('div', 'grid gap-4 md:grid-cols-2 xl:grid-cols-3', [
            this.buildDataCard('Status', this.imuConnEl, 'Connected state and sample timestamp.'),
            this.buildDataCard('Calibration', this.imuCalibEl, 'BNO055 calibration counters.'),
            this.buildDataCard('Orientation', this.imuOrientEl, 'Quaternion or pose-like values.'),
            this.buildDataCard('Angular velocity', this.imuGyroEl, 'Gyroscope values in rad/s.'),
            this.buildDataCard('Linear acceleration', this.imuAccelEl, 'Acceleration values in m/s².'),
            this.buildDataCard('Time', this.imuTimeEl, 'Source timestamp converted to local time.'),
        ]);

        return this.buildSectionShell(
            'imu',
            'IMU Debug',
            'This panel keeps the inertial stream visible, which is useful for quick validation on the robot without switching pages.',
            { label: 'Inertial', badge: 'bg-cyan-50 text-cyan-700' },
            content,
        );
    }

    buildLidarSection() {
        this.lidarCanvas = DOMBuilder.el('canvas', 'w-full max-w-3xl rounded-3xl border border-slate-200 bg-slate-950 shadow-2xl', [], {
            width: 720,
            height: 720,
        });
        this.lidarCtx = this.lidarCanvas.getContext('2d');
        this.lidarStateEl = DOMBuilder.el('div', 'text-sm leading-6 text-slate-600', ['Waiting for scan data...']);

        const scanPanel = DOMBuilder.el('div', 'space-y-4', [
            DOMBuilder.el('div', 'flex flex-wrap items-center gap-2 text-xs font-semibold uppercase tracking-[0.2em] text-slate-500', [
                DOMBuilder.el('span', 'rounded-full bg-slate-100 px-3 py-1', ['Top-down scan']),
                DOMBuilder.el('span', 'rounded-full bg-slate-100 px-3 py-1', ['Center = robot']),
                DOMBuilder.el('span', 'rounded-full bg-slate-100 px-3 py-1', ['Scale = 1 m / 100 px']),
            ]),
            this.lidarCanvas,
            this.lidarStateEl,
        ]);

        const infoPanel = DOMBuilder.el('div', 'grid gap-4 sm:grid-cols-2 xl:grid-cols-1', [
            this.buildDataCard('Usage', DOMBuilder.el('div', 'whitespace-pre-line font-mono text-sm text-slate-700', [
                'range_min: --\nrange_max: --\nangle_min: --\nangle_max: --',
            ]), 'The live scan is drawn on the canvas; the raw values stay summarized here.'),
            this.buildDataCard('Notes', DOMBuilder.el('div', 'whitespace-pre-line font-mono text-sm text-slate-700', [
                'Use this view to confirm returns, dead zones, and the overall point cloud shape before tuning navigation.',
            ]), 'This panel is optimized for fast visual checks on desktop and mobile.'),
        ]);

        const layout = DOMBuilder.el('div', 'grid gap-6 xl:grid-cols-[minmax(0,1.2fr)_minmax(280px,0.8fr)]', [
            scanPanel,
            infoPanel,
        ]);

        return this.buildSectionShell(
            'lidar',
            'LiDAR View',
            'The scan is rendered in-place, so you can verify geometry and coverage alongside the other robot signals.',
            { label: 'Laser scan', badge: 'bg-emerald-50 text-emerald-700' },
            layout,
        );
    }

    buildCameraSection() {
        this.cameraStatusEl = DOMBuilder.el('div', 'text-sm leading-6 text-slate-600', ['Streaming MJPEG feed from the robot camera.']);
        const streamUrl = `${this.API_BASE}/camera/stream?t=${Date.now()}`;
        this.cameraFrame = DOMBuilder.el('img', 'w-full rounded-3xl border border-slate-200 bg-slate-950 object-contain shadow-2xl', [], {
            src: streamUrl,
            alt: 'Camera stream',
        });
        this.cameraFrame.onerror = () => {
            this.cameraStatusEl.textContent = 'Camera stream failed to load. Check the camera driver and network connection.';
        };

        const previewPanel = DOMBuilder.el('div', 'space-y-4', [
            this.cameraFrame,
            this.cameraStatusEl,
        ]);

        const tipsPanel = DOMBuilder.el('div', 'grid gap-4 sm:grid-cols-2 xl:grid-cols-1', [
            this.buildDataCard('Preview', DOMBuilder.el('div', 'whitespace-pre-line font-mono text-sm text-slate-700', [
                'Live feed is loaded once and then kept open by the browser.\nUse it for rapid exposure and framing checks.',
            ]), 'The stream updates continuously without page refresh.'),
            this.buildDataCard('Mobile tip', DOMBuilder.el('div', 'whitespace-pre-line font-mono text-sm text-slate-700', [
                'Rotate the phone to landscape when you need a wider view.\nThe frame will still fit the page width.',
            ]), 'The layout collapses to a single column on narrow screens.'),
        ]);

        const layout = DOMBuilder.el('div', 'grid gap-6 xl:grid-cols-[minmax(0,1.2fr)_minmax(280px,0.8fr)]', [
            previewPanel,
            tipsPanel,
        ]);

        return this.buildSectionShell(
            'camera',
            'Camera',
            'The camera feed sits beside the other control panes, so checking alignment or exposure does not require leaving the dashboard.',
            { label: 'Vision', badge: 'bg-violet-50 text-violet-700' },
            layout,
        );
    }

    buildDataCard(title, contentNode, description) {
        return DOMBuilder.el('div', 'rounded-2xl border border-slate-200 bg-slate-50 p-4 shadow-sm', [
            DOMBuilder.el('div', 'mb-3 flex items-start justify-between gap-3', [
                DOMBuilder.el('h3', 'text-sm font-semibold uppercase tracking-[0.18em] text-slate-500', [title]),
                DOMBuilder.el('span', 'rounded-full bg-white px-2.5 py-1 text-[0.65rem] font-semibold uppercase tracking-[0.2em] text-slate-400', ['Live']),
            ]),
            contentNode,
            DOMBuilder.el('div', 'mt-3 text-xs leading-5 text-slate-500', [description]),
        ]);
    }

    buildFooterStatus() {
        this.statusDiv = DOMBuilder.el('div', 'rounded-2xl border border-slate-200 bg-slate-950 px-4 py-3 text-sm font-medium text-slate-200 shadow-lg shadow-slate-200/70 transition-opacity duration-300 opacity-0', ['Ready']);

        return DOMBuilder.el('footer', 'pt-2', [
            this.statusDiv,
        ]);
    }

    init() {
        this.bindEvents();
        this.bindKeyboardControls();
        this.fetchChassisConfig();
        this.pollBattery();
        this.pollSpeed();
        this.pollIMU();
        this.pollLidar();

        this.pollTimers.push(setInterval(() => this.pollBattery(), 2000));
        this.pollTimers.push(setInterval(() => this.pollSpeed(), 250));
        this.pollTimers.push(setInterval(() => this.pollIMU(), 250));
        this.pollTimers.push(setInterval(() => this.pollLidar(), 500));
    }

    bindEvents() {
        this.leftScaleInput.addEventListener('input', (event) => {
            const target = event.target;
            this.leftValDisplay.textContent = parseFloat(target.value).toFixed(2);
        });

        this.rightScaleInput.addEventListener('input', (event) => {
            const target = event.target;
            this.rightValDisplay.textContent = parseFloat(target.value).toFixed(2);
        });

        this.linearSpeedInput.addEventListener('input', (event) => {
            const target = event.target;
            this.linearSpeedDisplay.textContent = parseFloat(target.value).toFixed(2);
        });

        this.angularSpeedInput.addEventListener('input', (event) => {
            const target = event.target;
            this.angularSpeedDisplay.textContent = parseFloat(target.value).toFixed(2);
        });

        this.saveBtn.addEventListener('click', () => this.saveChassisConfig());

        this.bindPressHold(this.fwdBtn, 'move_forward');
        this.bindPressHold(this.bwdBtn, 'move_backward');
        this.bindPressHold(this.cwBtn, 'rotate_cw');
        this.bindPressHold(this.ccwBtn, 'rotate_ccw');
        this.stopBtn.addEventListener('click', () => this.sendControl('stop'));
    }

    bindKeyboardControls() {
        const keyMap = {
            w: 'move_forward',
            s: 'move_backward',
            a: 'rotate_ccw',
            d: 'rotate_cw',
            arrowup: 'move_forward',
            arrowdown: 'move_backward',
            arrowleft: 'rotate_ccw',
            arrowright: 'rotate_cw',
        };

        document.addEventListener('keydown', (event) => {
            if (event.repeat) return;
            const activeTag = event.target && event.target.tagName;
            if (activeTag === 'INPUT' || activeTag === 'TEXTAREA') return;

            const key = event.key.toLowerCase();
            const action = keyMap[key];
            if (action && !this.activeKey) {
                this.activeKey = key;
                this.sendControl(action);
            }
        });

        document.addEventListener('keyup', (event) => {
            const key = event.key.toLowerCase();
            if (key === this.activeKey) {
                this.activeKey = null;
                this.sendControl('stop');
            }
        });
    }

    bindPressHold(btn, action) {
        const start = () => this.sendControl(action);
        const end = () => this.sendControl('stop');

        btn.addEventListener('pointerdown', (event) => {
            event.preventDefault();
            btn.setPointerCapture?.(event.pointerId);
            start();
        });
        btn.addEventListener('pointerup', end);
        btn.addEventListener('pointercancel', end);
        btn.addEventListener('pointerleave', end);
    }

    async fetchChassisConfig() {
        try {
            const response = await fetch(`${this.API_BASE}/config`);
            if (!response.ok) throw new Error('Failed to fetch config');

            const data = await response.json();
            this.currentConfig = data;
            this.updateUI();
            this.showStatus('Configuration loaded', 'success');
        } catch (error) {
            this.showStatus(`Error loading config: ${error}`, 'error');
        }
    }

    updateUI() {
        this.leftScaleInput.value = this.currentConfig.left_scale.toString();
        this.rightScaleInput.value = this.currentConfig.right_scale.toString();
        this.leftValDisplay.textContent = this.currentConfig.left_scale.toFixed(2);
        this.rightValDisplay.textContent = this.currentConfig.right_scale.toFixed(2);
    }

    async saveChassisConfig() {
        const newConfig = {
            left_scale: parseFloat(this.leftScaleInput.value),
            right_scale: parseFloat(this.rightScaleInput.value),
        };

        this.setLoading(true);

        try {
            const response = await fetch(`${this.API_BASE}/config`, {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify(newConfig),
            });

            if (!response.ok) throw new Error('Failed to save');

            await response.json();
            this.currentConfig = newConfig;
            this.showStatus('Configuration saved successfully!', 'success');
        } catch (error) {
            this.showStatus(`Error saving config: ${error}`, 'error');
        } finally {
            this.setLoading(false);
        }
    }

    async sendControl(action) {
        try {
            const payload = { action };

            if (this.linearSpeedInput) {
                payload.linear_speed = parseFloat(this.linearSpeedInput.value);
            }
            if (this.angularSpeedInput) {
                payload.angular_speed = parseFloat(this.angularSpeedInput.value);
            }

            const response = await fetch(`${this.API_BASE}/control`, {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify(payload),
            });

            if (!response.ok) throw new Error('Command failed');

            if (action === 'stop') {
                this.showStatus('Stopped', 'info');
            } else {
                this.showStatus(`Running: ${action.replace('_', ' ').toUpperCase()}`, 'success');
            }
        } catch (error) {
            this.showStatus(`Error executing command: ${error}`, 'error');
        }
    }

    async pollBattery() {
        try {
            const response = await fetch(`${this.API_BASE}/battery`);
            if (!response.ok) throw new Error('Failed to fetch battery');

            const data = await response.json();
            const voltage = data.voltage;
            if (typeof voltage === 'number' && Number.isFinite(voltage)) {
                this.batteryVoltageDisplay.textContent = `${voltage.toFixed(2)} V`;
                const percentage = Math.max(0, Math.min(100, ((voltage - 10.0) / (12.6 - 10.0)) * 100));
                this.batteryLevelBar.style.width = `${percentage}%`;

                if (percentage < 20) {
                    this.batteryLevelBar.className = 'h-full rounded-full bg-rose-500 transition-all duration-500';
                    this.batteryStatusText.textContent = 'Low battery';
                    this.batteryStatusText.className = 'text-sm font-medium text-rose-600';
                    this.updateHeroBadge('battery-hero-badge', `${voltage.toFixed(2)} V`, 'text-rose-200');
                } else if (percentage < 50) {
                    this.batteryLevelBar.className = 'h-full rounded-full bg-amber-500 transition-all duration-500';
                    this.batteryStatusText.textContent = 'Medium';
                    this.batteryStatusText.className = 'text-sm font-medium text-amber-600';
                    this.updateHeroBadge('battery-hero-badge', `${voltage.toFixed(2)} V`, 'text-amber-200');
                } else {
                    this.batteryLevelBar.className = 'h-full rounded-full bg-emerald-500 transition-all duration-500';
                    this.batteryStatusText.textContent = 'Good';
                    this.batteryStatusText.className = 'text-sm font-medium text-emerald-600';
                    this.updateHeroBadge('battery-hero-badge', `${voltage.toFixed(2)} V`, 'text-white');
                }
            }
        } catch (error) {
            this.batteryVoltageDisplay.textContent = '-- V';
            this.batteryStatusText.textContent = 'Error';
            this.batteryStatusText.className = 'text-sm font-medium text-rose-600';
            this.updateHeroBadge('battery-hero-badge', '-- V', 'text-slate-200');
        }
    }

    async pollSpeed() {
        try {
            const response = await fetch(`${this.API_BASE}/chassis/speed`);
            if (!response.ok) throw new Error('Failed to fetch speed');

            const data = await response.json();
            if (typeof data.left_speed === 'number') {
                this.leftSpeedDisplay.textContent = `${data.left_speed.toFixed(3)} m/s`;
                this.updateHeroBadge('left-speed-hero-badge', `${data.left_speed.toFixed(3)} m/s`, 'text-white');
            }
            if (typeof data.right_speed === 'number') {
                this.rightSpeedDisplay.textContent = `${data.right_speed.toFixed(3)} m/s`;
                this.updateHeroBadge('right-speed-hero-badge', `${data.right_speed.toFixed(3)} m/s`, 'text-white');
            }
        } catch (error) {
            this.leftSpeedDisplay.textContent = '-- m/s';
            this.rightSpeedDisplay.textContent = '-- m/s';
            this.updateHeroBadge('left-speed-hero-badge', '-- m/s', 'text-slate-200');
            this.updateHeroBadge('right-speed-hero-badge', '-- m/s', 'text-slate-200');
        }
    }

    async pollIMU() {
        try {
            const response = await fetch(`${this.API_BASE}/imu`);
            if (!response.ok) throw new Error('Failed to fetch IMU');

            const data = await response.json();
            const hasData = data && data.orientation && data.angular_velocity && data.linear_acceleration;

            this.imuConnEl.textContent = `connection: ${hasData ? 'connected' : 'disconnected'}`;

            if (data && typeof data.timestamp === 'number') {
                this.imuTimeEl.textContent = `timestamp: ${new Date(data.timestamp * 1000).toLocaleString()}`;
            } else {
                this.imuTimeEl.textContent = 'timestamp: --';
            }

            this.imuCalibEl.textContent = this.formatValue(data && data.calibration);
            this.imuOrientEl.textContent = this.formatValue(data && data.orientation);
            this.imuGyroEl.textContent = this.formatValue(data && data.angular_velocity);
            this.imuAccelEl.textContent = this.formatValue(data && data.linear_acceleration);
        } catch (error) {
            this.imuConnEl.textContent = 'connection: disconnected';
            this.imuTimeEl.textContent = 'timestamp: --';
            this.imuCalibEl.textContent = '--';
            this.imuOrientEl.textContent = '--';
            this.imuGyroEl.textContent = '--';
            this.imuAccelEl.textContent = '--';
        }
    }

    async pollLidar() {
        try {
            const response = await fetch(`${this.API_BASE}/lidar`);
            if (!response.ok) throw new Error('Failed to fetch LiDAR');

            const data = await response.json();
            this.drawLidar(data);
        } catch (error) {
            if (this.lidarCtx && this.lidarCanvas) {
                this.drawLidar(null);
            }
        }
    }

    drawLidar(data) {
        if (!this.lidarCtx || !this.lidarCanvas) return;

        const ctx = this.lidarCtx;
        const width = this.lidarCanvas.width;
        const height = this.lidarCanvas.height;
        const cx = width / 2;
        const cy = height / 2;
        const scale = Math.min(width, height) * 0.34;

        ctx.fillStyle = '#020617';
        ctx.fillRect(0, 0, width, height);

        ctx.strokeStyle = 'rgba(148, 163, 184, 0.12)';
        ctx.lineWidth = 1;
        ctx.beginPath();
        for (let i = 0; i < width; i += width / 8) {
            ctx.moveTo(i, 0);
            ctx.lineTo(i, height);
            ctx.moveTo(0, i);
            ctx.lineTo(width, i);
        }
        ctx.stroke();

        ctx.strokeStyle = 'rgba(45, 212, 191, 0.22)';
        ctx.beginPath();
        ctx.arc(cx, cy, scale, 0, Math.PI * 2);
        ctx.stroke();
        ctx.beginPath();
        ctx.arc(cx, cy, scale * 0.5, 0, Math.PI * 2);
        ctx.stroke();

        ctx.fillStyle = '#ef4444';
        ctx.beginPath();
        ctx.arc(cx, cy, 5, 0, Math.PI * 2);
        ctx.fill();

        ctx.fillStyle = '#22c55e';

        const ranges = data && Array.isArray(data.ranges) ? data.ranges : [];
        if (!ranges.length) {
            ctx.fillStyle = 'rgba(226, 232, 240, 0.9)';
            ctx.font = '600 20px sans-serif';
            ctx.textAlign = 'center';
            ctx.fillText('Waiting for scan...', cx, cy - 4);
            ctx.font = '400 14px sans-serif';
            ctx.fillText('LiDAR data is not available yet', cx, cy + 22);
            if (this.lidarStateEl) this.lidarStateEl.textContent = 'No scan data yet. Check the driver or wait for the next frame.';
            return;
        }

        const angleMin = typeof data.angle_min === 'number' ? data.angle_min : 0;
        const angleMax = typeof data.angle_max === 'number' ? data.angle_max : Math.PI * 2;
        const rangeMin = typeof data.range_min === 'number' ? data.range_min : 0;
        const rangeMax = typeof data.range_max === 'number' ? data.range_max : 10;
        const angleIncrement = ranges.length > 1 ? (angleMax - angleMin) / ranges.length : 0;

        for (let index = 0; index < ranges.length; index += 1) {
            const dist = ranges[index];
            if (typeof dist !== 'number' || dist < rangeMin || dist > rangeMax) continue;

            const angle = angleMin + (index * angleIncrement);
            const x = cx + Math.cos(angle) * dist * scale;
            const y = cy - Math.sin(angle) * dist * scale;

            const alpha = Math.max(0.2, 1 - dist / rangeMax);
            ctx.fillStyle = `rgba(34, 197, 94, ${alpha})`;
            ctx.fillRect(x, y, 2.5, 2.5);
        }

        if (this.lidarStateEl) {
            const sampleCount = ranges.length;
            this.lidarStateEl.textContent = `Samples: ${sampleCount} | range: ${rangeMin.toFixed ? rangeMin.toFixed(2) : rangeMin} - ${rangeMax.toFixed ? rangeMax.toFixed(2) : rangeMax} m`;
        }
    }

    formatValue(value, depth = 0) {
        const indent = '  '.repeat(depth);

        if (value === null || value === undefined) {
            return `${indent}--`;
        }

        if (typeof value === 'number') {
            return `${indent}${Number.isFinite(value) ? value.toFixed(3) : '--'}`;
        }

        if (typeof value === 'string' || typeof value === 'boolean') {
            return `${indent}${String(value)}`;
        }

        if (Array.isArray(value)) {
            if (!value.length) return `${indent}[]`;
            return value.map((item) => this.formatValue(item, depth + 1)).join('\n');
        }

        if (typeof value === 'object') {
            const entries = Object.entries(value);
            if (!entries.length) return `${indent}{}`;

            return entries.map(([key, nestedValue]) => {
                const nested = this.formatValue(nestedValue, depth + 1);
                if (nested.includes('\n')) {
                    return `${indent}${key}:\n${nested}`;
                }
                return `${indent}${key}: ${nested.trimStart()}`;
            }).join('\n');
        }

        return `${indent}${String(value)}`;
    }

    updateHeroBadge(id, value, textClass) {
        const target = document.getElementById(id);
        if (!target) return;

        target.textContent = value;
        target.classList.remove('text-white', 'text-slate-200', 'text-amber-200', 'text-rose-200');
        target.classList.add(textClass);
    }

    showStatus(message, type = 'info') {
        if (!this.statusDiv) return;

        this.statusDiv.textContent = message;
        this.statusDiv.className = 'rounded-2xl border px-4 py-3 text-sm font-medium transition-opacity duration-300 opacity-100';

        if (type === 'success') {
            this.statusDiv.classList.add('border-emerald-200', 'bg-emerald-50', 'text-emerald-700');
        } else if (type === 'error') {
            this.statusDiv.classList.add('border-rose-200', 'bg-rose-50', 'text-rose-700');
        } else {
            this.statusDiv.classList.add('border-slate-200', 'bg-slate-100', 'text-slate-700');
        }

        clearTimeout(this.statusTimer);
        if (type !== 'error') {
            this.statusTimer = setTimeout(() => {
                this.statusDiv.classList.remove('opacity-100');
                this.statusDiv.classList.add('opacity-0');
            }, 3000);
        }
    }

    setLoading(isLoading) {
        if (!this.saveBtn) return;

        if (isLoading) {
            this.saveBtn.disabled = true;
            this.saveBtn.textContent = 'Saving...';
        } else {
            this.saveBtn.disabled = false;
            this.saveBtn.textContent = 'Save configuration';
        }
    }
}

document.addEventListener('DOMContentLoaded', () => {
    new App();
});
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
        const speedMonitorSection = this.buildSpeedMonitorSection();
        const controlSettingsSection = this.buildControlSettingsSection();
        const speedSection = this.buildSpeedSection();
        const controlsSection = this.buildControlsSection();
        
        this.contentArea.appendChild(DOMBuilder.el('div', 'space-y-8', [
            batterySection,
            speedMonitorSection,
            controlSettingsSection,
            speedSection,
            controlsSection
        ]));

        this.bindChassisEvents();
        this.fetchChassisConfig();
        
        // Poll battery every 2 seconds
        this.pollBattery();
        this.pollInterval = setInterval(() => this.pollBattery(), 2000);

        // Poll speed every 200ms
        this.pollSpeed();
        this.speedPollInterval = setInterval(() => this.pollSpeed(), 200);
    }

    buildSpeedMonitorSection() {
        this.leftSpeedDisplay = DOMBuilder.el('div', 'text-2xl font-bold text-gray-800', ['-- m/s']);
        this.rightSpeedDisplay = DOMBuilder.el('div', 'text-2xl font-bold text-gray-800', ['-- m/s']);

        const leftBox = DOMBuilder.el('div', 'bg-white rounded-lg p-4 border border-gray-200 shadow-sm flex flex-col items-center flex-1', [
            DOMBuilder.el('h4', 'text-sm font-semibold text-gray-500 mb-1', ['Left Wheel Speed']),
            this.leftSpeedDisplay
        ]);

        const rightBox = DOMBuilder.el('div', 'bg-white rounded-lg p-4 border border-gray-200 shadow-sm flex flex-col items-center flex-1', [
            DOMBuilder.el('h4', 'text-sm font-semibold text-gray-500 mb-1', ['Right Wheel Speed']),
            this.rightSpeedDisplay
        ]);

        return DOMBuilder.el('div', 'flex gap-4', [leftBox, rightBox]);
    }

    async pollSpeed() {
        try {
            const response = await fetch(`${this.API_BASE}/chassis/speed`);
            if (!response.ok) throw new Error('Failed to fetch speed');
            const data = await response.json();
            
            if (this.leftSpeedDisplay) this.leftSpeedDisplay.textContent = `${data.left_speed.toFixed(3)} m/s`;
            if (this.rightSpeedDisplay) this.rightSpeedDisplay.textContent = `${data.right_speed.toFixed(3)} m/s`;
        } catch (e) {
            // console.error(e);
            if (this.leftSpeedDisplay) this.leftSpeedDisplay.textContent = '-- m/s';
            if (this.rightSpeedDisplay) this.rightSpeedDisplay.textContent = '-- m/s';
        }
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

        const statusCard = DOMBuilder.el('div', 'bg-gray-50 p-4 rounded-lg border', [
            DOMBuilder.el('h3', 'text-sm font-semibold text-gray-500 uppercase mb-2', ['Status']),
            DOMBuilder.el('div', 'font-mono text-sm space-y-1', [
                DOMBuilder.el('div', '', ['connection: --'], { id: 'imu-conn' }),
                DOMBuilder.el('div', '', ['timestamp: --'], { id: 'imu-time' })
            ])
        ]);

        const calibrationCard = DOMBuilder.el('div', 'bg-gray-50 p-4 rounded-lg border', [
            DOMBuilder.el('h3', 'text-sm font-semibold text-gray-500 uppercase mb-2', ['Calibration (0-3)']),
            DOMBuilder.el('div', 'font-mono text-sm space-y-1', [], { id: 'imu-calib' })
        ]);

        const orientationCard = createCard('Orientation (Quaternion)', 'imu-orient');
        const gyroCard = createCard('Angular Velocity (rad/s)', 'imu-gyro');
        const accelCard = createCard('Linear Acceleration (m/s²)', 'imu-accel');

        this.contentArea.appendChild(DOMBuilder.el('div', 'grid grid-cols-1 md:grid-cols-3 gap-4', [
            statusCard, calibrationCard, orientationCard, gyroCard, accelCard
        ]));

        this.pollInterval = setInterval(() => this.pollIMU(), 200); // 5Hz
    }

    async pollIMU() {
        try {
            const response = await fetch(`${this.API_BASE}/imu`);
            if (!response.ok) throw new Error('Failed to fetch IMU');
            const data = await response.json();

            const format = (obj) => {
                if (!obj || typeof obj !== 'object') {
                    return '--';
                }
                return Object.entries(obj)
                    .map(([k, v]) => {
                        if (typeof v === 'number' && Number.isFinite(v)) {
                            return `${k}: ${v.toFixed(3)}`;
                        }
                        return `${k}: --`;
                    })
                    .join('<br>');
            };

            const connEl = document.getElementById('imu-conn');
            const timeEl = document.getElementById('imu-time');
            const calibEl = document.getElementById('imu-calib');
            const orientEl = document.getElementById('imu-orient');
            const gyroEl = document.getElementById('imu-gyro');
            const accelEl = document.getElementById('imu-accel');

            const hasData = data && data.orientation && data.angular_velocity && data.linear_acceleration;
            if (connEl) connEl.textContent = `connection: ${hasData ? 'connected' : 'disconnected'}`;
            if (timeEl) {
                if (data && typeof data.timestamp === 'number') {
                    timeEl.textContent = `timestamp: ${new Date(data.timestamp * 1000).toLocaleString()}`;
                } else {
                    timeEl.textContent = 'timestamp: --';
                }
            }
            if (calibEl) calibEl.innerHTML = format(data && data.calibration);
            if (orientEl) orientEl.innerHTML = format(data && data.orientation);
            if (gyroEl) gyroEl.innerHTML = format(data && data.angular_velocity);
            if (accelEl) accelEl.innerHTML = format(data && data.linear_acceleration);

        } catch (e) {
            const connEl = document.getElementById('imu-conn');
            const timeEl = document.getElementById('imu-time');
            const calibEl = document.getElementById('imu-calib');
            const orientEl = document.getElementById('imu-orient');
            const gyroEl = document.getElementById('imu-gyro');
            const accelEl = document.getElementById('imu-accel');
            if (connEl) connEl.textContent = 'connection: disconnected';
            if (timeEl) timeEl.textContent = 'timestamp: --';
            if (calibEl) calibEl.innerHTML = '--';
            if (orientEl) orientEl.innerHTML = '--';
            if (gyroEl) gyroEl.innerHTML = '--';
            if (accelEl) accelEl.innerHTML = '--';
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
