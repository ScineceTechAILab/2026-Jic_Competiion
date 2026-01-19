
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

class ChassisController {
    constructor() {
        this.API_BASE = '/api';
        
        this.currentConfig = {
            left_scale: 1.0,
            right_scale: 1.0
        };

        const root = document.getElementById('app');
        if (!root) throw new Error('Root element #app not found');
        this.appRoot = root;

        this.render();
        this.init();
    }

    render() {
        // Build Header
        const header = DOMBuilder.el('div', 'bg-slate-800 p-6 rounded-t-xl', [
            DOMBuilder.el('h1', 'text-2xl font-bold text-white text-center', ['Chassis Speed Tuning']),
            DOMBuilder.el('p', 'text-gray-400 text-center text-sm mt-1', ['Fine-tune differential drive motor speeds'])
        ]);

        // Build Sections
        const speedSection = this.buildSpeedSection();
        const controlsSection = this.buildControlsSection();

        // Build Status Bar
        this.statusDiv = DOMBuilder.el('div', 'text-center p-3 rounded-lg text-sm font-medium text-gray-500 bg-gray-100 transition-all duration-300 opacity-0', ['Ready']);

        // Main Container
        const content = DOMBuilder.el('div', 'bg-white rounded-xl shadow-lg overflow-hidden', [
            header,
            DOMBuilder.el('div', 'p-8 space-y-8', [
                speedSection,
                controlsSection,
                this.statusDiv
            ])
        ]);

        this.appRoot.appendChild(content);
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
        // CW Button
        this.cwBtn = DOMBuilder.el('button', 'bg-green-600 hover:bg-green-700 active:bg-green-800 text-white font-medium py-3 px-6 rounded-lg shadow transition-all transform active:scale-95 flex items-center gap-2 select-none', [
            'Rotate CW (Right)'
        ]);

        // CCW Button
        this.ccwBtn = DOMBuilder.el('button', 'bg-green-600 hover:bg-green-700 active:bg-green-800 text-white font-medium py-3 px-6 rounded-lg shadow transition-all transform active:scale-95 flex items-center gap-2 select-none', [
            'Rotate CCW (Left)'
        ]);

        // Stop Button
        this.stopBtn = DOMBuilder.el('button', 'bg-red-500 hover:bg-red-600 active:bg-red-700 text-white font-medium py-3 px-8 rounded-lg shadow transition-all transform active:scale-95 flex items-center gap-2', [
            'STOP'
        ]);

        return DOMBuilder.el('div', 'bg-gray-50 rounded-lg p-6 border border-gray-200', [
            DOMBuilder.el('h3', 'text-lg font-semibold text-gray-700 mb-4 border-b pb-2', ['Test Controls']),
            DOMBuilder.el('p', 'text-sm text-gray-600 mb-6', ['Press and hold the buttons below to rotate the chassis.']),
            DOMBuilder.el('div', 'flex flex-wrap gap-4 justify-center', [this.cwBtn, this.ccwBtn, this.stopBtn])
        ]);
    }

    init() {
        this.bindEvents();
        this.fetchConfig();
    }

    bindEvents() {
        this.leftScaleInput.addEventListener('input', (e) => {
            this.leftValDisplay.textContent = parseFloat(e.target.value).toFixed(2);
        });

        this.rightScaleInput.addEventListener('input', (e) => {
            this.rightValDisplay.textContent = parseFloat(e.target.value).toFixed(2);
        });

        this.saveBtn.addEventListener('click', () => this.saveConfig());

        this.bindPressHold(this.cwBtn, 'rotate_cw');
        this.bindPressHold(this.ccwBtn, 'rotate_ccw');
        this.stopBtn.addEventListener('click', () => this.sendControl('stop'));
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

    async fetchConfig() {
        try {
            const response = await fetch(`${this.API_BASE}/config`);
            if (!response.ok) throw new Error('Failed to fetch config');
            
            const data = await response.json();
            this.currentConfig = data;
            this.updateUI();
            this.showStatus('Configuration loaded', 'success');
        } catch (e) {
            this.showStatus(`Error loading config: ${e}`, 'error');
        }
    }

    updateUI() {
        this.leftScaleInput.value = this.currentConfig.left_scale;
        this.rightScaleInput.value = this.currentConfig.right_scale;
        this.leftValDisplay.textContent = parseFloat(this.currentConfig.left_scale).toFixed(2);
        this.rightValDisplay.textContent = parseFloat(this.currentConfig.right_scale).toFixed(2);
    }

    async saveConfig() {
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
            const response = await fetch(`${this.API_BASE}/control`, {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ action })
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

    showStatus(msg, type = 'info') {
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
    new ChassisController();
});

// TODO:增加IMU测试页
// TODO:增加雷达测试页