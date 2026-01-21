// Interfaces
interface Config {
    left_scale: number;
    right_scale: number;
}

interface ApiResponse {
    status: string;
    config?: Config;
    action?: string;
    detail?: string;
}

type MotorAction = 'rotate_cw' | 'rotate_ccw' | 'stop';

class DOMBuilder {
    static el(tag: string, classes: string = '', children: (HTMLElement | string)[] = [], props: any = {}): HTMLElement {
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
                Object.entries(value as object).forEach(([k, v]) => element.dataset[k] = v);
            } else {
                (element as any)[key] = value;
            }
        });

        return element;
    }
}

class ChassisController {
    private readonly API_BASE = '/api';
    
    // UI References
    private appRoot: HTMLElement;
    private leftScaleInput!: HTMLInputElement;
    private rightScaleInput!: HTMLInputElement;
    private leftValDisplay!: HTMLElement;
    private rightValDisplay!: HTMLElement;
    private saveBtn!: HTMLButtonElement;
    private cwBtn!: HTMLButtonElement;
    private ccwBtn!: HTMLButtonElement;
    private stopBtn!: HTMLButtonElement;
    private statusDiv!: HTMLElement;

    // State
    private currentConfig: Config = {
        left_scale: 1.0,
        right_scale: 1.0
    };

    constructor() {
        const root = document.getElementById('app');
        if (!root) throw new Error('Root element #app not found');
        this.appRoot = root;

        this.render();
        this.init();
    }

    private render(): void {
        // Build Header
        const header = DOMBuilder.el('div', 'bg-slate-800 p-6 rounded-t-xl', [
            DOMBuilder.el('h1', 'text-2xl font-bold text-white text-center', ['Chassis Speed Tuning']),
            DOMBuilder.el('p', 'text-gray-400 text-center text-sm mt-1', ['Fine-tune differential drive motor speeds'])
        ]);

        // Build Speed Correction Section
        const speedSection = this.buildSpeedSection();
        
        // Build Test Controls Section
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

    private buildSpeedSection(): HTMLElement {
        // Left Motor Input
        this.leftValDisplay = DOMBuilder.el('span', 'bg-blue-100 text-blue-800 text-xs font-mono px-2 py-1 rounded', ['1.00']);
        this.leftScaleInput = DOMBuilder.el('input', 'w-full h-2 bg-gray-200 rounded-lg appearance-none cursor-pointer accent-blue-600 hover:accent-blue-700 transition-all', [], {
            type: 'range', min: '0.5', max: '1.5', step: '0.01', value: '1.0'
        }) as HTMLInputElement;

        const leftControl = DOMBuilder.el('div', '', [
            DOMBuilder.el('div', 'flex justify-between items-center mb-2', [
                DOMBuilder.el('label', 'text-sm font-medium text-gray-700', ['Left Motor Scale (Left Wheel)']),
                this.leftValDisplay
            ]),
            this.leftScaleInput
        ]);

        // Right Motor Input
        this.rightValDisplay = DOMBuilder.el('span', 'bg-blue-100 text-blue-800 text-xs font-mono px-2 py-1 rounded', ['1.00']);
        this.rightScaleInput = DOMBuilder.el('input', 'w-full h-2 bg-gray-200 rounded-lg appearance-none cursor-pointer accent-blue-600 hover:accent-blue-700 transition-all', [], {
            type: 'range', min: '0.5', max: '1.5', step: '0.01', value: '1.0'
        }) as HTMLInputElement;

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
        ]) as HTMLButtonElement;

        return DOMBuilder.el('div', 'bg-gray-50 rounded-lg p-6 border border-gray-200', [
            DOMBuilder.el('h3', 'text-lg font-semibold text-gray-700 mb-4 border-b pb-2', ['Speed Correction Factors']),
            DOMBuilder.el('div', 'space-y-6', [leftControl, rightControl]),
            DOMBuilder.el('div', 'mt-6 flex justify-center', [this.saveBtn])
        ]);
    }

    private buildControlsSection(): HTMLElement {
        // CW Button
        this.cwBtn = DOMBuilder.el('button', 'bg-green-600 hover:bg-green-700 active:bg-green-800 text-white font-medium py-3 px-6 rounded-lg shadow transition-all transform active:scale-95 flex items-center gap-2 select-none', [
            'Rotate CW (Right)'
        ]) as HTMLButtonElement;

        // CCW Button
        this.ccwBtn = DOMBuilder.el('button', 'bg-green-600 hover:bg-green-700 active:bg-green-800 text-white font-medium py-3 px-6 rounded-lg shadow transition-all transform active:scale-95 flex items-center gap-2 select-none', [
            'Rotate CCW (Left)'
        ]) as HTMLButtonElement;

        // Stop Button
        this.stopBtn = DOMBuilder.el('button', 'bg-red-500 hover:bg-red-600 active:bg-red-700 text-white font-medium py-3 px-8 rounded-lg shadow transition-all transform active:scale-95 flex items-center gap-2', [
            'STOP'
        ]) as HTMLButtonElement;

        return DOMBuilder.el('div', 'bg-gray-50 rounded-lg p-6 border border-gray-200', [
            DOMBuilder.el('h3', 'text-lg font-semibold text-gray-700 mb-4 border-b pb-2', ['Test Controls']),
            DOMBuilder.el('p', 'text-sm text-gray-600 mb-6', ['Press and hold the buttons below to rotate the chassis.']),
            DOMBuilder.el('div', 'flex flex-wrap gap-4 justify-center', [this.cwBtn, this.ccwBtn, this.stopBtn])
        ]);
    }

    private init(): void {
        this.bindEvents();
        this.fetchConfig();
    }

    private bindEvents(): void {
        this.leftScaleInput.addEventListener('input', (e: Event) => {
            const target = e.target as HTMLInputElement;
            this.leftValDisplay.textContent = parseFloat(target.value).toFixed(2);
        });

        this.rightScaleInput.addEventListener('input', (e: Event) => {
            const target = e.target as HTMLInputElement;
            this.rightValDisplay.textContent = parseFloat(target.value).toFixed(2);
        });

        this.saveBtn.addEventListener('click', () => this.saveConfig());

        this.bindPressHold(this.cwBtn, 'rotate_cw');
        this.bindPressHold(this.ccwBtn, 'rotate_ccw');
        this.stopBtn.addEventListener('click', () => this.sendControl('stop'));
    }

    private bindPressHold(btn: HTMLButtonElement, action: MotorAction): void {
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

    private async fetchConfig(): Promise<void> {
        try {
            const response = await fetch(`${this.API_BASE}/config`);
            if (!response.ok) throw new Error('Failed to fetch config');
            
            const data: Config = await response.json();
            this.currentConfig = data;
            this.updateUI();
            this.showStatus('Configuration loaded', 'success');
        } catch (e) {
            this.showStatus(`Error loading config: ${e}`, 'error');
        }
    }

    private updateUI(): void {
        this.leftScaleInput.value = this.currentConfig.left_scale.toString();
        this.rightScaleInput.value = this.currentConfig.right_scale.toString();
        this.leftValDisplay.textContent = this.currentConfig.left_scale.toFixed(2);
        this.rightValDisplay.textContent = this.currentConfig.right_scale.toFixed(2);
    }

    private async saveConfig(): Promise<void> {
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

    private async sendControl(action: MotorAction): Promise<void> {
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

    private showStatus(msg: string, type: 'info' | 'success' | 'error' = 'info'): void {
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

    private setLoading(isLoading: boolean): void {
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

// Initialize when DOM is ready
document.addEventListener('DOMContentLoaded', () => {
    new ChassisController();
});
