class App {
    constructor() {
        this.canvas = document.getElementById('mainCanvas');
        this.renderer = new CanvasRenderer(this.canvas);
        this.coordinateTransform = new CoordinateTransform(this.canvas.width, this.canvas.height);
        this.frameTransition = new FrameTransition(this.renderer);
        this.playbackController = new PlaybackController(this.renderer, this.frameTransition, this.coordinateTransform);
        this.viewController = new ViewController(this.canvas, this.coordinateTransform, this.renderer);
        
        this.frameNavigator = new FrameNavigator(
            document.getElementById('frameSlider'),
            document.getElementById('startFrameInput'),
            document.getElementById('sliderMarkers')
        );
        
        this.metadata = null;
        this.frameDataMap = {};
        
        this.setupEventListeners();
        this.setupCallbacks();
        
        this.logMessage('应用初始化完成', 'info');
    }
    
    setupEventListeners() {
        document.getElementById('selectFileBtn').addEventListener('click', () => {
            document.getElementById('fileInput').click();
        });
        
        document.getElementById('fileInput').addEventListener('change', (e) => {
            if (e.target.files.length > 0) {
                this.handleFileUpload(e.target.files[0]);
            }
        });
        
        document.getElementById('playBtn').addEventListener('click', () => {
            this.playbackController.play();
            this.updateUI();
        });
        
        document.getElementById('pauseBtn').addEventListener('click', () => {
            this.playbackController.pause();
            this.updateUI();
        });
        
        document.getElementById('forwardBtn').addEventListener('click', () => {
            this.playbackController.stepForward();
            this.updateUI();
        });
        
        document.getElementById('backwardBtn').addEventListener('click', () => {
            this.playbackController.stepBackward();
            this.updateUI();
        });
        
        document.getElementById('speedSelect').addEventListener('change', (e) => {
            const speed = parseFloat(e.target.value);
            this.playbackController.setSpeed(speed);
            this.logMessage(`播放速度设置为 ${speed}x`, 'info');
        });
        
        document.getElementById('loopCheckbox').addEventListener('change', (e) => {
            const enabled = this.playbackController.toggleLoop();
            this.logMessage(`循环播放 ${enabled ? '开启' : '关闭'}`, 'info');
        });
        
        document.getElementById('showIdCheckbox').addEventListener('change', (e) => {
            this.renderer.showIds = e.target.checked;
            this.playbackController.immediateRenderCurrentFrame();
            this.logMessage(`多边形ID显示 ${e.target.checked ? '开启' : '关闭'}`, 'info');
        });
        
        document.getElementById('themeSelect').addEventListener('change', (e) => {
            const theme = e.target.value;
            VisualStyles.setTheme(theme);
            this.applyTheme(theme);
            this.playbackController.immediateRenderCurrentFrame();
            this.logMessage(`主题切换为 ${theme === 'dark' ? '暗色' : '白色'}`, 'info');
        });
        
        document.getElementById('clearLogBtn').addEventListener('click', () => {
            document.getElementById('logContainer').innerHTML = '<div class="log-placeholder">暂无日志信息</div>';
        });
    }
    
    setupCallbacks() {
        this.playbackController.onFrameChange = (frameId) => {
            this.frameNavigator.setCurrentFrame(frameId);
            this.updateFrameDisplay(frameId);
        };
        
        this.frameNavigator.onSliderDrag = (frameId) => {
            this.playbackController.jumpToFrame(frameId);
            this.updateFrameDisplay(frameId);
        };
        
        this.frameNavigator.onSliderRelease = (frameId) => {
            this.playbackController.jumpToFrame(frameId);
            this.updateFrameDisplay(frameId);
        };
        
        this.frameNavigator.onStartFrameInput = (frameId) => {
            if (this.frameNavigator.validateFrameId(frameId)) {
                this.playbackController.jumpToFrame(frameId);
                this.playbackController.pause();
                this.updateUI();
                this.logMessage(`跳转到开始帧: ${frameId}`, 'info');
            } else {
                this.logMessage(`无效的开始帧: ${frameId}`, 'error');
            }
        };
        
        this.viewController.onRender = () => {
            this.playbackController.immediateRenderCurrentFrame();
        };
    }
    
    async handleFileUpload(file) {
        this.showLoading(true);
        this.logMessage(`上传文件: ${file.name}`, 'info');
        
        try {
            const formData = new FormData();
            formData.append('file', file);
            
            const response = await fetch('/upload', {
                method: 'POST',
                body: formData
            });
            
            const result = await response.json();
            
            if (result.error) {
                throw new Error(result.error);
            }
            
            document.getElementById('filename').textContent = file.name;
            this.metadata = result.metadata;
            
            this.logMessage(`文件解析成功`, 'info');
            this.logMessage(`多边形数量: ${this.metadata.polygon_count}`, 'info');
            this.logMessage(`快照帧数: ${this.metadata.snapshot_count}`, 'info');
            this.logMessage(`可行域顶点: ${this.metadata.feasible_region_vertices}`, 'info');
            
            await this.loadMetadata();
            await this.loadAllFrames();
            
            this.playbackController.setTotalFrames(this.metadata.snapshot_count);
            this.frameNavigator.setTotalFrames(this.metadata.snapshot_count);
            
            this.playbackController.jumpToFrame(0);
            this.updateUI();
            
        } catch (error) {
            this.logMessage(`错误: ${error.message}`, 'error');
            this.showError(error.message);
        } finally {
            this.showLoading(false);
        }
    }
    
    async loadMetadata() {
        try {
            const response = await fetch('/metadata');
            const data = await response.json();
            
            if (data.error) {
                throw new Error(data.error);
            }
            
            this.frameNavigator.setIllegalFrames(data.illegal_frames);
            
            if (data.illegal_frames.length > 0) {
                this.logMessage(`非法帧数量: ${data.illegal_frames.length}`, 'warning');
            }
            
        } catch (error) {
            this.logMessage(`加载元信息失败: ${error.message}`, 'error');
        }
    }
    
    async loadAllFrames() {
        this.frameDataMap = {};
        
        const firstFrameResponse = await fetch('/frame/0');
        const firstFrameData = await firstFrameResponse.json();
        
        if (firstFrameData.error) {
            throw new Error(firstFrameData.error);
        }
        
        this.frameDataMap[0] = firstFrameData;
        
        this.playbackController.setFrameData(this.frameDataMap);
        
        this.coordinateTransform.computeAutoFitZoom(
            firstFrameData.polygon_states,
            firstFrameData.feasible_region
        );
        
        this.logMessage(`首帧加载完成，共 ${this.metadata.snapshot_count} 帧`, 'info');
        
        this.startBackgroundLoading();
    }
    
    startBackgroundLoading() {
        const batchSize = 10;
        let currentBatch = 1;
        
        const loadBatch = async () => {
            const endFrame = Math.min(currentBatch * batchSize, this.metadata.snapshot_count);
            
            for (let i = (currentBatch - 1) * batchSize; i < endFrame; i++) {
                if (this.frameDataMap[i]) continue;
                
                try {
                    const response = await fetch(`/frame/${i}`);
                    const data = await response.json();
                    
                    if (!data.error) {
                        this.frameDataMap[i] = data;
                    }
                } catch (error) {
                }
            }
            
            currentBatch++;
            
            if (currentBatch * batchSize < this.metadata.snapshot_count + batchSize) {
                setTimeout(loadBatch, 100);
            } else {
                this.logMessage(`所有帧加载完成`, 'info');
            }
        };
        
        setTimeout(loadBatch, 100);
    }
    
    updateUI() {
        const frameDisplay = document.getElementById('frameDisplay');
        const current = this.playbackController.currentFrame;
        const total = this.playbackController.totalFrames;
        frameDisplay.textContent = `当前帧: ${current} / ${total}`;
        
        this.frameNavigator.setCurrentFrame(current);
    }
    
    updateFrameDisplay(frameId) {
        const frameDisplay = document.getElementById('frameDisplay');
        const total = this.playbackController.totalFrames;
        frameDisplay.textContent = `当前帧: ${frameId} / ${total}`;
        
        if (this.frameDataMap[frameId]) {
            const data = this.frameDataMap[frameId];
            
            const legalCountEl = document.getElementById('legalPolygonCount');
            if (legalCountEl) {
                const totalPolygons = data.legal_count + data.illegal_count;
                legalCountEl.textContent = `${data.legal_count} / ${totalPolygons}`;
            }
            
            const displacementEl = document.getElementById('totalDisplacement');
            if (displacementEl && data.polygon_states) {
                let totalL = 0;
                for (const polygon of data.polygon_states) {
                    if (polygon.translation) {
                        const dx = polygon.translation[0];
                        const dy = polygon.translation[1];
                        const dist = Math.sqrt(dx * dx + dy * dy);
                        totalL += dist;
                    }
                }
                displacementEl.textContent = totalL.toFixed(3);
            }
            
            if (data.illegal_count > 0) {
                this.logMessage(
                    `帧 ${frameId}: 合法 ${data.legal_count}, 非法 ${data.illegal_count} (多边形 ${data.illegal_polygon_ids.join(', ')})`,
                    'warning'
                );
            }
        }
    }
    
    logMessage(message, level = 'info') {
        const logContainer = document.getElementById('logContainer');
        
        const placeholder = logContainer.querySelector('.log-placeholder');
        if (placeholder) {
            logContainer.innerHTML = '';
        }
        
        const entry = document.createElement('div');
        entry.className = `log-entry ${level}`;
        
        const timestamp = new Date().toLocaleTimeString();
        entry.textContent = `[${timestamp}] ${message}`;
        
        logContainer.appendChild(entry);
        logContainer.scrollTop = logContainer.scrollHeight;
    }
    
    applyTheme(theme) {
        const colors = VisualStyles.COLORS;
        
        document.documentElement.style.setProperty('--bg-primary', colors.background);
        document.documentElement.style.setProperty('--bg-secondary', colors.secondary);
        document.documentElement.style.setProperty('--color-accent', colors.accent);
        document.documentElement.style.setProperty('--color-text', colors.text);
        document.documentElement.style.setProperty('--color-text-secondary', colors.textSecondary);
        document.documentElement.style.setProperty('--color-border', colors.border);
        document.documentElement.style.setProperty('--color-grid', colors.grid);
        document.documentElement.style.setProperty('--color-illegal', colors.illegal);
        document.documentElement.style.setProperty('--color-feasible', colors.feasible);
        
        document.body.style.backgroundColor = colors.background;
        document.body.style.color = colors.text;
        
        const drawingArea = document.querySelector('.drawing-area');
        if (drawingArea) {
            drawingArea.style.backgroundColor = colors.background;
        }
        
        const controlPanel = document.querySelector('.control-panel');
        if (controlPanel) {
            controlPanel.style.backgroundColor = colors.secondary;
        }
        
        const frameInfo = document.getElementById('frameInfo');
        if (frameInfo) {
            frameInfo.style.backgroundColor = `${colors.background}cc`;
            frameInfo.style.borderColor = colors.border;
        }
        
        const buttons = document.querySelectorAll('.btn');
        buttons.forEach(btn => {
            btn.style.backgroundColor = colors.secondary;
            btn.style.borderColor = colors.accent;
        });
        
        const inputs = document.querySelectorAll('.select-input, .number-input');
        inputs.forEach(input => {
            input.style.backgroundColor = colors.background;
            input.style.color = colors.text;
            input.style.borderColor = colors.border;
        });
        
        const sections = document.querySelectorAll('.panel-section');
        sections.forEach(section => {
            section.style.borderColor = colors.border;
        });
        
        const headings = document.querySelectorAll('.panel-section h3');
        headings.forEach(h => {
            h.style.color = colors.accent;
        });
        
        const logContainer = document.getElementById('logContainer');
        if (logContainer) {
            logContainer.style.backgroundColor = colors.background;
            logContainer.style.borderColor = colors.border;
        }
        
        const loadingOverlay = document.getElementById('loadingOverlay');
        if (loadingOverlay) {
            loadingOverlay.style.backgroundColor = `${colors.background}e6`;
        }
    }
    
    showError(message) {
        this.logMessage(`错误: ${message}`, 'error');
    }
    
    showLoading(show) {
        const overlay = document.getElementById('loadingOverlay');
        if (show) {
            overlay.classList.add('active');
        } else {
            overlay.classList.remove('active');
        }
    }
}

let app;

document.addEventListener('DOMContentLoaded', () => {
    app = new App();
});
