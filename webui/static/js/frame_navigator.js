class FrameNavigator {
    constructor(sliderEl, startFrameInputEl, markersEl) {
        this.slider = sliderEl;
        this.startFrameInput = startFrameInputEl;
        this.markersEl = markersEl;
        
        this.totalFrames = 0;
        this.currentFrame = 0;
        this.illegalFrames = [];
        
        this.onSliderDrag = null;
        this.onSliderRelease = null;
        this.onStartFrameInput = null;
        
        this.setupEventListeners();
    }
    
    setupEventListeners() {
        this.slider.addEventListener('input', (e) => {
            const frameId = parseInt(e.target.value, 10);
            if (this.onSliderDrag) {
                this.onSliderDrag(frameId);
            }
        });
        
        this.slider.addEventListener('change', (e) => {
            const frameId = parseInt(e.target.value, 10);
            if (this.onSliderRelease) {
                this.onSliderRelease(frameId);
            }
        });
        
        this.startFrameInput.addEventListener('change', (e) => {
            const frameId = parseInt(e.target.value, 10);
            if (this.onStartFrameInput) {
                this.onStartFrameInput(frameId);
            }
        });
    }
    
    setTotalFrames(total) {
        this.totalFrames = total;
        this.slider.max = Math.max(0, total - 1);
        this.startFrameInput.max = Math.max(0, total - 1);
    }
    
    setCurrentFrame(frameId) {
        this.currentFrame = frameId;
        this.slider.value = frameId;
    }
    
    setIllegalFrames(illegalFrames) {
        this.illegalFrames = illegalFrames;
        this.updateMarkers();
    }
    
    updateMarkers() {
        this.markersEl.innerHTML = '';
        
        if (this.totalFrames === 0) {
            return;
        }
        
        const width = this.markersEl.offsetWidth;
        
        for (const frameData of this.illegalFrames) {
            const frameId = frameData.frame_id;
            const position = (frameId / (this.totalFrames - 1)) * width;
            
            const marker = document.createElement('div');
            marker.className = 'illegal-marker';
            marker.style.left = `${position}px`;
            marker.title = `帧 ${frameId}: 非法多边形 ${frameData.illegal_polygon_ids.join(', ')}`;
            
            this.markersEl.appendChild(marker);
        }
    }
    
    validateFrameId(frameId) {
        if (isNaN(frameId) || frameId < 0 || frameId >= this.totalFrames) {
            return false;
        }
        return true;
    }
}
