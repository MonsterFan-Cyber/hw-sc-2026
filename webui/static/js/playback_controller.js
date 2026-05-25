class PlaybackController {
    constructor(renderer, frameTransition, coordinateTransform) {
        this.renderer = renderer;
        this.frameTransition = frameTransition;
        this.coordinateTransform = coordinateTransform;
        
        this.isPlaying = false;
        this.speed = 1.0;
        this.loopEnabled = false;
        this.currentFrame = 0;
        this.totalFrames = 0;
        
        this.frameData = null;
        this.animationTimer = null;
        
        this.onFrameChange = null;
    }
    
    setTotalFrames(total) {
        this.totalFrames = total;
        if (total > 0 && this.currentFrame >= total) {
            this.currentFrame = 0;
        }
    }
    
    setFrameData(frameDataMap) {
        this.frameData = frameDataMap;
    }
    
    play() {
        if (this.totalFrames === 0 || !this.frameData) {
            return;
        }
        
        this.isPlaying = true;
        this.scheduleNextFrame();
    }
    
    pause() {
        this.isPlaying = false;
        if (this.animationTimer) {
            clearTimeout(this.animationTimer);
            this.animationTimer = null;
        }
    }
    
    stop() {
        this.pause();
        this.currentFrame = 0;
        this.renderCurrentFrame();
    }
    
    stepForward() {
        if (this.totalFrames === 0) {
            return false;
        }
        
        const nextFrame = this.currentFrame + 1;
        
        if (nextFrame >= this.totalFrames) {
            if (this.loopEnabled) {
                this.currentFrame = 0;
            } else {
                return false;
            }
        } else {
            this.currentFrame = nextFrame;
        }
        
        this.renderCurrentFrame();
        return true;
    }
    
    stepBackward() {
        if (this.totalFrames === 0) {
            return false;
        }
        
        const prevFrame = this.currentFrame - 1;
        
        if (prevFrame < 0) {
            if (this.loopEnabled) {
                this.currentFrame = this.totalFrames - 1;
            } else {
                return false;
            }
        } else {
            this.currentFrame = prevFrame;
        }
        
        this.renderCurrentFrame();
        return true;
    }
    
    jumpToFrame(frameId) {
        if (frameId < 0 || frameId >= this.totalFrames) {
            return false;
        }
        
        this.currentFrame = frameId;
        this.renderCurrentFrame();
        return true;
    }
    
    setSpeed(speed) {
        this.speed = speed;
    }
    
    toggleLoop() {
        this.loopEnabled = !this.loopEnabled;
        return this.loopEnabled;
    }
    
    getFrameInterval() {
        const baseInterval = 100;
        return baseInterval / this.speed;
    }
    
    scheduleNextFrame() {
        if (!this.isPlaying) {
            return;
        }
        
        const interval = this.getFrameInterval();
        this.animationTimer = setTimeout(() => this.onFrameTick(), interval);
    }
    
    onFrameTick() {
        if (!this.isPlaying) {
            return;
        }
        
        const hasNext = this.stepForward();
        
        if (!hasNext) {
            this.pause();
            return;
        }
        
        this.scheduleNextFrame();
    }
    
    renderCurrentFrame() {
        if (!this.frameData) {
            return;
        }
        
        if (!this.frameData[this.currentFrame]) {
            fetch(`/frame/${this.currentFrame}`)
                .then(response => response.json())
                .then(data => {
                    if (!data.error) {
                        this.frameData[this.currentFrame] = data;
                        this.immediateRenderCurrentFrame();
                    }
                })
                .catch(error => {});
            return;
        }
        
        this.immediateRenderCurrentFrame();
    }
    
    immediateRenderCurrentFrame() {
        if (!this.frameData || !this.frameData[this.currentFrame]) {
            return;
        }
        
        const data = this.frameData[this.currentFrame];
        
        this.frameTransition.immediateRender(data, this.coordinateTransform);
        
        this.renderer.renderFrameInfo(this.currentFrame, this.totalFrames);
        
        if (this.onFrameChange) {
            this.onFrameChange(this.currentFrame);
        }
    }
}
