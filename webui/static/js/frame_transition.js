class FrameTransition {
    constructor(renderer) {
        this.renderer = renderer;
        this.transitioning = false;
    }
    
    async transitionToFrame(frameData, coordinateTransform, duration = 300) {
        this.renderer.setOpacity(1.0);
        this.renderer.renderFrame(frameData, coordinateTransform);
    }
    
    async fadeOut(duration) {
        return new Promise(resolve => {
            const startTime = performance.now();
            const startOpacity = 1.0;
            const endOpacity = 0.5;
            
            const animate = (currentTime) => {
                const elapsed = currentTime - startTime;
                const progress = Math.min(elapsed / duration, 1.0);
                
                const opacity = startOpacity + (endOpacity - startOpacity) * progress;
                this.renderer.setOpacity(opacity);
                
                if (progress < 1.0) {
                    requestAnimationFrame(animate);
                } else {
                    resolve();
                }
            };
            
            requestAnimationFrame(animate);
        });
    }
    
    async fadeIn(duration) {
        return new Promise(resolve => {
            const startTime = performance.now();
            const startOpacity = 0.5;
            const endOpacity = 1.0;
            
            const animate = (currentTime) => {
                const elapsed = currentTime - startTime;
                const progress = Math.min(elapsed / duration, 1.0);
                
                const opacity = startOpacity + (endOpacity - startOpacity) * progress;
                this.renderer.setOpacity(opacity);
                
                if (progress < 1.0) {
                    requestAnimationFrame(animate);
                } else {
                    resolve();
                }
            };
            
            requestAnimationFrame(animate);
        });
    }
    
    immediateRender(frameData, coordinateTransform) {
        this.renderer.setOpacity(1.0);
        this.renderer.renderFrame(frameData, coordinateTransform);
    }
}
