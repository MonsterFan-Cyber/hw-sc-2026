class CoordinateTransform {
    constructor(canvasWidth, canvasHeight) {
        this.canvasWidth = canvasWidth;
        this.canvasHeight = canvasHeight;
        this.zoom = 1.0;
        this.offsetX = 0.0;
        this.offsetY = 0.0;
        this.MIN_ZOOM = 0.1;
        this.MAX_ZOOM = 1000.0;
    }
    
    computeAutoFitZoom(polygonStates, feasibleRegion) {
        if (!feasibleRegion || !feasibleRegion.vertices || feasibleRegion.vertices.length === 0) {
            if (!polygonStates || polygonStates.length === 0) {
                return;
            }
            
            let minX = Infinity, maxX = -Infinity;
            let minY = Infinity, maxY = -Infinity;
            
            for (const polygon of polygonStates) {
                for (const [x, y] of polygon.vertices) {
                    minX = Math.min(minX, x);
                    maxX = Math.max(maxX, x);
                    minY = Math.min(minY, y);
                    maxY = Math.max(maxY, y);
                }
            }
            
            if (!isFinite(minX) || !isFinite(maxX) || !isFinite(minY) || !isFinite(maxY)) {
                return;
            }
            
            const width = maxX - minX;
            const height = maxY - minY;
            
            if (width === 0 || height === 0) {
                return;
            }
            
            const padding = 0.05;
            const zoomX = (this.canvasWidth * (1 - padding)) / width;
            const zoomY = (this.canvasHeight * (1 - padding)) / height;
            
            this.zoom = this.clampZoom(Math.min(zoomX, zoomY));
            
            const centerX = (minX + maxX) / 2;
            const centerY = (minY + maxY) / 2;
            
            this.offsetX = centerX;
            this.offsetY = centerY;
            return;
        }
        
        let minX = Infinity, maxX = -Infinity;
        let minY = Infinity, maxY = -Infinity;
        
        for (const [x, y] of feasibleRegion.vertices) {
            minX = Math.min(minX, x);
            maxX = Math.max(maxX, x);
            minY = Math.min(minY, y);
            maxY = Math.max(maxY, y);
        }
        
        if (polygonStates) {
            for (const polygon of polygonStates) {
                for (const [x, y] of polygon.vertices) {
                    minX = Math.min(minX, x);
                    maxX = Math.max(maxX, x);
                    minY = Math.min(minY, y);
                    maxY = Math.max(maxY, y);
                }
            }
        }
        
        if (!isFinite(minX) || !isFinite(maxX) || !isFinite(minY) || !isFinite(maxY)) {
            return;
        }
        
        const width = maxX - minX;
        const height = maxY - minY;
        
        if (width === 0 || height === 0) {
            return;
        }
        
        const padding = 0.05;
        const zoomX = (this.canvasWidth * (1 - padding)) / width;
        const zoomY = (this.canvasHeight * (1 - padding)) / height;
        
        this.zoom = this.clampZoom(Math.min(zoomX, zoomY));
        
        const centerX = (minX + maxX) / 2;
        const centerY = (minY + maxY) / 2;
        
        this.offsetX = centerX;
        this.offsetY = centerY;
    }
    
    clampZoom(zoom) {
        return Math.max(this.MIN_ZOOM, Math.min(this.MAX_ZOOM, zoom));
    }
    
    worldToScreen(worldX, worldY) {
        const screenX = (worldX - this.offsetX) * this.zoom + this.canvasWidth / 2;
        const screenY = (worldY - this.offsetY) * this.zoom + this.canvasHeight / 2;
        return [screenX, screenY];
    }
    
    screenToWorld(screenX, screenY) {
        const worldX = (screenX - this.canvasWidth / 2) / this.zoom + this.offsetX;
        const worldY = (screenY - this.canvasHeight / 2) / this.zoom + this.offsetY;
        return [worldX, worldY];
    }
    
    zoomAtPoint(zoomFactor, screenX, screenY) {
        const [worldX, worldY] = this.screenToWorld(screenX, screenY);
        
        const newZoom = this.clampZoom(this.zoom * zoomFactor);
        
        if (newZoom === this.zoom) {
            return;
        }
        
        this.zoom = newZoom;
        
        const [newScreenX, newScreenY] = this.worldToScreen(worldX, worldY);
        
        this.offsetX += (newScreenX - screenX) / this.zoom;
        this.offsetY += (newScreenY - screenY) / this.zoom;
    }
    
    pan(dx, dy) {
        this.offsetX += dx / this.zoom;
        this.offsetY += dy / this.zoom;
    }
    
    reset() {
        this.zoom = 1.0;
        this.offsetX = 0.0;
        this.offsetY = 0.0;
    }
}
