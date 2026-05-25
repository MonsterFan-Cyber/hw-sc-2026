class CanvasRenderer {
    constructor(canvas) {
        this.canvas = canvas;
        this.ctx = canvas.getContext('2d');
        this.opacity = 1.0;
        this.showIds = false;
    }
    
    clear() {
        this.ctx.fillStyle = VisualStyles.COLORS.background;
        this.ctx.fillRect(0, 0, this.canvas.width, this.canvas.height);
        this.drawGrid();
    }
    
    drawGrid() {
        const gridSize = 50;
        this.ctx.strokeStyle = VisualStyles.COLORS.grid;
        this.ctx.lineWidth = 0.5;
        this.ctx.globalAlpha = 0.3;
        
        for (let x = 0; x < this.canvas.width; x += gridSize) {
            this.ctx.beginPath();
            this.ctx.moveTo(x, 0);
            this.ctx.lineTo(x, this.canvas.height);
            this.ctx.stroke();
        }
        
        for (let y = 0; y < this.canvas.height; y += gridSize) {
            this.ctx.beginPath();
            this.ctx.moveTo(0, y);
            this.ctx.lineTo(this.canvas.width, y);
            this.ctx.stroke();
        }
        
        this.ctx.globalAlpha = 1.0;
    }
    
    renderFeasibleRegion(feasibleRegion, coordinateTransform) {
        if (!feasibleRegion || !feasibleRegion.vertices || feasibleRegion.vertices.length < 3) {
            return;
        }
        
        const vertices = feasibleRegion.vertices;
        
        this.ctx.beginPath();
        const [startX, startY] = coordinateTransform.worldToScreen(vertices[0][0], vertices[0][1]);
        this.ctx.moveTo(startX, startY);
        
        for (let i = 1; i < vertices.length; i++) {
            const [x, y] = coordinateTransform.worldToScreen(vertices[i][0], vertices[i][1]);
            this.ctx.lineTo(x, y);
        }
        
        this.ctx.closePath();
        
        this.ctx.strokeStyle = VisualStyles.COLORS.feasible;
        this.ctx.lineWidth = 2;
        this.ctx.setLineDash([8, 4]);
        this.ctx.stroke();
        this.ctx.setLineDash([]);
    }
    
    renderPolygon(polygonState, coordinateTransform, hovered = false) {
        const { id, vertices, is_legal } = polygonState;
        
        if (vertices.length < 3) {
            return;
        }
        
        this.ctx.beginPath();
        const [startX, startY] = coordinateTransform.worldToScreen(vertices[0][0], vertices[0][1]);
        this.ctx.moveTo(startX, startY);
        
        for (let i = 1; i < vertices.length; i++) {
            const [x, y] = coordinateTransform.worldToScreen(vertices[i][0], vertices[i][1]);
            this.ctx.lineTo(x, y);
        }
        
        this.ctx.closePath();
        
        const fillColor = VisualStyles.getPolygonFillColor(id, is_legal);
        const fillOpacity = VisualStyles.getPolygonFillOpacity(is_legal);
        
        this.ctx.globalAlpha = fillOpacity;
        this.ctx.fillStyle = fillColor;
        this.ctx.fill();
        
        if (!is_legal) {
            this.ctx.globalAlpha = 0.3;
            this.ctx.fillStyle = VisualStyles.COLORS.illegal;
            this.ctx.fill();
        }
        
        this.ctx.globalAlpha = 1.0;
        
        this.renderPolygonLabel(id, vertices, coordinateTransform);
    }
    
    renderPolygonLabel(polygonId, vertices, coordinateTransform) {
        if (!this.showIds) {
            return;
        }
        
        let centerX = 0, centerY = 0;
        for (const [x, y] of vertices) {
            centerX += x;
            centerY += y;
        }
        centerX /= vertices.length;
        centerY /= vertices.length;
        
        const [screenX, screenY] = coordinateTransform.worldToScreen(centerX, centerY);
        
        this.ctx.font = 'bold 14px Consolas, Monaco, monospace';
        this.ctx.textAlign = 'center';
        this.ctx.textBaseline = 'middle';
        
        this.ctx.fillStyle = '#000000';
        this.ctx.fillText(polygonId, screenX + 1, screenY + 1);
        
        this.ctx.fillStyle = '#ffffff';
        this.ctx.fillText(polygonId, screenX, screenY);
    }
    
    renderFrame(frameData, coordinateTransform) {
        this.clear();
        
        const { polygon_states, feasible_region } = frameData;
        
        this.renderFeasibleRegion(feasible_region, coordinateTransform);
        
        for (const polygon of polygon_states) {
            this.renderPolygon(polygon, coordinateTransform);
        }
    }
    
    renderFrameInfo(currentFrame, totalFrames) {
        const text = `${currentFrame} / ${totalFrames}`;
        const frameInfoEl = document.getElementById('frameInfo');
        if (frameInfoEl) {
            frameInfoEl.textContent = text;
        }
    }
    
    setOpacity(opacity) {
        this.opacity = opacity;
        this.canvas.style.opacity = opacity;
    }
}
