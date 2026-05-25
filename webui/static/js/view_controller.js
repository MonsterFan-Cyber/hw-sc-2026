class ViewController {
    constructor(canvas, coordinateTransform, renderer) {
        this.canvas = canvas;
        this.coordinateTransform = coordinateTransform;
        this.renderer = renderer;
        
        this.isDragging = false;
        this.dragStartX = 0;
        this.dragStartY = 0;
        this.dragOffsetX = 0;
        this.dragOffsetY = 0;
        
        this.hoveredPolygonId = null;
        
        this.onRender = null;
        
        this.setupEventListeners();
    }
    
    setupEventListeners() {
        this.canvas.addEventListener('wheel', (e) => this.onMouseWheel(e));
        this.canvas.addEventListener('mousedown', (e) => this.onMouseDragStart(e));
        this.canvas.addEventListener('mousemove', (e) => this.onMouseDragMove(e));
        this.canvas.addEventListener('mouseup', (e) => this.onMouseDragEnd(e));
        this.canvas.addEventListener('mouseleave', (e) => this.onMouseDragEnd(e));
    }
    
    onMouseWheel(e) {
        e.preventDefault();
        
        const rect = this.canvas.getBoundingClientRect();
        const screenX = e.clientX - rect.left;
        const screenY = e.clientY - rect.top;
        
        const zoomFactor = e.deltaY < 0 ? 1.1 : 0.9;
        
        this.coordinateTransform.zoomAtPoint(zoomFactor, screenX, screenY);
        
        if (this.onRender) {
            this.onRender();
        }
    }
    
    onMouseDragStart(e) {
        if (e.button !== 0) {
            return;
        }
        
        this.isDragging = true;
        this.dragStartX = e.clientX;
        this.dragStartY = e.clientY;
        this.dragOffsetX = this.coordinateTransform.offsetX;
        this.dragOffsetY = this.coordinateTransform.offsetY;
        
        this.canvas.style.cursor = 'grabbing';
    }
    
    onMouseDragMove(e) {
        if (this.isDragging) {
            const dx = e.clientX - this.dragStartX;
            const dy = e.clientY - this.dragStartY;
            
            this.coordinateTransform.pan(-dx / this.coordinateTransform.zoom, -dy / this.coordinateTransform.zoom);
            
            this.dragStartX = e.clientX;
            this.dragStartY = e.clientY;
            
            if (this.onRender) {
                this.onRender();
            }
        }
    }
    
    onMouseDragEnd(e) {
        if (this.isDragging) {
            this.isDragging = false;
            this.canvas.style.cursor = 'grab';
        }
    }
    
    reset() {
        this.coordinateTransform.reset();
        if (this.onRender) {
            this.onRender();
        }
    }
}
