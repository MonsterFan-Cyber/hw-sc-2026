const VisualStyles = {
    THEMES: {
        dark: {
            background: '#0d1117',
            secondary: '#161b22',
            accent: '#58a6ff',
            text: '#c9d1d9',
            textSecondary: '#8b949e',
            border: '#30363d',
            grid: '#1e293b',
            illegal: '#ef4444',
            feasible: '#6366f1'
        },
        light: {
            background: '#ffffff',
            secondary: '#f6f8fa',
            accent: '#0969da',
            text: '#24292f',
            textSecondary: '#57606a',
            border: '#d0d7de',
            grid: '#eaeef2',
            illegal: '#cf222e',
            feasible: '#1f6feb'
        }
    },
    
    currentTheme: 'dark',
    
    COLORS: null,
    
    POLYGON_COLORS: [
        '#FF6B6B',
        '#4ECDC4',
        '#45B7D1',
        '#96CEB4',
        '#FFEAA7',
        '#DDA0DD',
        '#98D8C8',
        '#F7DC6F',
        '#BB8FCE',
        '#85C1E9',
        '#F8B500'
    ],
    
    init() {
        this.COLORS = this.THEMES[this.currentTheme];
    },
    
    setTheme(themeName) {
        if (this.THEMES[themeName]) {
            this.currentTheme = themeName;
            this.COLORS = this.THEMES[themeName];
            return true;
        }
        return false;
    },
    
    getPolygonColor(polygonId) {
        const index = (polygonId - 1) % this.POLYGON_COLORS.length;
        return this.POLYGON_COLORS[index];
    },
    
    applyGlowEffect(ctx, color, intensity = 15) {
    },
    
    clearGlowEffect(ctx) {
    },
    
    getPolygonFillColor(polygonId, isLegal) {
        const baseColor = this.getPolygonColor(polygonId);
        return baseColor;
    },
    
    getPolygonStrokeColor(polygonId, isLegal) {
        if (!isLegal) {
            return this.COLORS.illegal;
        }
        return this.getPolygonColor(polygonId);
    },
    
    getPolygonFillOpacity(isLegal) {
        return isLegal ? 0.9 : 0.9;
    }
};

VisualStyles.init();
