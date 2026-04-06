import MapboxDraw from '@mapbox/mapbox-gl-draw';

/**
 * A custom draw mode that draws a 2-point line and auto-completes on the second click.
 * Used for the split tool — click start, click end, done.
 * Extends MapboxDraw's built-in draw_line_string mode.
 *
 * How it works:
 * - First click: delegates to the original clickAnywhere (places first vertex)
 * - Second click: places the second vertex and immediately calls changeMode
 *   to finish. The inherited onStop from draw_line_string cleans up the
 *   trailing preview vertex and fires draw.create automatically.
 */
const DrawLineString = MapboxDraw.modes.draw_line_string;

const SplitLineMode: Record<string, any> = {};

// Copy all methods from the built-in draw_line_string mode
for (const key of Object.keys(DrawLineString)) {
    SplitLineMode[key] = (DrawLineString as any)[key];
}

// Override clickAnywhere to auto-complete after 2 points
const originalClickAnywhere = (DrawLineString as any).clickAnywhere;

SplitLineMode.clickAnywhere = function (state: any, e: any) {
    if (state.currentVertexPosition >= 1) {
        // Second click — place the real coordinate
        state.line.updateCoordinate(state.currentVertexPosition, e.lngLat.lng, e.lngLat.lat);
        // Advance the position so onStop's removeCoordinate removes the right index
        state.currentVertexPosition++;
        // Add a throwaway preview coord that onStop will strip
        state.line.updateCoordinate(state.currentVertexPosition, e.lngLat.lng, e.lngLat.lat);
        // Finish — onStop fires draw.create with the 2-point line
        return this.changeMode('simple_select');
    }
    // First click — delegate to original behavior
    return originalClickAnywhere.call(this, state, e);
};

export default SplitLineMode;
