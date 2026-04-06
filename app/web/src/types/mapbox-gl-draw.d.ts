declare module '@mapbox/mapbox-gl-draw' {
    import type {IControl, Map} from 'mapbox-gl';

    interface DrawOptions {
        displayControlsDefault?: boolean;
        controls?: {
            point?: boolean;
            line_string?: boolean;
            polygon?: boolean;
            trash?: boolean;
            combine_features?: boolean;
            uncombine_features?: boolean;
        };
        userProperties?: boolean;
        styles?: object[];
        modes?: Record<string, object>;
        defaultMode?: string;
        keybindings?: boolean;
        touchEnabled?: boolean;
        boxSelect?: boolean;
        clickBuffer?: number;
        touchBuffer?: number;
    }

    class MapboxDraw implements IControl {
        constructor(options?: DrawOptions);

        static modes: Record<string, object>;
        static constants: Record<string, any>;
        static lib: {
            doubleClickZoom: { enable(ctx: any): void; disable(ctx: any): void };
            CommonSelectors: {
                isOfMetaType(metaType: string): (e: any) => boolean;
                isActiveFeature(e: any): boolean;
                isInactiveFeature(e: any): boolean;
                noTarget(e: any): boolean;
                isFeature(e: any): boolean;
                isShiftDown(e: any): boolean;
                isVertex(e: any): boolean;
                isShiftMousedown(e: any): boolean;
                true(): boolean;
            };
            constrainFeatureMovement(geojsonFeatures: any, delta: any): any;
            moveFeatures(features: any, delta: any): void;
            createSupplementaryPoints(geojson: any, options?: any): any[];
            [key: string]: any;
        };

        onAdd(map: Map): HTMLElement;
        onRemove(map: Map): void;
        getDefaultPosition(): 'top-left' | 'top-right' | 'bottom-left' | 'bottom-right';

        add(geojson: GeoJSON.Feature | GeoJSON.FeatureCollection | GeoJSON.Geometry): string[];
        get(featureId: string): GeoJSON.Feature | undefined;
        getFeatureIdsAt(point: { x: number; y: number }): string[];
        getSelectedIds(): string[];
        getSelected(): GeoJSON.FeatureCollection;
        getSelectedPoints(): GeoJSON.FeatureCollection;
        getAll(): GeoJSON.FeatureCollection;
        delete(ids: string | string[]): this;
        deleteAll(): this;
        set(featureCollection: GeoJSON.FeatureCollection): string[];
        trash(): this;
        combineFeatures(): this;
        uncombineFeatures(): this;
        getMode(): string;
        changeMode(mode: string, options?: object): this;
        setFeatureProperty(featureId: string, property: string, value: any): this;
    }

    export default MapboxDraw;
}
