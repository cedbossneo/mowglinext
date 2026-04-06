import type { BBox, Feature, Polygon, Point, Position, LineString } from 'geojson';
import { MapArea, Point32 } from "./ros";
/**
 * Transpose function type — platform-specific (web uses map projection, mobile may differ).
 * Injected via the transpose() method parameter.
 */
export type TransposeFn = (offsetX: number, offsetY: number, datum: [number, number, number], y: number, x: number) => [number, number];
export declare class MowingFeature implements Feature {
    id: string;
    type: 'Feature';
    geometry: Polygon | Point | LineString;
    properties: Record<string, unknown>;
    constructor(id: string);
}
export declare class PointFeatureBase extends MowingFeature implements Feature<Point> {
    geometry: Point;
    properties: {
        color: string;
        feature_type: string;
    };
    constructor(id: string, coordinate: Position, feature_type: string);
    setColor(color: string): void;
}
export declare class LineFeatureBase extends MowingFeature implements Feature<LineString> {
    geometry: LineString;
    properties: {
        color: string;
        width: number;
        feature_type: string;
    };
    constructor(id: string, coordinates: Position[], color: string, feature_type: string);
}
export declare class PathFeature extends LineFeatureBase {
    constructor(id: string, coordinates: Position[], color: string, lineWidth?: number);
}
export declare class ActivePathFeature extends LineFeatureBase {
    constructor(id: string, coordinates: Position[]);
}
export declare class MowerFeatureBase extends PointFeatureBase {
    constructor(coordinate: Position);
}
export declare class DockFeatureBase extends PointFeatureBase {
    properties: {
        color: string;
        feature_type: string;
        heading: number;
    };
    constructor(coordinate: Position, heading?: number);
    getHeading(): number;
    setHeading(heading: number): void;
    getCoordinates(): Position;
    setCoordinates(coordinate: Position): void;
}
export declare class MowingFeatureBase extends MowingFeature implements Feature<Polygon> {
    geometry: Polygon;
    properties: {
        color: string;
        name?: string;
        index: number;
        mowing_order: number;
        feature_type: string;
    };
    bbox?: BBox | undefined;
    constructor(id: string, feature_type: string);
    setGeometry(geometry: Polygon): void;
    transpose(points: Point32[], offsetX: number, offsetY: number, datum: [number, number, number], transposeFn: TransposeFn): void;
    setColor(color: string): MowingFeatureBase;
}
export declare class ObstacleFeature extends MowingFeatureBase {
    mowing_area: MowingAreaFeature;
    constructor(id: string, mowing_area: MowingAreaFeature);
    getMowingArea(): MowingAreaFeature;
}
export declare class MapAreaFeature extends MowingFeatureBase {
    area?: MapArea;
    constructor(id: string, feature_type: string);
    setArea(area: MapArea, offsetX: number, offsetY: number, datum: [number, number, number], transposeFn: TransposeFn): void;
    getArea(): MapArea | undefined;
}
export declare class NavigationFeature extends MapAreaFeature {
    constructor(id: string);
}
export declare class MowingAreaFeature extends MapAreaFeature {
    constructor(id: string, mowing_order: number);
    setArea(area: MapArea, offsetX: number, offsetY: number, datum: [number, number, number], transposeFn: TransposeFn): void;
    setName(name: string): MowingAreaFeature;
    getName(): string;
    getMowingOrder(): number;
    setMowingOrder(val: number): MowingAreaFeature;
    getIndex(): number;
    getLabel(): string;
}
//# sourceMappingURL=map.d.ts.map