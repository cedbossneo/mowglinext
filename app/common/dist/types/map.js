export class MowingFeature {
    constructor(id) {
        Object.defineProperty(this, "id", {
            enumerable: true,
            configurable: true,
            writable: true,
            value: void 0
        });
        Object.defineProperty(this, "type", {
            enumerable: true,
            configurable: true,
            writable: true,
            value: void 0
        });
        Object.defineProperty(this, "geometry", {
            enumerable: true,
            configurable: true,
            writable: true,
            value: void 0
        });
        Object.defineProperty(this, "properties", {
            enumerable: true,
            configurable: true,
            writable: true,
            value: void 0
        });
        this.type = 'Feature';
        this.id = id;
        this.geometry = { type: 'Point', coordinates: [0, 0] };
        this.properties = {};
    }
}
export class PointFeatureBase extends MowingFeature {
    constructor(id, coordinate, feature_type) {
        super(id);
        Object.defineProperty(this, "geometry", {
            enumerable: true,
            configurable: true,
            writable: true,
            value: void 0
        });
        Object.defineProperty(this, "properties", {
            enumerable: true,
            configurable: true,
            writable: true,
            value: void 0
        });
        this.properties = {
            color: 'black',
            feature_type: feature_type
        };
        this.geometry = { type: 'Point', coordinates: coordinate };
    }
    setColor(color) {
        this.properties.color = color;
    }
}
export class LineFeatureBase extends MowingFeature {
    constructor(id, coordinates, color, feature_type) {
        super(id);
        Object.defineProperty(this, "geometry", {
            enumerable: true,
            configurable: true,
            writable: true,
            value: void 0
        });
        Object.defineProperty(this, "properties", {
            enumerable: true,
            configurable: true,
            writable: true,
            value: void 0
        });
        this.properties = {
            color: color,
            width: 1,
            feature_type: feature_type
        };
        this.geometry = { type: 'LineString', coordinates: coordinates };
    }
}
export class PathFeature extends LineFeatureBase {
    constructor(id, coordinates, color, lineWidth = 1) {
        super(id, coordinates, color, 'path');
        this.properties.width = lineWidth;
    }
}
export class ActivePathFeature extends LineFeatureBase {
    constructor(id, coordinates) {
        super(id, coordinates, 'orange', 'active_path');
        this.properties.width = 3;
    }
}
export class MowerFeatureBase extends PointFeatureBase {
    constructor(coordinate) {
        super('mower', coordinate, 'mower');
        this.setColor('#00a6ff');
    }
}
export class DockFeatureBase extends PointFeatureBase {
    constructor(coordinate, heading = 0) {
        super('dock', coordinate, 'dock');
        this.properties.heading = heading;
        this.setColor('#ff00f2');
    }
    getHeading() {
        return this.properties.heading ?? 0;
    }
    setHeading(heading) {
        this.properties.heading = heading;
    }
    getCoordinates() {
        return this.geometry.coordinates;
    }
    setCoordinates(coordinate) {
        this.geometry.coordinates = coordinate;
    }
}
export class MowingFeatureBase extends MowingFeature {
    constructor(id, feature_type) {
        super(id);
        Object.defineProperty(this, "geometry", {
            enumerable: true,
            configurable: true,
            writable: true,
            value: void 0
        });
        Object.defineProperty(this, "properties", {
            enumerable: true,
            configurable: true,
            writable: true,
            value: void 0
        });
        Object.defineProperty(this, "bbox", {
            enumerable: true,
            configurable: true,
            writable: true,
            value: void 0
        });
        this.type = 'Feature';
        this.properties = {
            color: 'black',
            index: 0,
            mowing_order: 9999,
            feature_type: feature_type
        };
        this.geometry = { type: 'Polygon', coordinates: [] };
    }
    setGeometry(geometry) {
        this.geometry = geometry;
    }
    transpose(points, offsetX, offsetY, datum, transposeFn) {
        this.geometry.coordinates = [points.map((point) => {
                return transposeFn(offsetX, offsetY, datum, point.Y || 0, point.X || 0);
            })];
    }
    setColor(color) {
        this.properties.color = color;
        return this;
    }
}
export class ObstacleFeature extends MowingFeatureBase {
    constructor(id, mowing_area) {
        super(id, 'obstacle');
        Object.defineProperty(this, "mowing_area", {
            enumerable: true,
            configurable: true,
            writable: true,
            value: void 0
        });
        this.setColor("#bf0000");
        this.mowing_area = mowing_area;
    }
    getMowingArea() {
        return this.mowing_area;
    }
}
export class MapAreaFeature extends MowingFeatureBase {
    constructor(id, feature_type) {
        super(id, feature_type);
        Object.defineProperty(this, "area", {
            enumerable: true,
            configurable: true,
            writable: true,
            value: void 0
        });
    }
    setArea(area, offsetX, offsetY, datum, transposeFn) {
        this.area = area;
        this.transpose(area.Area?.Points ?? [], offsetX, offsetY, datum, transposeFn);
    }
    getArea() {
        return this.area;
    }
}
export class NavigationFeature extends MapAreaFeature {
    constructor(id) {
        super(id, 'navigation');
        this.setColor("white");
    }
}
export class MowingAreaFeature extends MapAreaFeature {
    //mowing_order: number;
    constructor(id, mowing_order) {
        super(id, 'workarea');
        this.properties.mowing_order = mowing_order;
        this.setName('');
        this.setColor("#01d30d");
    }
    setArea(area, offsetX, offsetY, datum, transposeFn) {
        super.setArea(area, offsetX, offsetY, datum, transposeFn);
        this.setName(area.Name ?? '');
    }
    setName(name) {
        this.properties['name'] = name;
        if (this.area)
            this.area.Name = name;
        return this;
    }
    getName() {
        return this.properties?.name ? this.properties?.name : '';
    }
    getMowingOrder() {
        return this.properties.mowing_order;
    }
    setMowingOrder(val) {
        this.properties.mowing_order = val;
        return this;
    }
    getIndex() {
        return this.properties.mowing_order - 1;
    }
    getLabel() {
        const name = this.getName();
        return name ? name + " (" + this.getMowingOrder().toString() + ")" : "Area " + this.getMowingOrder().toString();
    }
}
