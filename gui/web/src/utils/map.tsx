import {Quaternion} from "../types/ros.ts";

export const earth = 6371008.8;  // radius of the earth in meters
export const pi = Math.PI;

// Metres per degree of latitude (WGS84 equatorial radius × deg→rad)
const METERS_PER_DEG = 6378137.0 * Math.PI / 180.0;

export function getQuaternionFromHeading(heading: number): Quaternion {
    const q = {
        X: 0,
        Y: 0,
        Z: 0,
        W: 0,
    } as Quaternion
    q.W = Math.cos(heading / 2)
    q.Z = Math.sin(heading / 2)
    return q
}

export function drawLine(offsetX: number, offsetY: number, datum: [number, number, number], y: number, x: number, orientation: number): [number, number] {
    const endX = x + Math.cos(orientation);
    const endY = y + Math.sin(orientation);
    return transpose(offsetX, offsetY, datum, endY, endX);
}

/**
 * Generate a rotated robot footprint polygon in [lon, lat] coordinates.
 * The footprint is a rectangle defined by front/rear/halfWidth offsets from
 * base_link (at wheel axis centre), rotated by the robot heading.
 */
export function drawRobotFootprint(
    offsetX: number, offsetY: number, datum: [number, number, number],
    posY: number, posX: number, heading: number,
    chassisFront: number, chassisRear: number, halfWidth: number,
): [number, number][] {
    const cos = Math.cos(heading);
    const sin = Math.sin(heading);
    // Corners in local frame relative to base_link: [forward, left]
    const corners = [
        [chassisFront, halfWidth],
        [chassisFront, -halfWidth],
        [chassisRear, -halfWidth],
        [chassisRear, halfWidth],
        [chassisFront, halfWidth], // close the polygon
    ];
    return corners.map(([fx, fy]) => {
        const rx = posX + fx * cos - fy * sin;
        const ry = posY + fx * sin + fy * cos;
        return transpose(offsetX, offsetY, datum, ry, rx);
    });
}

/**
 * Convert local map coordinates (x=east, y=north in metres relative to datum)
 * to [longitude, latitude].
 *
 * Uses the same equirectangular projection as navsat_to_absolute_pose_node:
 *   east  = (lon - datum_lon) * cos(datum_lat) * METERS_PER_DEG
 *   north = (lat - datum_lat) * METERS_PER_DEG
 */
export const transpose = (_offsetX: number, _offsetY: number, datum: [number, number, number], y: number, x: number): [number, number] => {
    // datum[0] = datum_easting (unused in equirectangular), datum[1] = datum_lat, datum[2] = datum_lon
    // However the GUI passes datum as [easting, northing, zone] from UTM.
    // We need datum_lat and datum_lon directly.  The datum is also stored as
    // settings["datum_lat"] / settings["datum_lon"] and passed as
    // datum = [datum_lon, datum_lat, 0] in useMapStreams — check the caller.
    //
    // After refactoring, datum = [datum_lat, datum_lon, 0]
    const datum_lat = datum[0];
    const datum_lon = datum[1];
    const cos_lat = Math.cos(datum_lat * Math.PI / 180.0);

    const lon = datum_lon + x / (cos_lat * METERS_PER_DEG);
    const lat = datum_lat + y / METERS_PER_DEG;
    return [lon, lat];
};

/**
 * Convert [longitude, latitude] to local map coordinates (x=east, y=north).
 * Inverse of transpose.
 */
export const itranspose = (_offsetX: number, _offsetY: number, datum: [number, number, number], lat: number, lon: number): [number, number] => {
    const datum_lat = datum[0];
    const datum_lon = datum[1];
    const cos_lat = Math.cos(datum_lat * Math.PI / 180.0);

    const x = (lon - datum_lon) * cos_lat * METERS_PER_DEG;
    const y = (lat - datum_lat) * METERS_PER_DEG;
    return [x, y];
};

/**
 * Remove near-duplicate consecutive points caused by floating-point precision.
 */
export function dedupePoints(points: { x: number; y: number; z: number }[], epsilon = 0.001): { x: number; y: number; z: number }[] {
    if (points.length === 0) return points;
    const result = [points[0]];
    for (let i = 1; i < points.length; i++) {
        const prev = result[result.length - 1];
        const curr = points[i];
        const dx = curr.x - prev.x;
        const dy = curr.y - prev.y;
        if (Math.sqrt(dx * dx + dy * dy) > epsilon) {
            result.push(curr);
        }
    }
    return result;
}
