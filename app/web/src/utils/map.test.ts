import {describe, it, expect} from 'vitest';
import {dedupePoints, getQuaternionFromHeading, transpose, itranspose} from './map';

describe('dedupePoints', () => {
    it('returns empty array for empty input', () => {
        expect(dedupePoints([])).toEqual([]);
    });

    it('returns single point unchanged', () => {
        const points = [{x: 1, y: 2, z: 0}];
        expect(dedupePoints(points)).toEqual(points);
    });

    it('keeps distinct points', () => {
        const points = [
            {x: 0, y: 0, z: 0},
            {x: 1, y: 0, z: 0},
            {x: 1, y: 1, z: 0},
        ];
        expect(dedupePoints(points)).toEqual(points);
    });

    it('removes duplicate consecutive points within epsilon', () => {
        const points = [
            {x: 0, y: 0, z: 0},
            {x: 0.0001, y: 0.0001, z: 0},
            {x: 1, y: 1, z: 0},
        ];
        const result = dedupePoints(points);
        expect(result).toHaveLength(2);
        expect(result[0]).toEqual({x: 0, y: 0, z: 0});
        expect(result[1]).toEqual({x: 1, y: 1, z: 0});
    });

    it('keeps points farther apart than epsilon', () => {
        const points = [
            {x: 0, y: 0, z: 0},
            {x: 0.002, y: 0, z: 0},
            {x: 0.004, y: 0, z: 0},
        ];
        const result = dedupePoints(points, 0.001);
        expect(result).toHaveLength(3);
    });

    it('uses custom epsilon', () => {
        const points = [
            {x: 0, y: 0, z: 0},
            {x: 0.5, y: 0, z: 0},
            {x: 1.0, y: 0, z: 0},
        ];
        const result = dedupePoints(points, 0.6);
        expect(result).toHaveLength(2);
        expect(result[0]).toEqual({x: 0, y: 0, z: 0});
        expect(result[1]).toEqual({x: 1.0, y: 0, z: 0});
    });

    it('handles multiple consecutive duplicates', () => {
        const points = [
            {x: 0, y: 0, z: 0},
            {x: 0.0001, y: 0, z: 0},
            {x: 0.0002, y: 0, z: 0},
            {x: 0.0003, y: 0, z: 0},
            {x: 5, y: 5, z: 0},
        ];
        const result = dedupePoints(points);
        expect(result).toHaveLength(2);
    });

    it('preserves z values', () => {
        const points = [
            {x: 0, y: 0, z: 10},
            {x: 1, y: 1, z: 20},
        ];
        const result = dedupePoints(points);
        expect(result[0].z).toBe(10);
        expect(result[1].z).toBe(20);
    });
});

describe('getQuaternionFromHeading', () => {
    it('returns identity quaternion for heading 0', () => {
        const q = getQuaternionFromHeading(0);
        expect(q.W).toBeCloseTo(1);
        expect(q.X).toBe(0);
        expect(q.Y).toBe(0);
        expect(q.Z).toBeCloseTo(0);
    });

    it('returns correct quaternion for PI/2', () => {
        const q = getQuaternionFromHeading(Math.PI / 2);
        expect(q.W).toBeCloseTo(Math.cos(Math.PI / 4));
        expect(q.Z).toBeCloseTo(Math.sin(Math.PI / 4));
    });

    it('returns correct quaternion for PI', () => {
        const q = getQuaternionFromHeading(Math.PI);
        expect(q.W).toBeCloseTo(0, 5);
        expect(q.Z).toBeCloseTo(1);
    });

    it('handles negative heading', () => {
        const q = getQuaternionFromHeading(-Math.PI / 2);
        expect(q.W).toBeCloseTo(Math.cos(-Math.PI / 4));
        expect(q.Z).toBeCloseTo(Math.sin(-Math.PI / 4));
    });
});

describe('transpose / itranspose', () => {
    const datum: [number, number, number] = [500000, 5500000, 32];

    it('transpose converts UTM to lat/lon', () => {
        const [lon, lat] = transpose(0, 0, datum, 0, 0);
        expect(typeof lon).toBe('number');
        expect(typeof lat).toBe('number');
        expect(lon).not.toBeNaN();
        expect(lat).not.toBeNaN();
    });

    it('itranspose inverts transpose', () => {
        const [lon, lat] = transpose(0, 0, datum, 10, 20);
        const [x, y] = itranspose(0, 0, datum, lat, lon);
        expect(x).toBeCloseTo(20, 0);
        expect(y).toBeCloseTo(10, 0);
    });

    it('offset params are reserved but currently ignored', () => {
        const [lon1] = transpose(0, 0, datum, 0, 0);
        const [lon2] = transpose(100, 0, datum, 0, 0);
        // Offsets are currently unused (params prefixed with _)
        expect(lon2).toBeCloseTo(lon1, 5);
    });

    it('itranspose with offset inverts correctly', () => {
        const offsetX = 5;
        const offsetY = -3;
        const [lon, lat] = transpose(offsetX, offsetY, datum, 10, 20);
        const [x, y] = itranspose(offsetX, offsetY, datum, lat, lon);
        expect(x).toBeCloseTo(20, 0);
        expect(y).toBeCloseTo(10, 0);
    });
});
