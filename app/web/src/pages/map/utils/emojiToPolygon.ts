import type {Position} from 'geojson';

/**
 * Renders an emoji on a canvas and traces the outer contour to produce
 * a polygon ring (array of [x, y] in the range [-0.5, 0.5]).
 *
 * Uses a simple marching-squares contour extraction on the alpha channel,
 * then simplifies with Ramer-Douglas-Peucker.
 */
export function emojiToPolygon(emoji: string, resolution = 64, maxPoints = 48): Position[] | null {
    const canvas = document.createElement('canvas');
    canvas.width = resolution;
    canvas.height = resolution;
    const ctx = canvas.getContext('2d');
    if (!ctx) return null;

    // Render emoji centered on canvas
    ctx.clearRect(0, 0, resolution, resolution);
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.font = `${resolution * 0.8}px serif`;
    ctx.fillText(emoji, resolution / 2, resolution / 2);

    // Extract alpha channel
    const imageData = ctx.getImageData(0, 0, resolution, resolution);
    const alpha = new Uint8Array(resolution * resolution);
    for (let i = 0; i < alpha.length; i++) {
        alpha[i] = imageData.data[i * 4 + 3] > 40 ? 1 : 0;
    }

    // Find contour points by scanning the border between filled and empty pixels
    const contourPoints: Position[] = [];
    for (let y = 0; y < resolution; y++) {
        for (let x = 0; x < resolution; x++) {
            if (alpha[y * resolution + x] === 0) continue;
            // Check if this pixel is on the border
            const isBorder =
                x === 0 || x === resolution - 1 || y === 0 || y === resolution - 1 ||
                alpha[y * resolution + (x - 1)] === 0 ||
                alpha[y * resolution + (x + 1)] === 0 ||
                alpha[(y - 1) * resolution + x] === 0 ||
                alpha[(y + 1) * resolution + x] === 0;
            if (isBorder) {
                contourPoints.push([
                    (x / resolution) - 0.5,
                    0.5 - (y / resolution), // flip Y so up is positive
                ]);
            }
        }
    }

    if (contourPoints.length < 3) return null;

    // Order contour points by angle from centroid
    let cx = 0, cy = 0;
    for (const p of contourPoints) {
        cx += p[0];
        cy += p[1];
    }
    cx /= contourPoints.length;
    cy /= contourPoints.length;

    contourPoints.sort((a, b) => {
        const angleA = Math.atan2(a[1] - cy, a[0] - cx);
        const angleB = Math.atan2(b[1] - cy, b[0] - cx);
        return angleA - angleB;
    });

    // Simplify to maxPoints using RDP
    const simplified = rdpSimplify(contourPoints, maxPoints);

    // Close ring
    if (simplified.length > 0) {
        simplified.push(simplified[0]);
    }

    return simplified.length >= 4 ? simplified : null;
}

/**
 * Ramer-Douglas-Peucker simplification.
 * Iteratively increases epsilon until we have <= maxPoints.
 */
function rdpSimplify(points: Position[], maxPoints: number): Position[] {
    if (points.length <= maxPoints) return [...points];

    let lo = 0;
    let hi = 0.5;
    let result = points;

    for (let iter = 0; iter < 20; iter++) {
        const mid = (lo + hi) / 2;
        result = rdp(points, mid);
        if (result.length > maxPoints) {
            lo = mid;
        } else {
            hi = mid;
        }
        if (result.length === maxPoints) break;
    }
    return result;
}

function rdp(points: Position[], epsilon: number): Position[] {
    if (points.length <= 2) return [...points];

    const first = points[0];
    const last = points[points.length - 1];

    let maxDist = 0;
    let maxIdx = 0;
    for (let i = 1; i < points.length - 1; i++) {
        const d = perpendicularDist(points[i], first, last);
        if (d > maxDist) {
            maxDist = d;
            maxIdx = i;
        }
    }

    if (maxDist > epsilon) {
        const left = rdp(points.slice(0, maxIdx + 1), epsilon);
        const right = rdp(points.slice(maxIdx), epsilon);
        return [...left.slice(0, -1), ...right];
    }
    return [first, last];
}

function perpendicularDist(point: Position, lineStart: Position, lineEnd: Position): number {
    const dx = lineEnd[0] - lineStart[0];
    const dy = lineEnd[1] - lineStart[1];
    const lenSq = dx * dx + dy * dy;
    if (lenSq === 0) return Math.hypot(point[0] - lineStart[0], point[1] - lineStart[1]);
    const num = Math.abs(dy * point[0] - dx * point[1] + lineEnd[0] * lineStart[1] - lineEnd[1] * lineStart[0]);
    return num / Math.sqrt(lenSq);
}
