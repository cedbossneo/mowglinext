import { useEffect, useRef, useState } from 'react';
import { Typography, Skeleton } from 'antd';
import { useRobotConnection } from '@/connection/RobotConnection';
import { useThemeMode } from '@/theme/ThemeProvider';

const { Text } = Typography;

// ── Types ─────────────────────────────────────────────────────────────────────

interface RobotPose {
  x: number;
  y: number;
  yaw: number;
}

interface Area {
  id: number;
  name: string;
  polygon: Array<{ x: number; y: number }>;
}

interface PoseMessage {
  pose: {
    pose: {
      position: { x: number; y: number; z: number };
      orientation: { x: number; y: number; z: number; w: number };
    };
  };
}

function quatToYaw(qx: number, qy: number, qz: number, qw: number): number {
  return Math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz));
}

// ── Component ─────────────────────────────────────────────────────────────────

export function MapScreen() {
  const { colors } = useThemeMode();
  const { state, subscribe, request } = useRobotConnection();
  const containerRef = useRef<HTMLDivElement>(null);

  const [pose, setPose] = useState<RobotPose | null>(null);
  const [areas, setAreas] = useState<Area[]>([]);
  const [containerSize, setContainerSize] = useState({ w: 300, h: 400 });

  const connected = state === 'connected';

  // Track container size for responsive SVG
  useEffect(() => {
    const el = containerRef.current;
    if (!el) return;
    const obs = new ResizeObserver((entries) => {
      const { width, height } = entries[0]!.contentRect;
      setContainerSize({ w: Math.max(1, width), h: Math.max(1, height) });
    });
    obs.observe(el);
    return () => obs.disconnect();
  }, []);

  // Live pose subscription
  useEffect(() => {
    if (!connected) return;
    const unsub = subscribe<PoseMessage>(
      '/api/mowglinext/subscribe/pose',
      (msg) => {
        const pos = msg.pose.pose.position;
        const ori = msg.pose.pose.orientation;
        setPose({
          x: pos.x,
          y: pos.y,
          yaw: quatToYaw(ori.x, ori.y, ori.z, ori.w),
        });
      },
    );
    return unsub;
  }, [connected, subscribe]);

  // Fetch area outlines once on connect
  useEffect(() => {
    if (!connected) return;
    request<{ areas: Area[] }>('GET', '/api/mowglinext/areas')
      .then((res) => setAreas(res?.areas ?? []))
      .catch(() => undefined);
  }, [connected, request]);

  // Compute SVG viewBox from world coordinates
  const allPoints: Array<{ x: number; y: number }> = [
    ...areas.flatMap((a) => a.polygon),
    ...(pose ? [{ x: pose.x, y: pose.y }] : []),
  ];

  let viewBox = '-10 -10 20 20';
  if (allPoints.length > 0) {
    const xs = allPoints.map((p) => p.x);
    const ys = allPoints.map((p) => p.y);
    const minX = Math.min(...xs) - 3;
    const maxX = Math.max(...xs) + 3;
    const minY = Math.min(...ys) - 3;
    const maxY = Math.max(...ys) + 3;
    // SVG Y axis is flipped vs ENU: negate Y values in path data, keep viewBox in world units
    viewBox = `${minX} ${-maxY} ${maxX - minX} ${maxY - minY}`;
  }

  return (
    <div
      style={{
        height: '100%',
        display: 'flex',
        flexDirection: 'column',
        background: colors.bgBase,
        padding: '12px 16px',
        gap: 8,
      }}
    >
      <Text style={{ color: colors.textMuted, fontSize: 12 }}>
        Live robot position
      </Text>

      <div
        ref={containerRef}
        style={{
          flex: 1,
          background: colors.panel,
          borderRadius: 20,
          border: `1px solid ${colors.border}`,
          overflow: 'hidden',
          position: 'relative',
        }}
      >
        {!connected ? (
          <div style={{ padding: 20 }}>
            <Skeleton active paragraph={{ rows: 6 }} />
          </div>
        ) : (
          <svg
            width={containerSize.w}
            height={containerSize.h}
            viewBox={viewBox}
            preserveAspectRatio="xMidYMid meet"
            style={{ display: 'block' }}
          >
            {/* Area polygon outlines */}
            {areas.map((area) => {
              if (area.polygon.length < 3) return null;
              const pts = area.polygon.map((p) => `${p.x},${-p.y}`).join(' ');
              return (
                <polygon
                  key={area.id}
                  points={pts}
                  fill={colors.accentSoft}
                  stroke={colors.primary}
                  strokeWidth={0.15}
                />
              );
            })}

            {/* Robot marker */}
            {pose && (
              <g
                transform={`translate(${pose.x},${-pose.y}) rotate(${
                  (-pose.yaw * 180) / Math.PI
                })`}
              >
                <circle r={0.35} fill={colors.primary} opacity={0.95} />
                {/* Heading indicator */}
                <line
                  x1={0}
                  y1={0}
                  x2={0}
                  y2={-0.7}
                  stroke="white"
                  strokeWidth={0.14}
                  strokeLinecap="round"
                />
              </g>
            )}

            {/* North indicator */}
            <text
              x={0}
              y={0}
              fontSize={0.8}
              fill={colors.textMuted}
              textAnchor="middle"
              dominantBaseline="middle"
            >
              N↑
            </text>
          </svg>
        )}
      </div>

      {pose && (
        <Text
          style={{ color: colors.textMuted, fontSize: 11, textAlign: 'center' }}
        >
          {pose.x.toFixed(2)} m E, {pose.y.toFixed(2)} m N ·{' '}
          {((pose.yaw * 180) / Math.PI).toFixed(1)}°
        </Text>
      )}
    </div>
  );
}
