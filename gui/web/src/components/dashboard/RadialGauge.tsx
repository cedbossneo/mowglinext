import type {ReactNode} from "react";

interface RadialGaugeProps {
  value?: number;
  max?: number;
  size?: number;
  thickness?: number;
  color?: string;
  track?: string;
  startAngle?: number;
  endAngle?: number;
  children?: ReactNode;
}

export function RadialGauge({
  value = 50,
  max = 100,
  size = 120,
  thickness = 10,
  color = '#2CC76B',
  track = 'rgba(255,255,255,0.08)',
  startAngle = -210,
  endAngle = 30,
  children,
}: RadialGaugeProps) {
  const r = (size - thickness) / 2;
  const cx = size / 2;
  const cy = size / 2;
  const totalSweep = endAngle - startAngle;
  const pct = Math.max(0, Math.min(1, value / max));
  const rad = (a: number) => (a * Math.PI) / 180;
  const polar = (a: number): [number, number] => [
    cx + r * Math.cos(rad(a)),
    cy + r * Math.sin(rad(a)),
  ];
  const [sx, sy] = polar(startAngle);
  const [tx, ty] = polar(endAngle);
  const [vx, vy] = polar(startAngle + totalSweep * pct);
  const largeTrack = totalSweep > 180 ? 1 : 0;
  const largeValue = totalSweep * pct > 180 ? 1 : 0;

  return (
    <div style={{
      position: 'relative', width: size, height: size,
      display: 'flex', alignItems: 'center', justifyContent: 'center',
    }}>
      <svg width={size} height={size} style={{position: 'absolute', inset: 0}}>
        <path
          d={`M${sx},${sy} A${r},${r} 0 ${largeTrack} 1 ${tx},${ty}`}
          stroke={track} strokeWidth={thickness} fill="none" strokeLinecap="round"
        />
        <path
          d={`M${sx},${sy} A${r},${r} 0 ${largeValue} 1 ${vx},${vy}`}
          stroke={color} strokeWidth={thickness} fill="none" strokeLinecap="round"
          style={{transition: 'stroke-dasharray 0.5s, d 0.5s'}}
        />
      </svg>
      <div style={{position: 'relative', zIndex: 1, textAlign: 'center'}}>
        {children}
      </div>
    </div>
  );
}
