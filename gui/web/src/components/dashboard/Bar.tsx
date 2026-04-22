interface BarProps {
  value?: number;
  max?: number;
  color?: string;
  track?: string;
  height?: number;
  rounded?: boolean;
}

export function Bar({
  value = 0,
  max = 100,
  color = '#2CC76B',
  track = 'rgba(255,255,255,0.08)',
  height = 6,
  rounded = true,
}: BarProps) {
  const pct = Math.max(0, Math.min(100, (value / max) * 100));
  return (
    <div style={{
      height, background: track,
      borderRadius: rounded ? height : 0,
      overflow: 'hidden', width: '100%',
    }}>
      <div style={{
        height: '100%', width: `${pct}%`,
        background: color, transition: 'width .5s',
      }}/>
    </div>
  );
}
