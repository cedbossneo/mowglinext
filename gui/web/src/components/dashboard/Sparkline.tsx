interface SparklineProps {
  data: number[];
  width?: number;
  height?: number;
  stroke?: string;
  fill?: string;
  strokeWidth?: number;
}

export function Sparkline({
  data,
  width = 96,
  height = 24,
  stroke = '#2CC76B',
  fill = 'rgba(44,199,107,0.12)',
  strokeWidth = 1.5,
}: SparklineProps) {
  if (!data || data.length < 2) return <svg width={width} height={height}/>;
  const min = Math.min(...data);
  const max = Math.max(...data);
  const span = max - min || 1;
  const step = width / (data.length - 1);
  const points = data.map((v, i) => [
    i * step,
    height - ((v - min) / span) * (height - 2) - 1,
  ]);
  const path = points
    .map((p, i) => (i === 0 ? `M${p[0]},${p[1]}` : `L${p[0]},${p[1]}`))
    .join(' ');
  const area = `${path} L${width},${height} L0,${height} Z`;
  return (
    <svg width={width} height={height} style={{display: 'block'}}>
      <path d={area} fill={fill}/>
      <path d={path} fill="none" stroke={stroke} strokeWidth={strokeWidth}
            strokeLinecap="round" strokeLinejoin="round"/>
    </svg>
  );
}
