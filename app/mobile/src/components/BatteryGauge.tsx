import { useThemeMode } from '@/theme/ThemeProvider';

interface BatteryGaugeProps {
  percent: number;
  charging?: boolean;
  showLabel?: boolean;
  size?: 'sm' | 'md';
}

export function BatteryGauge({
  percent,
  charging = false,
  showLabel = true,
  size = 'md',
}: BatteryGaugeProps) {
  const { colors } = useThemeMode();
  const clamped = Math.max(0, Math.min(100, percent));
  const bodyW = size === 'sm' ? 32 : 48;
  const bodyH = size === 'sm' ? 16 : 24;
  const r = size === 'sm' ? 3 : 5;
  const tipW = size === 'sm' ? 3 : 4;
  const tipH = size === 'sm' ? 6 : 8;

  const fillColor =
    charging
      ? colors.info
      : clamped > 40
        ? colors.success
        : clamped > 20
          ? colors.warning
          : colors.danger;

  return (
    <span style={{ display: 'inline-flex', alignItems: 'center', gap: 8 }}>
      <svg
        width={bodyW + tipW}
        height={bodyH}
        viewBox={`0 0 ${bodyW + tipW} ${bodyH}`}
        style={{ display: 'block', flexShrink: 0 }}
        aria-label={`Battery ${clamped}%${charging ? ', charging' : ''}`}
      >
        {/* Body outline */}
        <rect
          x={0}
          y={0}
          width={bodyW}
          height={bodyH}
          rx={r}
          ry={r}
          fill="none"
          stroke={colors.border}
          strokeWidth={1.5}
        />
        {/* Terminal nub */}
        <rect
          x={bodyW + 1}
          y={(bodyH - tipH) / 2}
          width={tipW - 1}
          height={tipH}
          rx={1}
          fill={colors.border}
        />
        {/* Fill bar */}
        <rect
          x={2}
          y={2}
          width={Math.max(0, (bodyW - 4) * (clamped / 100))}
          height={bodyH - 4}
          rx={r - 2}
          fill={fillColor}
        />
      </svg>
      {showLabel && (
        <span
          style={{
            fontSize: size === 'sm' ? 12 : 14,
            color: colors.textSecondary,
            fontVariantNumeric: 'tabular-nums',
          }}
        >
          {charging ? '⚡ ' : ''}
          {clamped}%
        </span>
      )}
    </span>
  );
}
