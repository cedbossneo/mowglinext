import { useThemeMode } from '@/theme/ThemeProvider';

export type DotStatus = 'online' | 'offline' | 'connecting';

interface StatusDotProps {
  status: DotStatus;
  showLabel?: boolean;
  lastSeenSeconds?: number;
}

export function StatusDot({ status, showLabel = false, lastSeenSeconds }: StatusDotProps) {
  const { colors } = useThemeMode();

  const color =
    status === 'online'
      ? colors.success
      : status === 'connecting'
        ? colors.warning
        : colors.muted;

  const label =
    status === 'online'
      ? 'Online'
      : status === 'connecting'
        ? 'Connecting…'
        : lastSeenSeconds !== undefined
          ? `Offline — last seen ${formatAge(lastSeenSeconds)}`
          : 'Offline';

  return (
    <span style={{ display: 'inline-flex', alignItems: 'center', gap: 6 }}>
      <span
        style={{
          width: 8,
          height: 8,
          borderRadius: '50%',
          background: color,
          display: 'inline-block',
          flexShrink: 0,
          boxShadow: status === 'online' ? `0 0 6px ${color}` : undefined,
        }}
      />
      {showLabel && (
        <span style={{ fontSize: 13, color: colors.textSecondary }}>{label}</span>
      )}
    </span>
  );
}

function formatAge(seconds: number): string {
  if (seconds < 60) return `${seconds}s ago`;
  if (seconds < 3600) return `${Math.floor(seconds / 60)}m ago`;
  return `${Math.floor(seconds / 3600)}h ago`;
}
