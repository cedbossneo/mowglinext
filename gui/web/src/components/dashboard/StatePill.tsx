import {MOWER_STATES} from "./constants.ts";

interface StatePillProps {
  state: string;
  compact?: boolean;
  pulse?: boolean;
}

export function StatePill({state, compact = false, pulse = true}: StatePillProps) {
  const info = MOWER_STATES[state] ?? {label: state, tone: 'info' as const};
  const bg = info.tone === 'danger' ? 'rgba(255,107,107,0.14)'
    : info.tone === 'primary' ? 'rgba(44,199,107,0.14)'
    : 'rgba(255,255,255,0.08)';
  const fg = info.tone === 'danger' ? '#ff6b6b'
    : info.tone === 'primary' ? '#4ade80'
    : 'rgba(255,255,255,0.85)';
  return (
    <span style={{
      display: 'inline-flex', alignItems: 'center', gap: 6,
      background: bg, color: fg,
      padding: compact ? '3px 8px' : '5px 10px',
      borderRadius: 100, fontSize: compact ? 11 : 12, fontWeight: 600,
      letterSpacing: '0.02em',
    }}>
      <span style={{
        width: 6, height: 6, borderRadius: 3, background: fg,
        display: 'inline-block',
        animation: pulse && info.tone !== 'info' ? 'mn-pulse 1.2s ease-in-out infinite' : 'none',
      }}/>
      {info.label}
    </span>
  );
}
