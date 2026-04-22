export const FONT = "'Manrope', 'Inter', -apple-system, BlinkMacSystemFont, sans-serif";
export const MONO_FONT = "'JetBrains Mono', monospace";

export const MOWER_STATES: Record<string, { label: string; tone: 'info' | 'primary' | 'danger'; friendly: string }> = {
  IDLE_DOCKED:        { label: 'Docked',         tone: 'info',    friendly: 'Resting on dock' },
  IDLE:               { label: 'Idle',            tone: 'info',    friendly: 'Ready when you are' },
  MOWING:             { label: 'Mowing',          tone: 'primary', friendly: 'Mowing the lawn' },
  DOCKING:            { label: 'Returning',       tone: 'primary', friendly: 'Heading back to dock' },
  UNDOCKING:          { label: 'Undocking',       tone: 'primary', friendly: 'Leaving the dock' },
  CHARGING:           { label: 'Charging',        tone: 'info',    friendly: 'Topping up the battery' },
  BOUNDARY_VIOLATION: { label: 'Boundary Alert',  tone: 'danger',  friendly: 'Stopped -- outside boundary' },
  EMERGENCY:          { label: 'Emergency Stop',  tone: 'danger',  friendly: 'Emergency stop engaged' },
  AREA_RECORDING:     { label: 'Recording',       tone: 'primary', friendly: 'Recording area boundary' },
  MANUAL_MOWING:      { label: 'Manual Mowing',   tone: 'primary', friendly: 'Manual mowing mode' },
};

export const fmt = {
  v: (n: number | undefined) => n == null ? '--' : `${n.toFixed(2)} V`,
  a: (n: number | undefined) => n == null ? '--' : `${n.toFixed(2)} A`,
  c: (n: number | undefined) => n == null ? '--' : `${n.toFixed(1)} C`,
  pct: (n: number | undefined) => n == null ? '--' : `${Math.round(n)}%`,
  rpm: (n: number | undefined) => n == null ? '--' : `${Math.round(n)}`,
  mins: (n: number | undefined) => {
    if (n == null) return '--';
    const m = Math.round(n);
    if (m < 60) return `${m} min`;
    return `${Math.floor(m / 60)}h ${m % 60}m`;
  },
};

export const KEYFRAMES_CSS = `
@keyframes mn-pulse {
  0%, 100% { opacity: 1; transform: scale(1); }
  50% { opacity: 0.4; transform: scale(0.85); }
}
@media (prefers-reduced-motion: no-preference) {
  @keyframes mn-bounds-glow {
    0%, 100% { box-shadow: 0 0 0 0 rgba(255,107,107,0.7), inset 0 0 0 1px rgba(255,107,107,0.6); }
    50% { box-shadow: 0 0 0 6px rgba(255,107,107,0), inset 0 0 0 1px rgba(255,107,107,0.9); }
  }
  @keyframes mn-pulse-red {
    0%, 100% { box-shadow: 0 0 0 0 rgba(255,107,107,0); }
    50% { box-shadow: 0 0 0 8px rgba(255,107,107,0.15); }
  }
}
.mn-card-hover:hover { transform: translateY(-1px); }
.mn-btn { transition: background .12s, border-color .12s, transform .08s; }
.mn-btn:hover { transform: translateY(-1px); }
.mn-btn:active { transform: translateY(0); }
`;
