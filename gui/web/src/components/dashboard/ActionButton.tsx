import type {CSSProperties, ReactNode} from "react";
import {useThemeMode} from "../../theme/ThemeContext.tsx";
import {FONT} from "./constants.ts";

interface ActionButtonProps {
  label?: string;
  icon?: ReactNode;
  onClick?: () => void;
  primary?: boolean;
  danger?: boolean;
  style?: CSSProperties;
  disabled?: boolean;
}

export function ActionButton({label, icon, onClick, primary, danger, style, disabled}: ActionButtonProps) {
  const {colors} = useThemeMode();
  const bg = primary ? colors.accent
    : danger ? 'rgba(255,107,107,0.14)'
    : 'rgba(255,255,255,0.06)';
  const fg = primary ? '#0a1a10'
    : danger ? colors.danger
    : colors.text;
  const bd = danger ? 'rgba(255,107,107,0.3)'
    : primary ? colors.accent
    : 'rgba(255,255,255,0.1)';
  return (
    <button
      onClick={onClick}
      disabled={disabled}
      className="mn-btn"
      style={{
        display: 'inline-flex', alignItems: 'center', gap: 8,
        padding: '12px 18px', background: bg, color: fg,
        border: `1px solid ${bd}`, borderRadius: 14, cursor: disabled ? 'not-allowed' : 'pointer',
        fontSize: 14, fontWeight: 600, fontFamily: FONT,
        opacity: disabled ? 0.5 : 1,
        ...style,
      }}
    >
      {icon} {label}
    </button>
  );
}
