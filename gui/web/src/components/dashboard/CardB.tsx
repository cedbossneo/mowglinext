import type {CSSProperties, ReactNode} from "react";
import {useThemeMode} from "../../theme/ThemeContext.tsx";

interface CardBProps {
  children: ReactNode;
  style?: CSSProperties;
  tone?: 'accent' | 'danger';
  padding?: number;
  onClick?: () => void;
}

export function CardB({children, style, tone, padding = 18, onClick}: CardBProps) {
  const {colors} = useThemeMode();
  const bg = tone === 'accent'
    ? `linear-gradient(135deg, ${colors.accent}, ${colors.accent}aa)`
    : tone === 'danger'
      ? 'linear-gradient(135deg, rgba(255,107,107,0.25), rgba(255,107,107,0.08))'
      : colors.panel;
  return (
    <div
      onClick={onClick}
      className={onClick ? 'mn-card-hover' : undefined}
      style={{
        background: bg,
        borderRadius: 18,
        padding,
        border: tone === 'accent' ? 'none' : `1px solid ${colors.border}`,
        cursor: onClick ? 'pointer' : 'default',
        transition: 'transform .15s',
        ...style,
      }}
    >
      {children}
    </div>
  );
}
