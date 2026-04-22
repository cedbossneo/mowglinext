import type {ReactNode} from "react";
import {useThemeMode} from "../../theme/ThemeContext.tsx";
import {CardB} from "./CardB.tsx";
import {Sparkline} from "./Sparkline.tsx";

interface TileBProps {
  icon: ReactNode;
  label: string;
  value: string | number;
  unit?: string;
  accent?: string;
  hint?: string;
  trail?: number[];
}

export function TileB({icon, label, value, unit, accent, hint, trail}: TileBProps) {
  const {colors} = useThemeMode();
  const tileAccent = accent ?? colors.accent;
  return (
    <CardB style={{display: 'flex', flexDirection: 'column', gap: 10, minHeight: 96}}>
      <div style={{display: 'flex', alignItems: 'center', gap: 10}}>
        <div style={{
          width: 32, height: 32, borderRadius: 10,
          background: `${tileAccent}20`, color: tileAccent,
          display: 'flex', alignItems: 'center', justifyContent: 'center',
        }}>
          {icon}
        </div>
        <div style={{fontSize: 12, color: colors.textDim, fontWeight: 500}}>{label}</div>
      </div>
      <div style={{display: 'flex', alignItems: 'baseline', gap: 4}}>
        <div style={{fontSize: 26, fontWeight: 700, color: colors.text, letterSpacing: '-0.02em'}}>
          {value}
        </div>
        {unit && <div style={{fontSize: 13, color: colors.textDim, fontWeight: 500}}>{unit}</div>}
      </div>
      {trail && (
        <Sparkline
          data={trail} width={160} height={22}
          stroke={tileAccent} fill={`${tileAccent}22`} strokeWidth={1.8}
        />
      )}
      {hint && <div style={{fontSize: 11, color: colors.textMuted}}>{hint}</div>}
    </CardB>
  );
}
