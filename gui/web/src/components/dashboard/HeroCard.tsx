import {useThemeMode} from "../../theme/ThemeContext.tsx";
import {CardB} from "./CardB.tsx";
import {BigBtnB} from "./BigBtnB.tsx";
import {StatePill} from "./StatePill.tsx";
import {RadialGauge} from "./RadialGauge.tsx";
import {Bar} from "./Bar.tsx";
import {IconAlert, IconHome, IconPlay, IconPause, IconBolt, IconBattery, IconMower, IconRain, IconSignal} from "./Icons.tsx";
import {fmt} from "./constants.ts";

interface MowerData {
  state: string;
  battery: number;
  charging: boolean;
  emergency: boolean;
  gps: number;
  current: number;
  areaPct: number;
  timeToday: number;
  dockDistance: number;
  rain: boolean;
  currentArea?: string;
}

interface HeroCardProps {
  data: MowerData;
  onStart: () => void;
  onHome: () => void;
  onPause: () => void;
  onEmergency: () => void;
  onResumeFromBoundary: () => void;
  onResetEmergency: () => void;
}

export function HeroCard({data, onStart, onHome, onPause, onEmergency, onResumeFromBoundary, onResetEmergency}: HeroCardProps) {
  const {colors} = useThemeMode();
  const {state} = data;
  const critical = state === 'BOUNDARY_VIOLATION' || state === 'EMERGENCY';

  // Critical: boundary violation or emergency
  if (critical) {
    return (
      <CardB padding={22} tone="danger" style={{
        position: 'relative', overflow: 'hidden',
        animation: 'mn-bounds-glow 2.2s ease-in-out infinite',
      }}>
        <div style={{display: 'flex', alignItems: 'flex-start', gap: 18}}>
          <div style={{
            width: 52, height: 52, borderRadius: 16, flexShrink: 0,
            background: colors.dangerBg, color: colors.danger,
            display: 'flex', alignItems: 'center', justifyContent: 'center',
          }}>
            <IconAlert size={26}/>
          </div>
          <div style={{flex: 1}}>
            <div style={{
              fontSize: 11, color: colors.danger, fontWeight: 700,
              letterSpacing: '0.12em', textTransform: 'uppercase' as const,
            }}>
              Mower needs you
            </div>
            <div style={{fontSize: 22, fontWeight: 700, color: colors.text, marginTop: 4, letterSpacing: '-0.02em'}}>
              {state === 'BOUNDARY_VIOLATION' ? 'Crossed the boundary' : 'Emergency stop'}
            </div>
            <div style={{fontSize: 14, color: colors.textDim, marginTop: 6, maxWidth: 480, lineHeight: 1.5}}>
              {state === 'BOUNDARY_VIOLATION'
                ? 'The mower rolled past your lawn edge. Lift it back inside the green zone, then tap Resume.'
                : 'The stop button was triggered. Check the mower is clear of obstacles, then release.'}
            </div>
            <div style={{display: 'flex', gap: 8, marginTop: 14}}>
              {state === 'BOUNDARY_VIOLATION' ? (
                <>
                  <BigBtnB primary icon={<IconHome size={16}/>} label="Send home" onClick={onHome}/>
                  <BigBtnB label="I moved it -- resume" icon={<IconPlay size={14}/>} onClick={onResumeFromBoundary}/>
                </>
              ) : (
                <BigBtnB primary icon={<IconAlert size={16}/>} label="Reset emergency" onClick={onResetEmergency}/>
              )}
            </div>
          </div>
        </div>
      </CardB>
    );
  }

  // Charging
  if (state === 'CHARGING') {
    const etaMin = Math.max(1, Math.round((100 - data.battery) * 1.8));
    return (
      <CardB padding={22} style={{
        background: `linear-gradient(135deg, rgba(62,224,132,0.2), rgba(62,224,132,0.04))`,
        border: `1px solid rgba(62,224,132,0.3)`,
      }}>
        <div style={{display: 'flex', alignItems: 'center', gap: 20}}>
          <RadialGauge value={data.battery} size={120} thickness={10} color={colors.accent} track="rgba(255,255,255,0.08)">
            <div style={{fontSize: 28, fontWeight: 700, color: colors.text}}>{Math.round(data.battery)}%</div>
            <div style={{
              fontSize: 10, color: colors.accent, fontWeight: 600,
              letterSpacing: '0.08em', display: 'flex', gap: 3,
              alignItems: 'center', justifyContent: 'center',
            }}>
              <IconBolt size={10}/> CHARGING
            </div>
          </RadialGauge>
          <div style={{flex: 1}}>
            <div style={{
              fontSize: 11, color: colors.accent, fontWeight: 700,
              letterSpacing: '0.12em', textTransform: 'uppercase' as const,
            }}>
              Topping up
            </div>
            <div style={{fontSize: 28, fontWeight: 700, color: colors.text, marginTop: 4, letterSpacing: '-0.02em'}}>
              Back to full in ~{etaMin} min
            </div>
            <div style={{fontSize: 14, color: colors.textDim, marginTop: 6, lineHeight: 1.5}}>
              On the dock, pulling {data.current.toFixed(1)}A. I'll take over again once we hit 100%.
            </div>
            <div style={{display: 'flex', gap: 8, marginTop: 14}}>
              <BigBtnB label="Mow anyway" icon={<IconPlay size={14}/>} onClick={onStart}/>
            </div>
          </div>
        </div>
      </CardB>
    );
  }

  // Rain
  if (data.rain && (state === 'IDLE' || state === 'IDLE_DOCKED')) {
    return (
      <CardB padding={22} style={{
        background: `linear-gradient(135deg, rgba(123,198,255,0.18), rgba(123,198,255,0.05))`,
        border: `1px solid rgba(123,198,255,0.3)`,
      }}>
        <div style={{display: 'flex', alignItems: 'center', gap: 20}}>
          <div style={{
            width: 64, height: 64, borderRadius: 18, flexShrink: 0,
            background: colors.skySoft, color: colors.sky,
            display: 'flex', alignItems: 'center', justifyContent: 'center',
          }}>
            <IconRain size={32}/>
          </div>
          <div style={{flex: 1}}>
            <div style={{
              fontSize: 11, color: colors.sky, fontWeight: 700,
              letterSpacing: '0.12em', textTransform: 'uppercase' as const,
            }}>
              Paused -- rain detected
            </div>
            <div style={{fontSize: 26, fontWeight: 700, letterSpacing: '-0.02em', marginTop: 4, color: colors.text}}>
              Waiting out the weather
            </div>
            <div style={{fontSize: 14, color: colors.textDim, marginTop: 6, lineHeight: 1.5}}>
              I'll resume automatically when it clears.
            </div>
          </div>
          <BigBtnB label="Mow anyway" onClick={onStart}/>
        </div>
      </CardB>
    );
  }

  // Low battery docking
  if (state === 'DOCKING' && data.battery < 20) {
    return (
      <CardB padding={22} style={{
        background: `linear-gradient(135deg, rgba(255,197,103,0.22), rgba(255,197,103,0.05))`,
        border: `1px solid rgba(255,197,103,0.35)`,
      }}>
        <div style={{display: 'flex', alignItems: 'center', gap: 20}}>
          <div style={{
            width: 64, height: 64, borderRadius: 18, flexShrink: 0,
            background: colors.amberSoft, color: colors.amber,
            display: 'flex', alignItems: 'center', justifyContent: 'center',
          }}>
            <IconBattery size={32}/>
          </div>
          <div style={{flex: 1}}>
            <div style={{
              fontSize: 11, color: colors.amber, fontWeight: 700,
              letterSpacing: '0.12em', textTransform: 'uppercase' as const,
            }}>
              Running low
            </div>
            <div style={{fontSize: 26, fontWeight: 700, letterSpacing: '-0.02em', marginTop: 4, color: colors.text}}>
              Heading back to dock -- {Math.round(data.battery)}%
            </div>
            <div style={{fontSize: 14, color: colors.textDim, marginTop: 6, lineHeight: 1.5}}>
              I'll resume this zone automatically once I'm charged up.
            </div>
          </div>
          <BigBtnB label="Cancel -- keep mowing" onClick={onStart}/>
        </div>
      </CardB>
    );
  }

  // Mowing / Docking / Undocking / Recording / Manual
  if (state === 'MOWING' || state === 'DOCKING' || state === 'UNDOCKING' || state === 'AREA_RECORDING' || state === 'MANUAL_MOWING') {
    const areaName = data.currentArea ?? 'the lawn';
    const headline = state === 'MOWING' || state === 'MANUAL_MOWING'
      ? `Mowing ${areaName}`
      : state === 'AREA_RECORDING'
        ? 'Recording area boundary'
        : 'Heading back to dock';
    const subtitle = state === 'MOWING' || state === 'MANUAL_MOWING'
      ? `${data.areaPct.toFixed(0)}% of this pass done -- ${fmt.mins(data.timeToday)} today`
      : state === 'DOCKING'
        ? `Battery at ${Math.round(data.battery)}%`
        : state === 'AREA_RECORDING'
          ? 'Drive along the boundary, then finish recording'
          : '';

    return (
      <CardB padding={0} style={{
        overflow: 'hidden', position: 'relative',
        background: `linear-gradient(135deg, rgba(62,224,132,0.16), rgba(123,198,255,0.08))`,
        border: `1px solid ${colors.border}`,
      }}>
        <div style={{padding: '24px 24px 20px'}}>
          <StatePill state={state} compact/>
          <div style={{
            fontSize: 28, fontWeight: 700, color: colors.text,
            marginTop: 10, letterSpacing: '-0.02em', lineHeight: 1.15,
          }}>
            {headline}
          </div>
          <div style={{fontSize: 14, color: colors.textDim, marginTop: 6, lineHeight: 1.5}}>
            {subtitle}
          </div>
          {(state === 'MOWING' || state === 'MANUAL_MOWING') && (
            <div style={{marginTop: 16, display: 'flex', flexDirection: 'column', gap: 6}}>
              <div style={{display: 'flex', justifyContent: 'space-between', fontSize: 12, color: colors.textDim}}>
                <span>Zone progress</span><span>{data.areaPct.toFixed(0)}%</span>
              </div>
              <Bar value={data.areaPct} color={colors.accent} track="rgba(255,255,255,0.08)" height={8}/>
            </div>
          )}
          <div style={{display: 'flex', gap: 8, marginTop: 18}}>
            {(state === 'MOWING' || state === 'MANUAL_MOWING') && (
              <BigBtnB label="Pause" icon={<IconPause size={14}/>} onClick={onPause}/>
            )}
            {state === 'DOCKING' && (
              <BigBtnB label="Keep mowing" icon={<IconPlay size={14}/>} primary onClick={onStart}/>
            )}
            <BigBtnB label="Send home" icon={<IconHome size={14}/>} onClick={onHome}/>
            <BigBtnB icon={<IconAlert size={14}/>} danger onClick={onEmergency} style={{padding: '12px 14px'}}/>
          </div>
          <div style={{
            position: 'absolute', top: 14, right: 14,
            padding: '6px 10px', borderRadius: 100,
            background: 'rgba(0,0,0,0.4)', backdropFilter: 'blur(6px)',
            fontSize: 11, color: colors.text, display: 'flex', alignItems: 'center', gap: 6,
          }}>
            <IconSignal size={12}/> {data.gps.toFixed(0)}% GPS
          </div>
        </div>
      </CardB>
    );
  }

  // IDLE / IDLE_DOCKED (default)
  return (
    <CardB padding={22} style={{
      background: `linear-gradient(135deg, rgba(123,198,255,0.14), rgba(62,224,132,0.06))`,
      border: `1px solid ${colors.border}`,
    }}>
      <div style={{display: 'flex', alignItems: 'center', gap: 22}}>
        <div style={{
          width: 80, height: 80, borderRadius: 22, flexShrink: 0,
          background: 'linear-gradient(135deg, rgba(62,224,132,0.3), rgba(62,224,132,0.1))',
          color: colors.accent, display: 'flex', alignItems: 'center', justifyContent: 'center',
        }}>
          <IconMower size={40}/>
        </div>
        <div style={{flex: 1}}>
          <div style={{
            fontSize: 11, color: colors.accent, fontWeight: 700,
            letterSpacing: '0.12em', textTransform: 'uppercase' as const,
          }}>
            Mowgli is ready
          </div>
          <div style={{fontSize: 26, fontWeight: 700, color: colors.text, marginTop: 4, letterSpacing: '-0.02em'}}>
            All rested at {Math.round(data.battery)}%
          </div>
          <div style={{fontSize: 14, color: colors.textDim, marginTop: 6}}>
            Tap Start mowing to begin, or wait for the next scheduled run.
          </div>
        </div>
        <div style={{display: 'flex', gap: 8}}>
          <BigBtnB label="Start mowing" primary icon={<IconPlay size={16}/>} onClick={onStart}/>
          <BigBtnB icon={<IconAlert size={14}/>} danger onClick={onEmergency} style={{padding: '12px 14px'}}/>
        </div>
      </div>
    </CardB>
  );
}
