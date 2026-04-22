import {useThemeMode} from "../../theme/ThemeContext.tsx";
import {DashCard} from "./Card.tsx";
import {ActionButton} from "./ActionButton.tsx";
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
  compact?: boolean;
  onStart: () => void;
  onHome: () => void;
  onPause: () => void;
  onEmergency: () => void;
  onResumeFromBoundary: () => void;
  onResetEmergency: () => void;
}

export function HeroCard({data, compact, onStart, onHome, onPause, onEmergency, onResumeFromBoundary, onResetEmergency}: HeroCardProps) {
  const {colors} = useThemeMode();
  const {state} = data;
  const critical = state === 'BOUNDARY_VIOLATION' || state === 'EMERGENCY';

  // Critical: boundary violation or emergency
  if (critical) {
    return (
      <DashCard padding={compact ? 16 : 22} tone="danger" style={{
        position: 'relative', overflow: 'hidden',
        animation: 'mn-bounds-glow 2.2s ease-in-out infinite',
      }}>
        <div style={{display: 'flex', alignItems: compact ? 'center' : 'flex-start', gap: compact ? 12 : 18, flexWrap: compact ? 'wrap' : undefined}}>
          <div style={{
            width: compact ? 40 : 52, height: compact ? 40 : 52, borderRadius: compact ? 12 : 16, flexShrink: 0,
            background: colors.dangerBg, color: colors.danger,
            display: 'flex', alignItems: 'center', justifyContent: 'center',
          }}>
            <IconAlert size={compact ? 20 : 26}/>
          </div>
          <div style={{flex: 1, minWidth: compact ? '60%' : undefined}}>
            <div style={{
              fontSize: compact ? 10 : 11, color: colors.danger, fontWeight: 700,
              letterSpacing: '0.12em', textTransform: 'uppercase' as const,
            }}>
              Mower needs you
            </div>
            <div style={{fontSize: compact ? 17 : 22, fontWeight: 700, color: colors.text, marginTop: 4, letterSpacing: '-0.02em'}}>
              {state === 'BOUNDARY_VIOLATION' ? 'Crossed the boundary' : 'Emergency stop'}
            </div>
            <div style={{fontSize: compact ? 13 : 14, color: colors.textDim, marginTop: 6, lineHeight: 1.5}}>
              {state === 'BOUNDARY_VIOLATION'
                ? 'Lift the mower back inside the boundary, then resume.'
                : 'Check the mower is clear of obstacles, then release.'}
            </div>
          </div>
        </div>
        <div style={{display: 'flex', gap: 8, marginTop: 14, flexWrap: 'wrap'}}>
          {state === 'BOUNDARY_VIOLATION' ? (
            <>
              <ActionButton primary icon={<IconHome size={16}/>} label="Send home" onClick={onHome}
                       style={compact ? {flex: 1, justifyContent: 'center'} : undefined}/>
              <ActionButton label="Resume" icon={<IconPlay size={14}/>} onClick={onResumeFromBoundary}
                       style={compact ? {flex: 1, justifyContent: 'center'} : undefined}/>
            </>
          ) : (
            <ActionButton primary icon={<IconAlert size={16}/>} label="Reset emergency" onClick={onResetEmergency}
                     style={compact ? {flex: 1, justifyContent: 'center'} : undefined}/>
          )}
        </div>
      </DashCard>
    );
  }

  // Charging
  if (state === 'CHARGING') {
    const etaMin = Math.max(1, Math.round((100 - data.battery) * 1.8));
    const gaugeSize = compact ? 90 : 120;
    return (
      <DashCard padding={compact ? 16 : 22} style={{
        background: `linear-gradient(135deg, rgba(62,224,132,0.2), rgba(62,224,132,0.04))`,
        border: `1px solid rgba(62,224,132,0.3)`,
      }}>
        <div style={{
          display: 'flex',
          flexDirection: compact ? 'column' : 'row',
          alignItems: compact ? 'flex-start' : 'center',
          gap: compact ? 14 : 20,
        }}>
          {compact ? (
            <div style={{display: 'flex', alignItems: 'center', gap: 14, width: '100%'}}>
              <RadialGauge value={data.battery} size={gaugeSize} thickness={8} color={colors.accent} track="rgba(255,255,255,0.08)">
                <div style={{fontSize: 22, fontWeight: 700, color: colors.text}}>{Math.round(data.battery)}%</div>
              </RadialGauge>
              <div style={{flex: 1}}>
                <div style={{
                  fontSize: 10, color: colors.accent, fontWeight: 700,
                  letterSpacing: '0.08em', display: 'flex', gap: 3, alignItems: 'center',
                }}>
                  <IconBolt size={10}/> CHARGING
                </div>
                <div style={{fontSize: 20, fontWeight: 700, color: colors.text, marginTop: 4, letterSpacing: '-0.02em'}}>
                  ~{etaMin} min left
                </div>
                <div style={{fontSize: 13, color: colors.textDim, marginTop: 4}}>
                  Pulling {data.current.toFixed(1)}A
                </div>
              </div>
            </div>
          ) : (
            <>
              <RadialGauge value={data.battery} size={gaugeSize} thickness={10} color={colors.accent} track="rgba(255,255,255,0.08)">
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
                  <ActionButton label="Mow anyway" icon={<IconPlay size={14}/>} onClick={onStart}/>
                </div>
              </div>
            </>
          )}
        </div>
        {compact && (
          <div style={{display: 'flex', gap: 8, marginTop: 12}}>
            <ActionButton label="Mow anyway" icon={<IconPlay size={14}/>} onClick={onStart}
                     style={{flex: 1, justifyContent: 'center'}}/>
          </div>
        )}
      </DashCard>
    );
  }

  // Rain
  if (data.rain && (state === 'IDLE' || state === 'IDLE_DOCKED')) {
    return (
      <DashCard padding={compact ? 16 : 22} style={{
        background: `linear-gradient(135deg, rgba(123,198,255,0.18), rgba(123,198,255,0.05))`,
        border: `1px solid rgba(123,198,255,0.3)`,
      }}>
        <div style={{
          display: 'flex', alignItems: 'center', gap: compact ? 12 : 20,
          flexWrap: compact ? 'wrap' : undefined,
        }}>
          <div style={{
            width: compact ? 44 : 64, height: compact ? 44 : 64, borderRadius: compact ? 14 : 18, flexShrink: 0,
            background: colors.skySoft, color: colors.sky,
            display: 'flex', alignItems: 'center', justifyContent: 'center',
          }}>
            <IconRain size={compact ? 22 : 32}/>
          </div>
          <div style={{flex: 1, minWidth: compact ? '60%' : undefined}}>
            <div style={{
              fontSize: compact ? 10 : 11, color: colors.sky, fontWeight: 700,
              letterSpacing: '0.12em', textTransform: 'uppercase' as const,
            }}>
              Paused -- rain detected
            </div>
            <div style={{fontSize: compact ? 20 : 26, fontWeight: 700, letterSpacing: '-0.02em', marginTop: 4, color: colors.text}}>
              Waiting out the weather
            </div>
            <div style={{fontSize: compact ? 13 : 14, color: colors.textDim, marginTop: 6, lineHeight: 1.5}}>
              I'll resume automatically when it clears.
            </div>
          </div>
        </div>
        <div style={{display: 'flex', gap: 8, marginTop: 12}}>
          <ActionButton label="Mow anyway" onClick={onStart}
                   style={compact ? {flex: 1, justifyContent: 'center'} : undefined}/>
        </div>
      </DashCard>
    );
  }

  // Low battery docking
  if (state === 'DOCKING' && data.battery < 20) {
    return (
      <DashCard padding={compact ? 16 : 22} style={{
        background: `linear-gradient(135deg, rgba(255,197,103,0.22), rgba(255,197,103,0.05))`,
        border: `1px solid rgba(255,197,103,0.35)`,
      }}>
        <div style={{
          display: 'flex', alignItems: 'center', gap: compact ? 12 : 20,
          flexWrap: compact ? 'wrap' : undefined,
        }}>
          <div style={{
            width: compact ? 44 : 64, height: compact ? 44 : 64, borderRadius: compact ? 14 : 18, flexShrink: 0,
            background: colors.amberSoft, color: colors.amber,
            display: 'flex', alignItems: 'center', justifyContent: 'center',
          }}>
            <IconBattery size={compact ? 22 : 32}/>
          </div>
          <div style={{flex: 1, minWidth: compact ? '60%' : undefined}}>
            <div style={{
              fontSize: compact ? 10 : 11, color: colors.amber, fontWeight: 700,
              letterSpacing: '0.12em', textTransform: 'uppercase' as const,
            }}>
              Running low
            </div>
            <div style={{fontSize: compact ? 20 : 26, fontWeight: 700, letterSpacing: '-0.02em', marginTop: 4, color: colors.text}}>
              Heading to dock -- {Math.round(data.battery)}%
            </div>
            <div style={{fontSize: compact ? 13 : 14, color: colors.textDim, marginTop: 6, lineHeight: 1.5}}>
              I'll resume automatically once charged.
            </div>
          </div>
        </div>
        <div style={{display: 'flex', gap: 8, marginTop: 12}}>
          <ActionButton label="Keep mowing" onClick={onStart}
                   style={compact ? {flex: 1, justifyContent: 'center'} : undefined}/>
        </div>
      </DashCard>
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
      <DashCard padding={0} style={{
        overflow: 'hidden', position: 'relative',
        background: `linear-gradient(135deg, rgba(62,224,132,0.16), rgba(123,198,255,0.08))`,
        border: `1px solid ${colors.border}`,
      }}>
        <div style={{padding: compact ? '16px 16px 12px' : '24px 24px 20px'}}>
          <div style={{display: 'flex', alignItems: 'center', justifyContent: 'space-between'}}>
            <StatePill state={state} compact/>
            <div style={{
              padding: '4px 8px', borderRadius: 100,
              background: 'rgba(0,0,0,0.3)', backdropFilter: 'blur(6px)',
              fontSize: 11, color: colors.text, display: 'flex', alignItems: 'center', gap: 4,
            }}>
              <IconSignal size={12}/> {data.gps.toFixed(0)}%
            </div>
          </div>
          <div style={{
            fontSize: compact ? 20 : 28, fontWeight: 700, color: colors.text,
            marginTop: compact ? 8 : 10, letterSpacing: '-0.02em', lineHeight: 1.15,
          }}>
            {headline}
          </div>
          <div style={{fontSize: compact ? 13 : 14, color: colors.textDim, marginTop: 6, lineHeight: 1.5}}>
            {subtitle}
          </div>
          {(state === 'MOWING' || state === 'MANUAL_MOWING') && (
            <div style={{marginTop: compact ? 10 : 16, display: 'flex', flexDirection: 'column', gap: 4}}>
              <div style={{display: 'flex', justifyContent: 'space-between', fontSize: 11, color: colors.textDim}}>
                <span>Zone progress</span><span>{data.areaPct.toFixed(0)}%</span>
              </div>
              <Bar value={data.areaPct} color={colors.accent} track="rgba(255,255,255,0.08)" height={compact ? 6 : 8}/>
            </div>
          )}
          <div style={{display: 'flex', gap: 8, marginTop: compact ? 12 : 18, flexWrap: 'wrap'}}>
            {(state === 'MOWING' || state === 'MANUAL_MOWING') && (
              <ActionButton label="Pause" icon={<IconPause size={14}/>} onClick={onPause}
                       style={compact ? {flex: 1, justifyContent: 'center', padding: '10px 14px'} : undefined}/>
            )}
            {state === 'DOCKING' && (
              <ActionButton label="Keep mowing" icon={<IconPlay size={14}/>} primary onClick={onStart}
                       style={compact ? {flex: 1, justifyContent: 'center', padding: '10px 14px'} : undefined}/>
            )}
            <ActionButton label="Home" icon={<IconHome size={14}/>} onClick={onHome}
                     style={compact ? {flex: 1, justifyContent: 'center', padding: '10px 14px'} : undefined}/>
            <ActionButton icon={<IconAlert size={14}/>} danger onClick={onEmergency}
                     style={{padding: compact ? '10px 14px' : '12px 14px'}}/>
          </div>
        </div>
      </DashCard>
    );
  }

  // IDLE / IDLE_DOCKED (default)
  return (
    <DashCard padding={compact ? 16 : 22} style={{
      background: `linear-gradient(135deg, rgba(123,198,255,0.14), rgba(62,224,132,0.06))`,
      border: `1px solid ${colors.border}`,
    }}>
      <div style={{
        display: 'flex',
        flexDirection: compact ? 'column' : 'row',
        alignItems: compact ? 'flex-start' : 'center',
        gap: compact ? 14 : 22,
      }}>
        <div style={{display: 'flex', alignItems: 'center', gap: 14}}>
          <div style={{
            width: compact ? 56 : 80, height: compact ? 56 : 80, borderRadius: compact ? 16 : 22, flexShrink: 0,
            background: 'linear-gradient(135deg, rgba(62,224,132,0.3), rgba(62,224,132,0.1))',
            color: colors.accent, display: 'flex', alignItems: 'center', justifyContent: 'center',
          }}>
            <IconMower size={compact ? 28 : 40}/>
          </div>
          {compact && (
            <div>
              <div style={{
                fontSize: 10, color: colors.accent, fontWeight: 700,
                letterSpacing: '0.12em', textTransform: 'uppercase' as const,
              }}>
                Mowgli is ready
              </div>
              <div style={{fontSize: 20, fontWeight: 700, color: colors.text, marginTop: 2, letterSpacing: '-0.02em'}}>
                All rested at {Math.round(data.battery)}%
              </div>
            </div>
          )}
        </div>
        {!compact && (
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
        )}
        {compact && (
          <div style={{fontSize: 13, color: colors.textDim}}>
            Tap Start to begin, or wait for the next scheduled run.
          </div>
        )}
        <div style={{display: 'flex', gap: 8, width: compact ? '100%' : undefined}}>
          <ActionButton label="Start mowing" primary icon={<IconPlay size={16}/>} onClick={onStart}
                   style={compact ? {flex: 1, justifyContent: 'center'} : undefined}/>
          <ActionButton icon={<IconAlert size={14}/>} danger onClick={onEmergency} style={{padding: '12px 14px'}}/>
        </div>
      </div>
    </DashCard>
  );
}
