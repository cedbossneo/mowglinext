import { useEffect, useState } from 'react';
import { Card, Typography, Space, Skeleton } from 'antd';
import { CalendarOutlined } from '@ant-design/icons';
import { useRobotConnection } from '@/connection/RobotConnection';
import { EmptyState } from '@/components/EmptyState';
import { useThemeMode } from '@/theme/ThemeProvider';

const { Title, Text } = Typography;

// ── Types ─────────────────────────────────────────────────────────────────────

interface Schedule {
  id: number;
  days: number[];           // 0 = Sunday … 6 = Saturday
  start_time: string;       // "HH:MM" wall-clock string, displayed as-is
  duration_minutes: number;
  area_id?: number;
  enabled: boolean;
}

const DAY_LABELS = ['Sun', 'Mon', 'Tue', 'Wed', 'Thu', 'Fri', 'Sat'];

function formatDuration(minutes: number): string {
  const h = Math.floor(minutes / 60);
  const m = minutes % 60;
  if (h === 0) return `${m}m`;
  if (m === 0) return `${h}h`;
  return `${h}h ${m}m`;
}

// ── Component ─────────────────────────────────────────────────────────────────

export function ScheduleScreen() {
  const { colors } = useThemeMode();
  const { state, request } = useRobotConnection();

  const [schedules, setSchedules] = useState<Schedule[]>([]);
  const [loading, setLoading] = useState(true);

  const connected = state === 'connected';

  useEffect(() => {
    if (!connected) return;
    setLoading(true);
    request<Schedule[]>('GET', '/api/schedules')
      .then((data) => setSchedules(Array.isArray(data) ? data : []))
      .catch(() => setSchedules([]))
      .finally(() => setLoading(false));
  }, [connected, request]);

  return (
    <div
      style={{
        height: '100%',
        overflowY: 'auto',
        padding: '12px 16px',
        background: colors.bgBase,
      }}
    >
      <div
        style={{
          display: 'flex',
          alignItems: 'center',
          justifyContent: 'space-between',
          marginBottom: 12,
        }}
      >
        <Title level={4} style={{ margin: 0, color: colors.text }}>
          Schedule
        </Title>
        <Text style={{ color: colors.textMuted, fontSize: 12 }}>
          Edit via web interface
        </Text>
      </div>

      {!connected || loading ? (
        <Space direction="vertical" size={12} style={{ width: '100%' }}>
          {[0, 1, 2].map((i) => (
            <Skeleton key={i} active paragraph={{ rows: 2 }} />
          ))}
        </Space>
      ) : schedules.length === 0 ? (
        <EmptyState
          icon={<CalendarOutlined />}
          title="No schedules"
          description="Create mowing schedules in the robot's web interface."
        />
      ) : (
        <Space direction="vertical" size={12} style={{ width: '100%' }}>
          {schedules.map((schedule) => (
            <Card
              key={schedule.id}
              style={{
                background: schedule.enabled ? colors.panel : colors.bgSubtle,
                borderRadius: 16,
                border: `1px solid ${colors.border}`,
                opacity: schedule.enabled ? 1 : 0.55,
              }}
              styles={{ body: { padding: 16 } }}
            >
              <div
                style={{
                  display: 'flex',
                  justifyContent: 'space-between',
                  alignItems: 'baseline',
                  marginBottom: 10,
                }}
              >
                <Text
                  style={{ color: colors.text, fontWeight: 700, fontSize: 20 }}
                >
                  {schedule.start_time}
                </Text>
                <Space size={8}>
                  <Text style={{ color: colors.textSecondary, fontSize: 13 }}>
                    {formatDuration(schedule.duration_minutes)}
                  </Text>
                  {!schedule.enabled && (
                    <Text style={{ color: colors.textMuted, fontSize: 12 }}>
                      Disabled
                    </Text>
                  )}
                </Space>
              </div>

              {/* Day-of-week pills */}
              <div style={{ display: 'flex', gap: 6, flexWrap: 'wrap' }}>
                {DAY_LABELS.map((label, idx) => {
                  const active = schedule.days.includes(idx);
                  return (
                    <span
                      key={idx}
                      style={{
                        width: 34,
                        height: 34,
                        borderRadius: 9,
                        display: 'inline-flex',
                        alignItems: 'center',
                        justifyContent: 'center',
                        fontSize: 11,
                        fontWeight: active ? 700 : 400,
                        background: active ? colors.primaryBg : colors.bgSubtle,
                        color: active ? colors.primary : colors.textMuted,
                        border: active
                          ? `1.5px solid ${colors.primary}`
                          : `1px solid ${colors.borderSubtle}`,
                      }}
                    >
                      {label}
                    </span>
                  );
                })}
              </div>

              {schedule.area_id !== undefined && (
                <Text
                  style={{
                    color: colors.textMuted,
                    fontSize: 11,
                    display: 'block',
                    marginTop: 8,
                  }}
                >
                  Area {schedule.area_id}
                </Text>
              )}
            </Card>
          ))}
        </Space>
      )}
    </div>
  );
}
