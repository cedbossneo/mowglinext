import { useEffect, useState } from 'react';
import { Card, Typography, Space, Skeleton, Row, Col } from 'antd';
import {
  ThunderboltOutlined,
  AimOutlined,
  ReloadOutlined,
} from '@ant-design/icons';
import { useRobotConnection } from '@/connection/RobotConnection';
import { BatteryGauge } from '@/components/BatteryGauge';
import { useThemeMode } from '@/theme/ThemeProvider';

const { Title, Text } = Typography;

// ── ROS message shapes (snake_case — rosbridge JSON serialisation) ────────────

interface HighLevelStatus {
  state: number;
  state_name: string;
  battery_voltage: number;
  charging: boolean;
  emergency: boolean;
  emergency_reason: string;
  mow_progress: number;
  current_area: number;
  current_strip: number;
}

interface PowerStatus {
  voltage: number;
  current: number;
  charging: boolean;
  battery_empty: boolean;
}

interface GpsStatus {
  status: { status: number };
  latitude: number;
  longitude: number;
}

interface SystemInfo {
  version?: string;
  uptime?: number;
}

// ── Helpers ───────────────────────────────────────────────────────────────────

const GPS_LABELS: Record<number, string> = {
  [-1]: 'No fix',
  0: 'Fix',
  1: 'SBAS',
  2: 'RTK / GBAS',
};

function gpsLabel(status: number): string {
  return GPS_LABELS[status] ?? `Status ${status}`;
}

function voltageToPercent(v: number): number {
  // Linear estimate for 6S LiPo: 25.2 V = 100 %, 21.0 V = 0 %
  return Math.max(0, Math.min(100, Math.round(((v - 21.0) / 4.2) * 100)));
}

function formatUptime(seconds: number): string {
  const h = Math.floor(seconds / 3600);
  const m = Math.floor((seconds % 3600) / 60);
  return h > 0 ? `${h}h ${m}m` : `${m}m`;
}

// ── Component ─────────────────────────────────────────────────────────────────

export function DashboardScreen() {
  const { colors } = useThemeMode();
  const { state: connState, subscribe, request } = useRobotConnection();

  const [hlStatus, setHlStatus] = useState<HighLevelStatus | null>(null);
  const [power, setPower] = useState<PowerStatus | null>(null);
  const [gps, setGps] = useState<GpsStatus | null>(null);
  const [sysInfo, setSysInfo] = useState<SystemInfo | null>(null);
  const [refreshing, setRefreshing] = useState(false);

  const connected = connState === 'connected';

  // Live WS subscriptions — re-subscribe whenever connection (re)establishes
  useEffect(() => {
    if (!connected) return;

    const unsubHl = subscribe<HighLevelStatus>(
      '/api/mowglinext/subscribe/high_level_status',
      setHlStatus,
    );
    const unsubPower = subscribe<PowerStatus>(
      '/api/mowglinext/subscribe/power',
      setPower,
    );
    const unsubGps = subscribe<GpsStatus>(
      '/api/mowglinext/subscribe/gps',
      setGps,
    );

    return () => {
      unsubHl();
      unsubPower();
      unsubGps();
    };
  }, [connected, subscribe]);

  // One-shot system info fetch
  useEffect(() => {
    if (!connected) return;
    request<SystemInfo>('GET', '/api/system/info')
      .then(setSysInfo)
      .catch(() => undefined);
  }, [connected, request]);

  async function handleRefresh() {
    setRefreshing(true);
    try {
      const info = await request<SystemInfo>('GET', '/api/system/info');
      setSysInfo(info);
    } catch {
      // non-fatal
    } finally {
      setRefreshing(false);
    }
  }

  const batteryV = power?.voltage ?? hlStatus?.battery_voltage ?? 0;
  const batteryPct = voltageToPercent(batteryV);
  const charging = power?.charging ?? hlStatus?.charging ?? false;

  return (
    <div
      style={{
        height: '100%',
        overflowY: 'auto',
        padding: '12px 16px',
        background: colors.bgBase,
      }}
    >
      {/* Main status card */}
      <Card
        style={{
          background: colors.panel,
          borderRadius: 20,
          border: `1px solid ${colors.border}`,
          marginBottom: 12,
        }}
        styles={{ body: { padding: 20 } }}
      >
        {!connected || !hlStatus ? (
          <Skeleton active paragraph={{ rows: 3 }} />
        ) : (
          <Space direction="vertical" size={12} style={{ width: '100%' }}>
            <div
              style={{
                display: 'flex',
                justifyContent: 'space-between',
                alignItems: 'flex-start',
              }}
            >
              <div>
                <Text style={{ color: colors.textMuted, fontSize: 12 }}>
                  State
                </Text>
                <Title level={4} style={{ margin: 0, color: colors.text }}>
                  {hlStatus.state_name ?? `State ${hlStatus.state}`}
                </Title>
              </div>
              <BatteryGauge percent={batteryPct} charging={charging} />
            </div>

            {hlStatus.emergency && (
              <div
                style={{
                  background: colors.dangerBg,
                  borderRadius: 10,
                  padding: '8px 12px',
                }}
              >
                <Text style={{ color: colors.danger, fontWeight: 600 }}>
                  Emergency:{' '}
                  {hlStatus.emergency_reason || 'Active'}
                </Text>
              </div>
            )}

            {hlStatus.mow_progress > 0 && (
              <div>
                <div
                  style={{
                    display: 'flex',
                    justifyContent: 'space-between',
                    marginBottom: 4,
                  }}
                >
                  <Text style={{ color: colors.textMuted, fontSize: 12 }}>
                    Mow progress
                  </Text>
                  <Text style={{ color: colors.textSecondary, fontSize: 12 }}>
                    {Math.round(hlStatus.mow_progress)}%
                  </Text>
                </div>
                <div
                  style={{
                    height: 6,
                    borderRadius: 3,
                    background: colors.bgSubtle,
                    overflow: 'hidden',
                  }}
                >
                  <div
                    style={{
                      height: '100%',
                      width: `${Math.min(100, hlStatus.mow_progress)}%`,
                      background: colors.primary,
                      borderRadius: 3,
                      transition: 'width 0.4s ease',
                    }}
                  />
                </div>
              </div>
            )}
          </Space>
        )}
      </Card>

      {/* GPS + Power cards */}
      <Row gutter={12}>
        <Col span={12}>
          <Card
            style={{
              background: colors.panel,
              borderRadius: 16,
              border: `1px solid ${colors.border}`,
            }}
            styles={{ body: { padding: 16 } }}
          >
            <Space align="center" size={6} style={{ marginBottom: 6 }}>
              <AimOutlined style={{ color: colors.sky, fontSize: 16 }} />
              <Text style={{ color: colors.textMuted, fontSize: 12 }}>GPS</Text>
            </Space>
            {!connected || !gps ? (
              <Skeleton active paragraph={{ rows: 1 }} title={false} />
            ) : (
              <>
                <Text
                  style={{
                    display: 'block',
                    color: colors.text,
                    fontWeight: 600,
                    fontSize: 15,
                  }}
                >
                  {gpsLabel(gps.status.status)}
                </Text>
                <Text style={{ fontSize: 11, color: colors.textMuted }}>
                  {gps.latitude.toFixed(5)}, {gps.longitude.toFixed(5)}
                </Text>
              </>
            )}
          </Card>
        </Col>

        <Col span={12}>
          <Card
            style={{
              background: colors.panel,
              borderRadius: 16,
              border: `1px solid ${colors.border}`,
            }}
            styles={{ body: { padding: 16 } }}
          >
            <Space align="center" size={6} style={{ marginBottom: 6 }}>
              <ThunderboltOutlined style={{ color: colors.amber, fontSize: 16 }} />
              <Text style={{ color: colors.textMuted, fontSize: 12 }}>Power</Text>
            </Space>
            {!connected || !power ? (
              <Skeleton active paragraph={{ rows: 1 }} title={false} />
            ) : (
              <>
                <Text
                  style={{
                    display: 'block',
                    color: colors.text,
                    fontWeight: 600,
                    fontSize: 15,
                  }}
                >
                  {power.voltage.toFixed(1)} V
                </Text>
                <Text style={{ fontSize: 11, color: colors.textMuted }}>
                  {power.current.toFixed(2)} A
                  {charging ? ' · charging' : ''}
                </Text>
              </>
            )}
          </Card>
        </Col>
      </Row>

      {/* System info */}
      {sysInfo && (
        <Card
          style={{
            background: colors.panel,
            borderRadius: 16,
            border: `1px solid ${colors.border}`,
            marginTop: 12,
          }}
          styles={{ body: { padding: 16 } }}
          title={
            <Text style={{ color: colors.textMuted, fontSize: 12 }}>System</Text>
          }
          extra={
            <ReloadOutlined
              style={{ color: colors.textMuted, cursor: 'pointer' }}
              spin={refreshing}
              onClick={() => void handleRefresh()}
            />
          }
        >
          <Space size={24}>
            {sysInfo.version && (
              <div>
                <Text style={{ color: colors.textMuted, fontSize: 11 }}>
                  Version
                </Text>
                <Text style={{ display: 'block', color: colors.text }}>
                  {sysInfo.version}
                </Text>
              </div>
            )}
            {sysInfo.uptime !== undefined && (
              <div>
                <Text style={{ color: colors.textMuted, fontSize: 11 }}>
                  Uptime
                </Text>
                <Text style={{ display: 'block', color: colors.text }}>
                  {formatUptime(sysInfo.uptime)}
                </Text>
              </div>
            )}
          </Space>
        </Card>
      )}
    </div>
  );
}
