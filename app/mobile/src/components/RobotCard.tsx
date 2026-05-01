import { Card, Typography, Space } from 'antd';
import { RightOutlined } from '@ant-design/icons';
import { useThemeMode } from '@/theme/ThemeProvider';
import { StatusDot, type DotStatus } from './StatusDot';
import { BatteryGauge } from './BatteryGauge';
import type { RobotRecord } from '@/connection/RobotsProvider';

const { Text, Title } = Typography;

interface RobotCardProps {
  robot: RobotRecord;
  onPress: (rid: string) => void;
  batteryPercent?: number;
  charging?: boolean;
}

export function RobotCard({ robot, onPress, batteryPercent, charging }: RobotCardProps) {
  const { colors } = useThemeMode();

  const lastSeenSeconds = robot.lastSeen
    ? Math.floor((Date.now() - robot.lastSeen.toMillis()) / 1000)
    : undefined;

  const dotStatus: DotStatus =
    lastSeenSeconds !== undefined && lastSeenSeconds < 30
      ? 'online'
      : 'offline';

  return (
    <Card
      onClick={() => onPress(robot.rid)}
      style={{
        background: colors.panel,
        borderRadius: 16,
        border: `1px solid ${colors.border}`,
        cursor: 'pointer',
        marginBottom: 12,
      }}
      styles={{ body: { padding: '16px 20px' } }}
    >
      <div style={{ display: 'flex', alignItems: 'center', gap: 12 }}>
        <div style={{ flex: 1, minWidth: 0 }}>
          <Space align="center" size={8}>
            <Title
              level={5}
              style={{
                margin: 0,
                color: colors.text,
                overflow: 'hidden',
                textOverflow: 'ellipsis',
                whiteSpace: 'nowrap',
              }}
            >
              {robot.name}
            </Title>
            <StatusDot status={dotStatus} />
          </Space>
          <Space size={16} style={{ marginTop: 4 }}>
            <Text style={{ fontSize: 12, color: colors.textMuted }}>
              {robot.role === 'owner' ? 'Owner' : 'Member'}
            </Text>
            {batteryPercent !== undefined && (
              <BatteryGauge
                percent={batteryPercent}
                charging={charging}
                size="sm"
              />
            )}
          </Space>
        </div>
        <RightOutlined style={{ color: colors.textMuted, fontSize: 14 }} />
      </div>
    </Card>
  );
}
