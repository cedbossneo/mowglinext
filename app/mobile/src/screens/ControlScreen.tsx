import { useState } from 'react';
import { Button, Space, Typography, App } from 'antd';
import {
  PlayCircleOutlined,
  HomeOutlined,
  StopOutlined,
  ToolOutlined,
  WarningOutlined,
} from '@ant-design/icons';
import { useRobotConnection } from '@/connection/RobotConnection';
import { ConfirmModal } from '@/components/ConfirmModal';
import { useThemeMode } from '@/theme/ThemeProvider';
import { impactHeavy, impactMedium, notifyError } from '@/native/haptics';

const { Title, Text } = Typography;

interface ControlAction {
  key: string;
  label: string;
  description: string;
  icon: React.ReactNode;
  danger?: boolean;
  endpoint: string;
}

const ACTIONS: ControlAction[] = [
  {
    key: 'start',
    label: 'Start mowing',
    description: 'The robot will undock and begin autonomous mowing.',
    icon: <PlayCircleOutlined />,
    endpoint: '/api/mowglinext/service/COMMAND_START',
  },
  {
    key: 'home',
    label: 'Return to dock',
    description: 'The robot will stop mowing and navigate back to the dock.',
    icon: <HomeOutlined />,
    endpoint: '/api/mowglinext/service/COMMAND_HOME',
  },
  {
    key: 'manual',
    label: 'Manual mow',
    description: "Enter manual mowing mode. Use the robot's web interface for movement.",
    icon: <ToolOutlined />,
    endpoint: '/api/mowglinext/service/COMMAND_MANUAL_MOW',
  },
  {
    key: 'stop',
    label: 'Stop / pause',
    description: 'Send a return-to-dock command to halt the current operation.',
    icon: <StopOutlined />,
    danger: true,
    endpoint: '/api/mowglinext/service/COMMAND_HOME',
  },
  {
    key: 'reset',
    label: 'Reset emergency',
    description:
      'Clear the latched emergency stop. Only effective when the physical trigger is no longer asserted — firmware is the sole safety authority.',
    icon: <WarningOutlined />,
    danger: true,
    endpoint: '/api/mowglinext/service/COMMAND_RESET_EMERGENCY',
  },
];

export function ControlScreen() {
  const { colors } = useThemeMode();
  const { state, request } = useRobotConnection();
  const { message } = App.useApp();

  const [pendingAction, setPendingAction] = useState<ControlAction | null>(null);
  const [loading, setLoading] = useState(false);

  const offline = state !== 'connected';

  async function executeAction(action: ControlAction) {
    setLoading(true);
    try {
      await impactHeavy();
      await request('POST', action.endpoint, {});
      await impactMedium();
      message.success(`${action.label} sent`);
    } catch (err) {
      await notifyError();
      message.error(
        err instanceof Error ? err.message : `Failed to send ${action.label}`,
      );
    } finally {
      setLoading(false);
      setPendingAction(null);
    }
  }

  return (
    <div
      style={{
        height: '100%',
        overflowY: 'auto',
        padding: '12px 16px',
        background: colors.bgBase,
      }}
    >
      <Title level={4} style={{ color: colors.text, marginBottom: 4 }}>
        Controls
      </Title>

      {offline && (
        <Text
          style={{
            color: colors.textMuted,
            fontSize: 13,
            display: 'block',
            marginBottom: 12,
          }}
        >
          Controls are disabled while offline.
        </Text>
      )}

      <Space direction="vertical" size={12} style={{ width: '100%' }}>
        {ACTIONS.map((action) => (
          <Button
            key={action.key}
            block
            size="large"
            type={action.danger ? 'default' : 'primary'}
            danger={action.danger}
            icon={action.icon}
            disabled={offline}
            loading={loading && pendingAction?.key === action.key}
            onClick={() => setPendingAction(action)}
            style={{
              borderRadius: 14,
              height: 56,
              fontWeight: 600,
              display: 'flex',
              alignItems: 'center',
            }}
          >
            {action.label}
          </Button>
        ))}
      </Space>

      {pendingAction && (
        <ConfirmModal
          open
          title={pendingAction.label}
          description={pendingAction.description}
          confirmLabel={pendingAction.label}
          danger={pendingAction.danger}
          loading={loading}
          onConfirm={() => void executeAction(pendingAction)}
          onCancel={() => setPendingAction(null)}
        />
      )}
    </div>
  );
}
