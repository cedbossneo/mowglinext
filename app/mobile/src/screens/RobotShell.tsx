import { useEffect } from 'react';
import { Outlet, useNavigate, useParams, useLocation } from 'react-router-dom';
import { Typography } from 'antd';
import {
  DashboardOutlined,
  ControlOutlined,
  EnvironmentOutlined,
  CalendarOutlined,
  SettingOutlined,
} from '@ant-design/icons';
import { useRobots } from '@/connection/RobotsProvider';
import { RobotConnectionProvider, useRobotConnection } from '@/connection/RobotConnection';
import { StatusDot, type DotStatus } from '@/components/StatusDot';
import { useThemeMode } from '@/theme/ThemeProvider';
import { registerPush } from '@/native/push';

const { Text } = Typography;

const TABS = [
  { key: 'dashboard', label: 'Dashboard', icon: <DashboardOutlined /> },
  { key: 'control',   label: 'Control',   icon: <ControlOutlined /> },
  { key: 'map',       label: 'Map',       icon: <EnvironmentOutlined /> },
  { key: 'schedule',  label: 'Schedule',  icon: <CalendarOutlined /> },
  { key: 'settings',  label: 'Settings',  icon: <SettingOutlined /> },
] as const;

type TabKey = (typeof TABS)[number]['key'];

function RobotShellInner() {
  const { rid } = useParams<{ rid: string }>();
  const navigate = useNavigate();
  const location = useLocation();
  const { colors } = useThemeMode();
  const { robots } = useRobots();
  const { state: connState } = useRobotConnection();

  const robot = robots.find((r) => r.rid === rid);

  useEffect(() => {
    if (rid) void registerPush(rid);
  }, [rid]);

  const segments = location.pathname.split('/');
  const activeTab = (segments[segments.length - 1] ?? 'dashboard') as TabKey;

  function handleTabPress(key: TabKey) {
    navigate(`/r/${rid}/${key}`);
  }

  const dotStatus: DotStatus =
    connState === 'connected'
      ? 'online'
      : connState === 'connecting' || connState === 'reconnecting'
        ? 'connecting'
        : 'offline';

  const lastSeenSeconds =
    robot?.lastSeen
      ? Math.floor((Date.now() - robot.lastSeen.toMillis()) / 1000)
      : undefined;

  return (
    <div
      style={{
        height: '100dvh',
        display: 'flex',
        flexDirection: 'column',
        background: colors.bgBase,
        paddingTop: 'env(safe-area-inset-top)',
      }}
    >
      {/* Connection status pill */}
      <div
        style={{
          display: 'flex',
          alignItems: 'center',
          justifyContent: 'space-between',
          padding: '10px 16px 6px',
          flexShrink: 0,
        }}
      >
        <Text
          style={{
            fontWeight: 600,
            fontSize: 16,
            color: colors.text,
            overflow: 'hidden',
            textOverflow: 'ellipsis',
            whiteSpace: 'nowrap',
            maxWidth: '65%',
          }}
        >
          {robot?.name ?? rid}
        </Text>
        <StatusDot
          status={dotStatus}
          showLabel
          lastSeenSeconds={dotStatus === 'offline' ? lastSeenSeconds : undefined}
        />
      </div>

      {/* Page content */}
      <div style={{ flex: 1, overflow: 'hidden', position: 'relative' }}>
        <Outlet />
      </div>

      {/* Bottom tab bar — 56px + safe area inset */}
      <div
        style={{
          display: 'flex',
          borderTop: `1px solid ${colors.border}`,
          background: colors.bgCard,
          flexShrink: 0,
          paddingBottom: 'env(safe-area-inset-bottom)',
        }}
      >
        {TABS.map(({ key, label, icon }) => {
          const active = activeTab === key;
          return (
            <button
              key={key}
              onClick={() => handleTabPress(key)}
              style={{
                flex: 1,
                height: 56,
                display: 'flex',
                flexDirection: 'column',
                alignItems: 'center',
                justifyContent: 'center',
                gap: 2,
                background: 'none',
                border: 'none',
                cursor: 'pointer',
                padding: '6px 4px',
                color: active ? colors.primary : colors.textMuted,
                fontSize: 10,
                fontWeight: active ? 600 : 400,
                transition: 'color 0.15s',
              }}
              aria-label={label}
              aria-current={active ? 'page' : undefined}
            >
              <span style={{ fontSize: 20 }}>{icon}</span>
              <span>{label}</span>
            </button>
          );
        })}
      </div>
    </div>
  );
}

export function RobotShell() {
  const { rid } = useParams<{ rid: string }>();
  const { robots, loading } = useRobots();
  const navigate = useNavigate();

  const robot = robots.find((r) => r.rid === rid);

  useEffect(() => {
    if (!loading && !robot) {
      navigate('/robots', { replace: true });
    }
  }, [loading, robot, navigate]);

  if (!robot) return null;

  return (
    <RobotConnectionProvider
      rid={robot.rid}
      tunnelHostname={robot.tunnelHostname}
      robotPubKey={robot.robotPubKey}
    >
      <RobotShellInner />
    </RobotConnectionProvider>
  );
}
