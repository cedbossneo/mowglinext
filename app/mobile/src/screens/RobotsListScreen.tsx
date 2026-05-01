import { useNavigate } from 'react-router-dom';
import { Button, Skeleton, Typography } from 'antd';
import { PlusOutlined, RobotOutlined } from '@ant-design/icons';
import { useRobots } from '@/connection/RobotsProvider';
import { RobotCard } from '@/components/RobotCard';
import { EmptyState } from '@/components/EmptyState';
import { useThemeMode } from '@/theme/ThemeProvider';

const { Title } = Typography;

export function RobotsListScreen() {
  const navigate = useNavigate();
  const { colors } = useThemeMode();
  const { robots, loading } = useRobots();

  return (
    <div
      style={{
        minHeight: '100dvh',
        display: 'flex',
        flexDirection: 'column',
        background: colors.bgBase,
        paddingTop: 'env(safe-area-inset-top)',
        paddingBottom: 'calc(16px + env(safe-area-inset-bottom))',
      }}
    >
      {/* Header */}
      <div
        style={{
          display: 'flex',
          alignItems: 'center',
          justifyContent: 'space-between',
          padding: '20px 20px 12px',
        }}
      >
        <Title level={3} style={{ margin: 0, color: colors.text }}>
          My robots
        </Title>
        <Button
          type="primary"
          shape="circle"
          icon={<PlusOutlined />}
          size="large"
          onClick={() => navigate('/pair')}
          aria-label="Pair new robot"
        />
      </div>

      {/* Content */}
      <div style={{ flex: 1, overflowY: 'auto', padding: '0 16px' }}>
        {loading ? (
          <div style={{ paddingTop: 8 }}>
            {[0, 1, 2].map((i) => (
              <Skeleton
                key={i}
                active
                paragraph={{ rows: 1 }}
                style={{
                  background: colors.panel,
                  borderRadius: 16,
                  padding: 16,
                  marginBottom: 12,
                }}
              />
            ))}
          </div>
        ) : robots.length === 0 ? (
          <EmptyState
            icon={<RobotOutlined />}
            title="No robots yet"
            description="Pair your first Mowgli robot to get started."
            actionLabel="Pair a robot"
            onAction={() => navigate('/pair')}
          />
        ) : (
          <div style={{ paddingTop: 8 }}>
            {robots.map((robot) => (
              <RobotCard
                key={robot.rid}
                robot={robot}
                onPress={(rid) => navigate(`/r/${rid}/dashboard`)}
              />
            ))}
          </div>
        )}
      </div>
    </div>
  );
}
