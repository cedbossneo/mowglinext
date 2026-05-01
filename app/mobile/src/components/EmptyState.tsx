import type { ReactNode } from 'react';
import { Button, Typography } from 'antd';
import { useThemeMode } from '@/theme/ThemeProvider';

const { Title, Text } = Typography;

interface EmptyStateProps {
  icon?: ReactNode;
  title: string;
  description?: string;
  actionLabel?: string;
  onAction?: () => void;
}

export function EmptyState({
  icon,
  title,
  description,
  actionLabel,
  onAction,
}: EmptyStateProps) {
  const { colors } = useThemeMode();

  return (
    <div
      style={{
        display: 'flex',
        flexDirection: 'column',
        alignItems: 'center',
        justifyContent: 'center',
        padding: '48px 24px',
        gap: 12,
        textAlign: 'center',
      }}
    >
      {icon && (
        <div style={{ fontSize: 48, color: colors.textMuted, marginBottom: 8 }}>
          {icon}
        </div>
      )}
      <Title level={4} style={{ margin: 0, color: colors.text }}>
        {title}
      </Title>
      {description && (
        <Text
          style={{ color: colors.textSecondary, maxWidth: 280, display: 'block' }}
        >
          {description}
        </Text>
      )}
      {actionLabel && onAction && (
        <Button
          type="primary"
          size="large"
          onClick={onAction}
          style={{ marginTop: 8, borderRadius: 12 }}
        >
          {actionLabel}
        </Button>
      )}
    </div>
  );
}
