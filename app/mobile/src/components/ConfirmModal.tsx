import { Modal, Typography } from 'antd';
import { useThemeMode } from '@/theme/ThemeProvider';

const { Text } = Typography;

interface ConfirmModalProps {
  open: boolean;
  title: string;
  description?: string;
  confirmLabel?: string;
  cancelLabel?: string;
  danger?: boolean;
  loading?: boolean;
  onConfirm: () => void;
  onCancel: () => void;
}

export function ConfirmModal({
  open,
  title,
  description,
  confirmLabel = 'Confirm',
  cancelLabel = 'Cancel',
  danger = false,
  loading = false,
  onConfirm,
  onCancel,
}: ConfirmModalProps) {
  const { colors } = useThemeMode();

  return (
    <Modal
      open={open}
      title={title}
      okText={confirmLabel}
      cancelText={cancelLabel}
      okButtonProps={{
        danger,
        loading,
        style: { borderRadius: 10 },
      }}
      cancelButtonProps={{ style: { borderRadius: 10 } }}
      onOk={onConfirm}
      onCancel={onCancel}
      centered
      styles={{
        content: {
          background: colors.bgCard,
          borderRadius: 20,
        },
        header: { background: colors.bgCard },
        footer: { background: colors.bgCard },
      }}
    >
      {description && (
        <Text style={{ color: colors.textSecondary }}>{description}</Text>
      )}
    </Modal>
  );
}
