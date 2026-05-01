import { useEffect } from 'react';
import { useNavigate } from 'react-router-dom';
import { Button, Input, Space, Typography, Alert, Steps } from 'antd';
import { QrcodeOutlined, CheckCircleOutlined } from '@ant-design/icons';
import { usePairing } from './usePairing';
import { ConfirmCodeDisplay } from '@/components/ConfirmCodeDisplay';
import { useThemeMode } from '@/theme/ThemeProvider';
import { scanBarcode } from '@/native/barcode';
import { impactLight, impactMedium } from '@/native/haptics';

const { Title, Text } = Typography;

export function PairScreen() {
  const navigate = useNavigate();
  const { colors } = useThemeMode();
  const { state, startScanning, onQrScanned, onConfirm, onCancel, setRobotName } =
    usePairing();

  // Auto-navigate to robots list when pairing completes
  useEffect(() => {
    if (state.step === 'done') {
      void impactMedium();
      const t = setTimeout(() => navigate('/robots'), 1500);
      return () => clearTimeout(t);
    }
    return undefined;
  }, [state.step, navigate]);

  async function handleScan() {
    startScanning();
    try {
      const raw = await scanBarcode();
      if (raw) {
        await impactLight();
        await onQrScanned(raw);
      } else {
        onCancel();
      }
    } catch {
      onCancel();
    }
  }

  const stepIndex: number = {
    idle: 0,
    scanning: 0,
    connecting: 1,
    confirm: 2,
    confirming: 2,
    done: 3,
    error: 0,
  }[state.step];

  return (
    <div
      style={{
        minHeight: '100dvh',
        display: 'flex',
        flexDirection: 'column',
        padding: '24px 20px',
        paddingTop: 'calc(24px + env(safe-area-inset-top))',
        paddingBottom: 'calc(24px + env(safe-area-inset-bottom))',
        background: colors.bgBase,
        gap: 24,
      }}
    >
      <div>
        <Title level={3} style={{ margin: 0, color: colors.text }}>
          Pair a robot
        </Title>
        <Text style={{ color: colors.textSecondary }}>
          Open the robot's web wizard on your laptop, then scan the QR code it shows.
        </Text>
      </div>

      <Steps
        current={stepIndex}
        size="small"
        items={[
          { title: 'Scan QR' },
          { title: 'Connect' },
          { title: 'Verify' },
          { title: 'Done' },
        ]}
      />

      {state.error && (
        <Alert
          type="error"
          message={state.error}
          showIcon
          closable
          onClose={onCancel}
        />
      )}

      {(state.step === 'idle' || state.step === 'error') && (
        <Space direction="vertical" size={16} style={{ width: '100%' }}>
          <div>
            <Text
              style={{ color: colors.textSecondary, display: 'block', marginBottom: 8 }}
            >
              Robot name (optional)
            </Text>
            <Input
              size="large"
              placeholder="My Mowgli"
              value={state.robotName}
              onChange={(e) => setRobotName(e.target.value)}
              style={{ borderRadius: 12 }}
            />
          </div>
          <Button
            type="primary"
            size="large"
            block
            icon={<QrcodeOutlined />}
            onClick={() => void handleScan()}
            style={{ borderRadius: 12, height: 52 }}
          >
            Scan QR code
          </Button>
          <Button
            block
            size="large"
            onClick={() => navigate('/robots')}
            style={{ borderRadius: 12 }}
          >
            Cancel
          </Button>
        </Space>
      )}

      {state.step === 'scanning' && (
        <div style={{ textAlign: 'center', padding: '40px 0' }}>
          <QrcodeOutlined style={{ fontSize: 64, color: colors.primary }} />
          <Text
            style={{ display: 'block', marginTop: 16, color: colors.textSecondary }}
          >
            Point your camera at the QR code…
          </Text>
        </div>
      )}

      {state.step === 'connecting' && (
        <div style={{ textAlign: 'center', padding: '40px 0' }}>
          <Text style={{ color: colors.textSecondary }}>Connecting to robot…</Text>
        </div>
      )}

      {(state.step === 'confirm' || state.step === 'confirming') &&
        state.confirmCode && (
          <Space
            direction="vertical"
            size={24}
            style={{ width: '100%', alignItems: 'center' }}
          >
            <Text style={{ color: colors.textSecondary, textAlign: 'center' }}>
              Verify this code matches what's shown in the robot's web wizard:
            </Text>
            <ConfirmCodeDisplay code={state.confirmCode} />
            <Button
              type="primary"
              size="large"
              block
              loading={state.step === 'confirming'}
              onClick={() => void onConfirm()}
              style={{ borderRadius: 12, height: 52 }}
            >
              Codes match — confirm
            </Button>
            <Button
              block
              size="large"
              onClick={onCancel}
              disabled={state.step === 'confirming'}
              style={{ borderRadius: 12 }}
            >
              Cancel
            </Button>
          </Space>
        )}

      {state.step === 'done' && (
        <div style={{ textAlign: 'center', padding: '40px 0' }}>
          <CheckCircleOutlined style={{ fontSize: 64, color: colors.success }} />
          <Title level={4} style={{ color: colors.text, marginTop: 16 }}>
            Robot paired!
          </Title>
          <Text style={{ color: colors.textSecondary }}>
            Redirecting to your robots…
          </Text>
        </div>
      )}
    </div>
  );
}
