import { useState } from 'react';
import { Button, Form, Input, Tabs, Typography, Space, Divider, App } from 'antd';
import { GoogleOutlined, AppleOutlined } from '@ant-design/icons';
import { useAuth } from './AuthProvider';
import { useThemeMode } from '@/theme/ThemeProvider';

const { Title, Text } = Typography;

interface EmailForm {
  email: string;
  password: string;
}

export function AuthScreen() {
  const { signInEmail, signUpEmail, signInGoogle, signInApple } = useAuth();
  const { colors } = useThemeMode();
  const { message } = App.useApp();
  const [loading, setLoading] = useState(false);

  async function handleEmailSignIn(values: EmailForm) {
    setLoading(true);
    try {
      await signInEmail(values.email, values.password);
    } catch (err: unknown) {
      message.error(err instanceof Error ? err.message : 'Sign-in failed');
    } finally {
      setLoading(false);
    }
  }

  async function handleEmailSignUp(values: EmailForm) {
    setLoading(true);
    try {
      await signUpEmail(values.email, values.password);
    } catch (err: unknown) {
      message.error(err instanceof Error ? err.message : 'Sign-up failed');
    } finally {
      setLoading(false);
    }
  }

  async function handleGoogle() {
    setLoading(true);
    try {
      await signInGoogle();
    } catch (err: unknown) {
      message.error(err instanceof Error ? err.message : 'Google sign-in failed');
    } finally {
      setLoading(false);
    }
  }

  async function handleApple() {
    setLoading(true);
    try {
      await signInApple();
    } catch (err: unknown) {
      message.error(err instanceof Error ? err.message : 'Apple sign-in failed');
    } finally {
      setLoading(false);
    }
  }

  const emailForm = (onFinish: (v: EmailForm) => Promise<void>) => (
    <Form layout="vertical" onFinish={onFinish} requiredMark={false}>
      <Form.Item
        name="email"
        rules={[{ required: true, type: 'email', message: 'Enter a valid email' }]}
      >
        <Input
          size="large"
          placeholder="Email"
          autoComplete="email"
          style={{ borderRadius: 12 }}
        />
      </Form.Item>
      <Form.Item
        name="password"
        rules={[{ required: true, min: 8, message: 'At least 8 characters' }]}
      >
        <Input.Password
          size="large"
          placeholder="Password"
          autoComplete="current-password"
          style={{ borderRadius: 12 }}
        />
      </Form.Item>
      <Form.Item style={{ marginBottom: 0 }}>
        <Button
          type="primary"
          htmlType="submit"
          block
          size="large"
          loading={loading}
          style={{ borderRadius: 12 }}
        >
          Continue
        </Button>
      </Form.Item>
    </Form>
  );

  return (
    <div
      style={{
        minHeight: '100dvh',
        display: 'flex',
        flexDirection: 'column',
        alignItems: 'center',
        justifyContent: 'center',
        padding: '24px 20px',
        paddingTop: 'calc(24px + env(safe-area-inset-top))',
        paddingBottom: 'calc(24px + env(safe-area-inset-bottom))',
        background: colors.bgBase,
      }}
    >
      <div style={{ width: '100%', maxWidth: 380 }}>
        <Space direction="vertical" size={8} style={{ width: '100%', marginBottom: 32 }}>
          <Title level={2} style={{ margin: 0, color: colors.text }}>
            Mowgli
          </Title>
          <Text style={{ color: colors.textSecondary }}>
            Control your robot mower from anywhere
          </Text>
        </Space>

        <Tabs
          defaultActiveKey="signin"
          items={[
            {
              key: 'signin',
              label: 'Sign in',
              children: emailForm(handleEmailSignIn),
            },
            {
              key: 'signup',
              label: 'Sign up',
              children: emailForm(handleEmailSignUp),
            },
          ]}
        />

        <Divider style={{ color: colors.textMuted }}>or</Divider>

        <Space direction="vertical" style={{ width: '100%' }} size={12}>
          <Button
            block
            size="large"
            icon={<GoogleOutlined />}
            onClick={handleGoogle}
            loading={loading}
            style={{ borderRadius: 12 }}
          >
            Continue with Google
          </Button>
          <Button
            block
            size="large"
            icon={<AppleOutlined />}
            onClick={handleApple}
            loading={loading}
            style={{ borderRadius: 12 }}
          >
            Continue with Apple
          </Button>
        </Space>
      </div>
    </div>
  );
}
