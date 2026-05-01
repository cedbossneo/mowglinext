import { useState } from 'react';
import { useParams, useNavigate } from 'react-router-dom';
import {
  Button,
  Input,
  Typography,
  Space,
  Divider,
  App,
  Switch,
} from 'antd';
import { LogoutOutlined, UserAddOutlined } from '@ant-design/icons';
import { doc, updateDoc } from 'firebase/firestore';
import { httpsCallable } from 'firebase/functions';
import { db, functions } from '@/firebase';
import { useAuth } from '@/auth/AuthProvider';
import { useRobots } from '@/connection/RobotsProvider';
import { useThemeMode } from '@/theme/ThemeProvider';
import { clearCachedPushToken } from '@/native/push';
import { evictSession } from '@/connection/RobotConnection';

const { Title, Text } = Typography;

export function SettingsScreen() {
  const { rid } = useParams<{ rid: string }>();
  const navigate = useNavigate();
  const { colors, mode, toggleMode } = useThemeMode();
  const { user, signOut } = useAuth();
  const { robots, refresh } = useRobots();
  const { message } = App.useApp();

  const robot = robots.find((r) => r.rid === rid);
  const isOwner = robot?.role === 'owner';

  const [robotName, setRobotName] = useState(robot?.name ?? '');
  const [savingName, setSavingName] = useState(false);
  const [inviteEmail, setInviteEmail] = useState('');
  const [inviting, setInviting] = useState(false);

  async function handleSaveName() {
    if (!rid || !robotName.trim()) return;
    setSavingName(true);
    try {
      await updateDoc(doc(db, 'robots', rid), { name: robotName.trim() });
      refresh();
      message.success('Robot name updated');
    } catch (err) {
      message.error(err instanceof Error ? err.message : 'Failed to update name');
    } finally {
      setSavingName(false);
    }
  }

  async function handleInvite() {
    if (!rid || !inviteEmail.trim()) return;
    setInviting(true);
    try {
      const fn = httpsCallable(functions, 'inviteUser');
      await fn({ robotId: rid, inviteeEmail: inviteEmail.trim() });
      setInviteEmail('');
      message.success('Invitation sent');
    } catch (err) {
      message.error(err instanceof Error ? err.message : 'Failed to invite user');
    } finally {
      setInviting(false);
    }
  }

  async function handleSignOut() {
    await clearCachedPushToken();
    if (rid) evictSession(rid);
    await signOut();
    navigate('/auth', { replace: true });
  }

  const sectionLabel = (text: string) => (
    <Text
      style={{
        color: colors.textMuted,
        fontSize: 12,
        display: 'block',
        marginBottom: 10,
        textTransform: 'uppercase',
        letterSpacing: '0.06em',
      }}
    >
      {text}
    </Text>
  );

  return (
    <div
      style={{
        height: '100%',
        overflowY: 'auto',
        padding: '12px 16px',
        background: colors.bgBase,
      }}
    >
      <Title level={4} style={{ color: colors.text, marginBottom: 20 }}>
        Settings
      </Title>

      {/* Robot name — owner only */}
      {isOwner && (
        <section style={{ marginBottom: 24 }}>
          {sectionLabel('Robot')}
          <Space.Compact style={{ width: '100%' }}>
            <Input
              value={robotName}
              onChange={(e) => setRobotName(e.target.value)}
              placeholder="Robot name"
              size="large"
              style={{ borderRadius: '12px 0 0 12px' }}
            />
            <Button
              type="primary"
              size="large"
              loading={savingName}
              onClick={() => void handleSaveName()}
              style={{ borderRadius: '0 12px 12px 0' }}
            >
              Save
            </Button>
          </Space.Compact>
        </section>
      )}

      {/* Invite member — owner only */}
      {isOwner && (
        <section style={{ marginBottom: 24 }}>
          {sectionLabel('Members')}
          <Space.Compact style={{ width: '100%' }}>
            <Input
              value={inviteEmail}
              onChange={(e) => setInviteEmail(e.target.value)}
              placeholder="Invitee email address"
              type="email"
              size="large"
              style={{ borderRadius: '12px 0 0 12px' }}
            />
            <Button
              type="primary"
              size="large"
              icon={<UserAddOutlined />}
              loading={inviting}
              disabled={!inviteEmail.trim()}
              onClick={() => void handleInvite()}
              style={{ borderRadius: '0 12px 12px 0' }}
            >
              Invite
            </Button>
          </Space.Compact>
          <Text
            style={{
              color: colors.textMuted,
              fontSize: 12,
              marginTop: 6,
              display: 'block',
            }}
          >
            The invitee must already have a Mowgli account.
          </Text>
        </section>
      )}

      <Divider style={{ borderColor: colors.border, margin: '4px 0 20px' }} />

      {/* Appearance */}
      <section style={{ marginBottom: 24 }}>
        {sectionLabel('Appearance')}
        <div
          style={{
            display: 'flex',
            alignItems: 'center',
            justifyContent: 'space-between',
            padding: '10px 0',
          }}
        >
          <Text style={{ color: colors.text }}>Dark mode</Text>
          <Switch
            checked={mode === 'dark'}
            onChange={toggleMode}
          />
        </div>
      </section>

      <Divider style={{ borderColor: colors.border, margin: '4px 0 20px' }} />

      {/* Account */}
      <section style={{ marginBottom: 24 }}>
        {sectionLabel('Account')}
        {user && (
          <Text
            style={{
              color: colors.textSecondary,
              display: 'block',
              marginBottom: 14,
              fontSize: 14,
            }}
          >
            {user.email ?? user.displayName ?? 'Signed in'}
          </Text>
        )}
        <Button
          block
          size="large"
          danger
          icon={<LogoutOutlined />}
          onClick={() => void handleSignOut()}
          style={{ borderRadius: 12 }}
        >
          Sign out
        </Button>
      </section>
    </div>
  );
}
