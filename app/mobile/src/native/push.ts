import {
  PushNotifications,
  type Token,
  type PushNotificationSchema,
} from '@capacitor/push-notifications';
import { httpsCallable } from 'firebase/functions';
import { Capacitor } from '@capacitor/core';
import { functions } from '@/firebase';
import { prefsGet, prefsSet, prefsRemove } from './preferences';

// Per-robot cache key — registering for robot A then robot B must not
// dedupe to a single upload. Each robot stores its own copy of the token
// in robots/{rid}/devices/{tokenHash}.
const tokenCacheKey = (robotId: string) => `mowgli-fcm-token:${robotId}`;

// The most recent OS-issued FCM token observed in this session. The OS
// only fires the 'registration' event once per process; subsequent calls
// to registerPush(otherRobot) reuse this value to upload for that robot.
let cachedSessionToken: string | null = null;
let listenersInstalled = false;

/**
 * Register for push notifications and upload the FCM token via the
 * registerFcmToken Cloud Function. Idempotent per (robot, token) — safe
 * to call on every robot screen mount.
 */
export async function registerPush(robotId: string): Promise<void> {
  if (!Capacitor.isNativePlatform()) {
    // Web push via FCM VAPID is a separate flow; skip native registration.
    return;
  }
  if (!robotId) return;

  try {
    const { receive } = await PushNotifications.checkPermissions();
    if (receive === 'prompt' || receive === 'prompt-with-rationale') {
      const { receive: granted } = await PushNotifications.requestPermissions();
      if (granted !== 'granted') return;
    } else if (receive !== 'granted') {
      return;
    }

    await PushNotifications.register();

    if (!listenersInstalled) {
      listenersInstalled = true;

      await PushNotifications.addListener('registration', async (token: Token) => {
        cachedSessionToken = token.value;
        await uploadTokenForRobot(robotId, token.value);
      });

      await PushNotifications.addListener(
        'registrationError',
        (_err: { error: string }) => {
          // Registration failed — notifications unavailable this session.
        },
      );

      await PushNotifications.addListener(
        'pushNotificationReceived',
        (_notification: PushNotificationSchema) => {
          // Foreground notification received — OS handles display for MVP.
          // Add custom in-app banner here if required.
        },
      );
    } else if (cachedSessionToken) {
      // Listeners already installed; the OS won't fire 'registration'
      // again this session. Upload the cached token for this robot.
      await uploadTokenForRobot(robotId, cachedSessionToken);
    }
  } catch {
    // Push notifications not supported on this device/OS — ignore.
  }
}

async function uploadTokenForRobot(robotId: string, fcmToken: string): Promise<void> {
  if (!fcmToken) return;
  const cached = await prefsGet<string>(tokenCacheKey(robotId));
  if (cached === fcmToken) return;

  try {
    const fn = httpsCallable(functions, 'registerFcmToken');
    await fn({
      robotId,
      fcmToken,
      platform: Capacitor.getPlatform(),
    });
    await prefsSet(tokenCacheKey(robotId), fcmToken);
  } catch {
    // Non-fatal — will retry on next app launch / robot screen mount.
  }
}

/**
 * Clear cached push state on sign-out. Drops the legacy single-key
 * cache and the in-memory session token; per-robot keys remain in
 * Capacitor Preferences (the corresponding server-side device docs are
 * cleaned up via revokeUser / deleteRobot).
 */
export async function clearCachedPushToken(): Promise<void> {
  cachedSessionToken = null;
  await prefsRemove('mowgli-last-fcm-token');
}
