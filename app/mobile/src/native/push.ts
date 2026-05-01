import {
  PushNotifications,
  type Token,
  type PushNotificationSchema,
} from '@capacitor/push-notifications';
import { httpsCallable } from 'firebase/functions';
import { Capacitor } from '@capacitor/core';
import { functions } from '@/firebase';
import { prefsGet, prefsSet } from './preferences';

const LAST_TOKEN_KEY = 'mowgli-last-fcm-token';

/**
 * Register for push notifications and upload the FCM token via the
 * registerFcmToken Cloud Function. Re-registration is skipped when the
 * token is unchanged since the last successful upload.
 *
 * Call this once after authentication + robotId are available.
 */
export async function registerPush(robotId: string): Promise<void> {
  if (!Capacitor.isNativePlatform()) {
    // Web push via FCM VAPID is a separate flow; skip native registration
    return;
  }

  try {
    const { receive } = await PushNotifications.checkPermissions();
    if (receive === 'prompt' || receive === 'prompt-with-rationale') {
      const { receive: granted } = await PushNotifications.requestPermissions();
      if (granted !== 'granted') return;
    } else if (receive !== 'granted') {
      return;
    }

    await PushNotifications.register();

    await PushNotifications.addListener('registration', async (token: Token) => {
      const fcmToken = token.value;
      const lastToken = await prefsGet<string>(LAST_TOKEN_KEY);
      if (lastToken === fcmToken) return;

      try {
        const fn = httpsCallable(functions, 'registerFcmToken');
        await fn({
          robotId,
          fcmToken,
          platform: Capacitor.getPlatform(),
        });
        await prefsSet(LAST_TOKEN_KEY, fcmToken);
      } catch {
        // Non-fatal — will retry on next app launch
      }
    });

    await PushNotifications.addListener(
      'registrationError',
      (_err: { error: string }) => {
        // Push registration failed — notifications unavailable this session
      },
    );

    await PushNotifications.addListener(
      'pushNotificationReceived',
      (_notification: PushNotificationSchema) => {
        // Foreground notification received — OS handles display for MVP.
        // Add custom in-app banner here if required.
      },
    );
  } catch {
    // Push notifications not supported on this device/OS — ignore
  }
}

/**
 * Clear the cached FCM token. Call on sign-out so the next sign-in
 * forces a fresh token upload.
 */
export async function clearCachedPushToken(): Promise<void> {
  await prefsSet(LAST_TOKEN_KEY, null);
}
