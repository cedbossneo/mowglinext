import type { CapacitorConfig } from '@capacitor/cli';

const config: CapacitorConfig = {
  appId: 'garden.mowgli.app',
  appName: 'Mowgli',
  webDir: 'dist',
  server: {
    allowNavigation: [
      '*.tunnel.mowgli.garden',
      'mowgli.local',
      '*.local',
      '192.168.*',
    ],
  },
  plugins: {
    PushNotifications: {
      presentationOptions: ['badge', 'sound', 'alert'],
    },
    SplashScreen: {
      launchShowDuration: 0,
    },
  },
};

export default config;
