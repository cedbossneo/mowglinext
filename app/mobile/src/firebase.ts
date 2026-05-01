import { initializeApp, getApps, type FirebaseApp } from 'firebase/app';
import { getAuth, type Auth } from 'firebase/auth';
import { getFirestore, type Firestore } from 'firebase/firestore';
import { getFunctions, type Functions } from 'firebase/functions';

const firebaseConfig = {
  apiKey: import.meta.env.VITE_FIREBASE_API_KEY as string,
  authDomain: import.meta.env.VITE_FIREBASE_AUTH_DOMAIN as string,
  projectId: import.meta.env.VITE_FIREBASE_PROJECT_ID as string,
  storageBucket: import.meta.env.VITE_FIREBASE_STORAGE_BUCKET as string,
  messagingSenderId: import.meta.env.VITE_FIREBASE_MESSAGING_SENDER_ID as string,
  appId: import.meta.env.VITE_FIREBASE_APP_ID as string,
};

// Prevent double-initialization in hot-reload scenarios
const app: FirebaseApp =
  getApps().length > 0 ? getApps()[0]! : initializeApp(firebaseConfig);

export const auth: Auth = getAuth(app);
export const db: Firestore = getFirestore(app);
export const functions: Functions = getFunctions(app, 'us-central1');

// FCM messaging is only available in browser contexts with service worker support.
// On Capacitor native, push is handled by @capacitor/push-notifications instead.
let _messaging: unknown = null;
export function getMessagingIfSupported(): unknown {
  return _messaging;
}

if (typeof window !== 'undefined' && 'serviceWorker' in navigator) {
  import('firebase/messaging')
    .then(({ getMessaging, isSupported }) =>
      isSupported().then((ok) => {
        if (ok) {
          _messaging = getMessaging(app);
        }
      })
    )
    .catch(() => {
      // FCM not available in this environment — native push handles it
    });
}

export { app };
