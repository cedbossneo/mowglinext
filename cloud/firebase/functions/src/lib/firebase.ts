import { initializeApp, getApps } from "firebase-admin/app";
import { getFirestore } from "firebase-admin/firestore";
import { getAuth } from "firebase-admin/auth";

// Initialize firebase-admin exactly once. When deployed to Cloud Functions the
// SDK auto-discovers credentials from the runtime service account. During local
// emulator runs set FIREBASE_AUTH_EMULATOR_HOST / FIRESTORE_EMULATOR_HOST
// environment variables before importing this module.
const app =
  getApps().length === 0
    ? initializeApp()
    : getApps()[0]!;

export const db = getFirestore(app);
export const auth = getAuth(app);
export { app };
