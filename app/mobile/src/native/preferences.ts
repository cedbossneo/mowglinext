import { Preferences } from '@capacitor/preferences';

/**
 * Thin typed wrapper around @capacitor/preferences.
 * All values are serialized to/from JSON strings so any JSON-safe type
 * can be stored without callers managing serialization.
 */

export async function prefsGet<T>(key: string): Promise<T | null> {
  try {
    const { value } = await Preferences.get({ key });
    if (value === null || value === undefined) return null;
    return JSON.parse(value) as T;
  } catch {
    return null;
  }
}

export async function prefsSet<T>(key: string, value: T): Promise<void> {
  try {
    await Preferences.set({ key, value: JSON.stringify(value) });
  } catch {
    // Storage unavailable (e.g. private browsing mode) — ignore
  }
}

export async function prefsRemove(key: string): Promise<void> {
  try {
    await Preferences.remove({ key });
  } catch {
    // ignore
  }
}
