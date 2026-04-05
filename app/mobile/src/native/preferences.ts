import { Preferences } from "@capacitor/preferences";
import type { ConnectionState } from "@mowglinext/common";

const CONN_KEY = "mowglinext_connection";
const SAVED_CONNS_KEY = "mowglinext_saved_connections";

interface SavedConn {
  name: string;
  host: string;
  isCloud: boolean;
}

export async function getSavedConnection(): Promise<ConnectionState | null> {
  try {
    const { value } = await Preferences.get({ key: CONN_KEY });
    return value ? JSON.parse(value) : null;
  } catch {
    return null;
  }
}

export async function saveConnection(
  info: SavedConn,
  state: ConnectionState
): Promise<void> {
  await Preferences.set({ key: CONN_KEY, value: JSON.stringify(state) });

  // Also save to saved connections list
  const existing = await getSavedConnections();
  const filtered = existing.filter((c) => c.host !== info.host);
  filtered.unshift(info);
  await Preferences.set({
    key: SAVED_CONNS_KEY,
    value: JSON.stringify(filtered.slice(0, 10)),
  });
}

export async function getSavedConnections(): Promise<SavedConn[]> {
  try {
    const { value } = await Preferences.get({ key: SAVED_CONNS_KEY });
    return value ? JSON.parse(value) : [];
  } catch {
    return [];
  }
}

export async function clearSavedConnection(): Promise<void> {
  await Preferences.remove({ key: CONN_KEY });
}
