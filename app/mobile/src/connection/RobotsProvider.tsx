import {
  createContext,
  useCallback,
  useContext,
  useEffect,
  useState,
  type ReactNode,
} from 'react';
import {
  collection,
  doc,
  onSnapshot,
  getDoc,
  type Timestamp,
} from 'firebase/firestore';
import { db } from '@/firebase';
import { useAuth } from '@/auth/AuthProvider';

// ── Types ────────────────────────────────────────────────────────────────────

export interface RobotRecord {
  rid: string;
  name: string;
  role: 'owner' | 'member';
  tunnelHostname: string;
  /** Base64-encoded Curve25519 public key from Firestore */
  robotPubKeyB64: string;
  /** Decoded public key bytes, ready for NoiseSession */
  robotPubKey: Uint8Array;
  lastSeen: Timestamp | null;
  status: string;
}

interface RobotsContextValue {
  robots: RobotRecord[];
  loading: boolean;
  refresh: () => void;
}

// ── Context ───────────────────────────────────────────────────────────────────

const RobotsContext = createContext<RobotsContextValue>({
  robots: [],
  loading: true,
  refresh: () => undefined,
});

// ── Provider ──────────────────────────────────────────────────────────────────

export function RobotsProvider({ children }: { children: ReactNode }) {
  const { user } = useAuth();
  const [robots, setRobots] = useState<RobotRecord[]>([]);
  const [loading, setLoading] = useState(true);
  const [refreshKey, setRefreshKey] = useState(0);

  const refresh = useCallback(() => setRefreshKey((k) => k + 1), []);

  useEffect(() => {
    if (!user) {
      setRobots([]);
      setLoading(false);
      return;
    }

    setLoading(true);
    const userRobotsRef = collection(db, 'users', user.uid, 'robots');

    const unsub = onSnapshot(
      userRobotsRef,
      async (snap) => {
        const records: RobotRecord[] = [];

        await Promise.all(
          snap.docs.map(async (userRobotDoc) => {
            const rid = userRobotDoc.id;
            const userRobotData = userRobotDoc.data() as {
              name?: string;
              role?: 'owner' | 'member';
              addedAt?: Timestamp;
            };

            // Fetch the canonical robot document for tunnelHostname + pubKey
            try {
              const robotSnap = await getDoc(doc(db, 'robots', rid));
              if (!robotSnap.exists()) return;

              const robotData = robotSnap.data() as {
                tunnelHostname?: string;
                robotPubKey?: string;
                lastSeen?: Timestamp;
                status?: string;
              };

              const pubKeyB64 = robotData.robotPubKey ?? '';
              const pubKeyBytes = base64ToBytes(pubKeyB64);

              records.push({
                rid,
                name: userRobotData.name ?? rid,
                role: userRobotData.role ?? 'member',
                tunnelHostname:
                  robotData.tunnelHostname ?? `r-${rid}.tunnel.mowgli.garden`,
                robotPubKeyB64: pubKeyB64,
                robotPubKey: pubKeyBytes,
                lastSeen: robotData.lastSeen ?? null,
                status: robotData.status ?? 'unknown',
              });
            } catch {
              // Robot doc unreadable (permissions) — skip silently
            }
          }),
        );

        // Sort: owner entries first, then alphabetically by name
        records.sort((a, b) => {
          if (a.role === 'owner' && b.role !== 'owner') return -1;
          if (b.role === 'owner' && a.role !== 'owner') return 1;
          return a.name.localeCompare(b.name);
        });

        setRobots(records);
        setLoading(false);
      },
      () => {
        // Firestore error (offline / permissions) — surface as empty list
        setLoading(false);
      },
    );

    return unsub;
  }, [user, refreshKey]);

  return (
    <RobotsContext.Provider value={{ robots, loading, refresh }}>
      {children}
    </RobotsContext.Provider>
  );
}

export function useRobots(): RobotsContextValue {
  return useContext(RobotsContext);
}

// ── Helpers ───────────────────────────────────────────────────────────────────

function base64ToBytes(b64: string): Uint8Array {
  try {
    const binary = atob(b64);
    const bytes = new Uint8Array(binary.length);
    for (let i = 0; i < binary.length; i++) {
      bytes[i] = binary.charCodeAt(i);
    }
    return bytes;
  } catch {
    // Return zero-filled key — handshake will fail gracefully with an auth error
    return new Uint8Array(32);
  }
}
