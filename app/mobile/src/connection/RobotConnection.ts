/**
 * RobotConnection manages a single NoiseSession per robot ID.
 * Sessions are cached in a module-level map so navigating between tabs
 * does not tear down and recreate the tunnel.
 */

import {
  createContext,
  useCallback,
  useContext,
  useEffect,
  useRef,
  useState,
  createElement,
  type ReactNode,
} from 'react';
import { NoiseSession, type HandshakeReply, type NoiseStream } from './NoiseClient';
import { useAuth } from '@/auth/AuthProvider';

// ── Types ────────────────────────────────────────────────────────────────────

export type ConnectionState =
  | 'connecting'
  | 'connected'
  | 'reconnecting'
  | 'closed'
  | 'error';

export interface RobotConnectionValue {
  state: ConnectionState;
  reply: HandshakeReply | null;
  request<T>(method: string, path: string, body?: unknown): Promise<T>;
  subscribe<T>(path: string, cb: (msg: T) => void): () => void;
  session: NoiseSession | null;
}

export interface RobotConnectionProviderProps {
  rid: string;
  tunnelHostname: string;
  robotPubKey: Uint8Array;
  children: ReactNode;
}

// ── Session cache ─────────────────────────────────────────────────────────────
// One NoiseSession per robotId, shared across remounts of RobotShell.

interface CachedSession {
  session: NoiseSession;
  tunnelHostname: string;
}

const sessionCache = new Map<string, CachedSession>();

// ── Context ───────────────────────────────────────────────────────────────────

const RobotConnectionContext = createContext<RobotConnectionValue | null>(null);

export function RobotConnectionProvider({
  rid,
  tunnelHostname,
  robotPubKey,
  children,
}: RobotConnectionProviderProps): ReturnType<typeof createElement> {
  const { idToken } = useAuth();
  const [state, setState] = useState<ConnectionState>('connecting');
  const [reply, setReply] = useState<HandshakeReply | null>(null);
  const sessionRef = useRef<NoiseSession | null>(null);

  useEffect(() => {
    const cached = sessionCache.get(rid);

    let session: NoiseSession;
    if (cached && cached.tunnelHostname === tunnelHostname) {
      session = cached.session;
      const existing = session.getHandshakeReply();
      if (existing) {
        setReply(existing);
        setState(session.getState() === 'transport' ? 'connected' : 'reconnecting');
      }
    } else {
      // Evict stale session for this rid if hostname changed
      if (cached) {
        cached.session.close();
      }

      session = new NoiseSession({
        wsUrl: `wss://${tunnelHostname}`,
        peerStaticPub: robotPubKey,
        idToken,
        appVersion: '0.1.0',
      });
      sessionCache.set(rid, { session, tunnelHostname });

      session.on('connected', (r) => {
        setReply(r);
        setState('connected');
      });
      session.on('reconnecting', () => setState('reconnecting'));
      session.on('closed', () => setState('closed'));
      session.on('error', () => setState('error'));

      session.connect().catch(() => {
        setState('error');
      });
    }

    sessionRef.current = session;

    return () => {
      // Intentionally keep session alive in cache across tab navigation.
      // Session is only destroyed via evictSession() or app close.
    };
  }, [rid, tunnelHostname, robotPubKey, idToken]);

  const request = useCallback(
    async <T,>(method: string, path: string, body?: unknown): Promise<T> => {
      const s = sessionRef.current;
      if (!s) throw new Error('No active session for robot ' + rid);

      let bodyBytes: Uint8Array | undefined;
      const headers: Record<string, string> = {};

      if (body !== undefined) {
        const json = JSON.stringify(body);
        bodyBytes = new TextEncoder().encode(json);
        headers['Content-Type'] = 'application/json';
      }

      const res = await s.request(method, path, headers, bodyBytes);

      if (res.status >= 400) {
        const msg = new TextDecoder().decode(res.body);
        throw new Error(`HTTP ${res.status}: ${msg}`);
      }

      if (res.body.length === 0) return undefined as T;
      return JSON.parse(new TextDecoder().decode(res.body)) as T;
    },
    [rid],
  );

  const subscribe = useCallback(
    <T,>(path: string, cb: (msg: T) => void): (() => void) => {
      const s = sessionRef.current;
      if (!s) return () => undefined;

      let stream: NoiseStream | null = null;
      let cancelled = false;

      s.openStream(path)
        .then((str) => {
          if (cancelled) {
            str.close();
            return;
          }
          stream = str;
          str.on('data', ({ data }) => {
            try {
              const text = new TextDecoder().decode(data);
              if (text.length === 0) return; // wsa ack empty payload
              const parsed = JSON.parse(text) as T;
              cb(parsed);
            } catch {
              // Ignore malformed frames
            }
          });
        })
        .catch(() => {
          // Session error is surfaced via the state field
        });

      return () => {
        cancelled = true;
        stream?.close();
      };
    },
    [],
  );

  const value: RobotConnectionValue = {
    state,
    reply,
    request,
    subscribe,
    session: sessionRef.current,
  };

  return createElement(RobotConnectionContext.Provider, { value }, children);
}

export function useRobotConnection(): RobotConnectionValue {
  const ctx = useContext(RobotConnectionContext);
  if (!ctx) {
    throw new Error('useRobotConnection must be used within RobotConnectionProvider');
  }
  return ctx;
}

/** Evict and close a session from the cache (e.g. after robot is deleted or unpaired) */
export function evictSession(rid: string): void {
  const cached = sessionCache.get(rid);
  if (cached) {
    cached.session.close();
    sessionCache.delete(rid);
  }
}
