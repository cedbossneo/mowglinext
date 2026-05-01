import { useCallback, useState } from 'react';
import { useAuth } from '@/auth/AuthProvider';

// ── QR payload ────────────────────────────────────────────────────────────────

export interface QrPayload {
  rid: string;
  pub: string;
  tok: string;
  lan: string;
  cf: string;
}

export function parseQrPayload(raw: string): QrPayload {
  let url: URL;
  try {
    url = new URL(raw);
  } catch {
    throw new Error('Invalid QR code: not a URL');
  }

  if (url.protocol !== 'mowgli:' || url.hostname !== 'pair') {
    throw new Error('Invalid QR code: not a mowgli://pair URL');
  }

  const v = url.searchParams.get('v');
  if (v !== '1') throw new Error(`Unsupported QR version: ${v ?? 'missing'}`);

  const rid = url.searchParams.get('rid');
  const pub = url.searchParams.get('pub');
  const tok = url.searchParams.get('tok');
  const lan = url.searchParams.get('lan');
  const cf = url.searchParams.get('cf');

  if (!rid || !pub || !tok || !lan || !cf) {
    throw new Error('Invalid QR code: missing required fields (rid, pub, tok, lan, cf)');
  }

  return { rid, pub, tok, lan, cf };
}

// ── State machine ─────────────────────────────────────────────────────────────

export type PairingStep =
  | 'idle'
  | 'scanning'
  | 'connecting'
  | 'confirm'
  | 'confirming'
  | 'done'
  | 'error';

export interface PairingState {
  step: PairingStep;
  confirmCode: string | null;
  error: string | null;
  payload: QrPayload | null;
  robotName: string;
}

// ── Hook ──────────────────────────────────────────────────────────────────────

export interface UsePairingReturn {
  state: PairingState;
  startScanning: () => void;
  onQrScanned: (raw: string) => Promise<void>;
  onConfirm: () => Promise<void>;
  onCancel: () => void;
  setRobotName: (name: string) => void;
}

const INITIAL_STATE: PairingState = {
  step: 'idle',
  confirmCode: null,
  error: null,
  payload: null,
  robotName: 'My Mowgli',
};

export function usePairing(): UsePairingReturn {
  const { user, idToken } = useAuth();
  const [state, setState] = useState<PairingState>(INITIAL_STATE);

  const startScanning = useCallback(() => {
    setState((s) => ({ ...s, step: 'scanning', error: null }));
  }, []);

  const setRobotName = useCallback((name: string) => {
    setState((s) => ({ ...s, robotName: name }));
  }, []);

  const onQrScanned = useCallback(
    async (raw: string) => {
      let payload: QrPayload;
      try {
        payload = parseQrPayload(raw);
      } catch (err) {
        setState((s) => ({
          ...s,
          step: 'error',
          error: err instanceof Error ? err.message : 'Invalid QR code',
        }));
        return;
      }

      setState((s) => ({ ...s, step: 'connecting', payload, error: null }));

      try {
        if (!user) throw new Error('Not signed in');
        const token = await idToken();

        const res = await fetch(`http://${payload.lan}/api/pair/start`, {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({
            uid: user.uid,
            idToken: token,
            displayName: user.displayName ?? user.email ?? 'Mowgli User',
            setupToken: payload.tok,
          }),
        });

        if (!res.ok) {
          const text = await res.text();
          throw new Error(`Robot rejected pairing (${res.status}): ${text}`);
        }

        const body = (await res.json()) as { confirmCode: string; expiresAt: string };
        setState((s) => ({
          ...s,
          step: 'confirm',
          confirmCode: body.confirmCode,
        }));
      } catch (err) {
        setState((s) => ({
          ...s,
          step: 'error',
          error: err instanceof Error ? err.message : 'Failed to connect to robot',
        }));
      }
    },
    [user, idToken],
  );

  const onConfirm = useCallback(async () => {
    const { payload } = state;
    if (!payload) return;

    setState((s) => ({ ...s, step: 'confirming', error: null }));

    try {
      // POST confirm to robot's local API.
      // The robot is responsible for calling the pairRobot Cloud Function —
      // the mobile app's role ends here (trust model: only the robot can prove
      // possession of its static private key to Cloudflare).
      const res = await fetch(`http://${payload.lan}/api/pair/confirm`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ setupToken: payload.tok }),
      });

      if (!res.ok) {
        const text = await res.text();
        throw new Error(`Confirm failed (${res.status}): ${text}`);
      }

      setState((s) => ({ ...s, step: 'done' }));
    } catch (err) {
      setState((s) => ({
        ...s,
        step: 'error',
        error: err instanceof Error ? err.message : 'Confirmation failed',
      }));
    }
  }, [state]);

  const onCancel = useCallback(() => {
    setState({ ...INITIAL_STATE });
  }, []);

  return { state, startScanning, onQrScanned, onConfirm, onCancel, setRobotName };
}
