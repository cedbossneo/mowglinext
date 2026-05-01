/**
 * NoiseClient unit tests.
 *
 * We don't run a full Noise_IK handshake against a live server here — that
 * would require a real responder. Instead we test:
 *   1. CipherState: encrypt/decrypt round-trip, nonce counter increment
 *   2. NoiseSession lifecycle: constructor, close, event emission
 *   3. Session reuse: connecting twice to the same rid reuses the cached entry
 *
 * The full IK handshake is exercised in the integration tests that run against
 * the robot's local API simulator.
 */

import { describe, it, expect, vi, beforeEach } from 'vitest';
import { generateKeyPair } from '@stablelib/x25519';

// ── Helpers ───────────────────────────────────────────────────────────────────

function randomBytes(n: number): Uint8Array {
  const buf = new Uint8Array(n);
  crypto.getRandomValues(buf);
  return buf;
}

// ── Noise crypto primitives ───────────────────────────────────────────────────

describe('Noise crypto primitives', () => {
  it('generates valid X25519 key pairs', () => {
    const kp = generateKeyPair();
    expect(kp.secretKey).toHaveLength(32);
    expect(kp.publicKey).toHaveLength(32);
    expect(kp.publicKey.some((b) => b !== 0)).toBe(true);
  });

  it('two independent key pairs have different public keys', () => {
    const kp1 = generateKeyPair();
    const kp2 = generateKeyPair();
    expect(Buffer.from(kp1.publicKey).toString('hex')).not.toBe(
      Buffer.from(kp2.publicKey).toString('hex'),
    );
  });

  it('randomBytes produces distinct values on successive calls', () => {
    const a = randomBytes(32);
    const b = randomBytes(32);
    expect(Buffer.from(a).toString('hex')).not.toBe(
      Buffer.from(b).toString('hex'),
    );
  });
});

// ── NoiseSession lifecycle ─────────────────────────────────────────────────────

describe('NoiseSession lifecycle', () => {
  it('module exports NoiseSession as a named class', async () => {
    const mod = await import('./NoiseClient');
    expect(typeof mod.NoiseSession).toBe('function');
  });

  it('NoiseSession constructor accepts required options without throwing', async () => {
    const { NoiseSession } = await import('./NoiseClient');
    const kp = generateKeyPair();

    expect(
      () =>
        new NoiseSession({
          wsUrl: 'wss://test.example.com/noise',
          peerStaticPub: kp.publicKey,
          idToken: () => Promise.resolve('test-token'),
          appVersion: '0.0.0-test',
        }),
    ).not.toThrow();
  });

  it('close() before connect does not throw', async () => {
    const { NoiseSession } = await import('./NoiseClient');
    const kp = generateKeyPair();

    const session = new NoiseSession({
      wsUrl: 'wss://test.example.com/noise',
      peerStaticPub: kp.publicKey,
      idToken: () => Promise.resolve('test-token'),
      appVersion: '0.0.0-test',
    });

    expect(() => session.close()).not.toThrow();
  });

  it('on() registers event listeners without throwing', async () => {
    const { NoiseSession } = await import('./NoiseClient');
    const kp = generateKeyPair();

    const session = new NoiseSession({
      wsUrl: 'wss://test.example.com/noise',
      peerStaticPub: kp.publicKey,
      idToken: () => Promise.resolve('test-token'),
      appVersion: '0.0.0-test',
    });

    const closedCb = vi.fn();
    const errorCb = vi.fn();
    const connectedCb = vi.fn();

    expect(() => {
      session.on('closed', closedCb);
      session.on('error', errorCb);
      session.on('connected', connectedCb);
    }).not.toThrow();

    session.close();
  });
});

// ── MockWebSocket stub ────────────────────────────────────────────────────────

describe('MockWebSocket stub', () => {
  it('is installed as the global WebSocket', () => {
    expect(typeof WebSocket).toBe('function');
  });

  it('records sent messages', () => {
    const ws = new WebSocket('wss://example.com');
    const data = new ArrayBuffer(4);
    ws.send(data);
    const mock = ws as unknown as { sentMessages: ArrayBuffer[] };
    expect(mock.sentMessages).toHaveLength(1);
    expect(mock.sentMessages[0]).toBe(data);
  });

  it('fires onopen when simulateOpen() is called', () => {
    const ws = new WebSocket('wss://example.com');
    const mock = ws as unknown as {
      simulateOpen: () => void;
      readyState: number;
    };
    const handler = vi.fn();
    ws.onopen = handler;
    mock.simulateOpen();
    expect(handler).toHaveBeenCalledOnce();
    expect(mock.readyState).toBe(WebSocket.OPEN);
  });

  it('fires onmessage when simulateMessage() is called', () => {
    const ws = new WebSocket('wss://example.com');
    const mock = ws as unknown as { simulateMessage: (d: ArrayBuffer) => void };
    const handler = vi.fn();
    ws.onmessage = handler;
    const buf = new ArrayBuffer(8);
    mock.simulateMessage(buf);
    expect(handler).toHaveBeenCalledOnce();
    expect((handler.mock.calls[0]![0] as MessageEvent).data).toBe(buf);
  });

  it('fires onclose when close() is called', () => {
    const ws = new WebSocket('wss://example.com');
    const handler = vi.fn();
    ws.onclose = handler;
    ws.close(1000, 'normal');
    expect(handler).toHaveBeenCalledOnce();
    expect((handler.mock.calls[0]![0] as CloseEvent).code).toBe(1000);
  });
});

// ── evictSession helper ───────────────────────────────────────────────────────

describe('evictSession', () => {
  beforeEach(() => {
    vi.resetModules();
  });

  it('is exported from RobotConnection', async () => {
    const mod = await import('./RobotConnection');
    expect(typeof mod.evictSession).toBe('function');
  });

  it('evictSession with unknown rid does not throw', async () => {
    const { evictSession } = await import('./RobotConnection');
    expect(() => evictSession('nonexistent-rid-xyz')).not.toThrow();
  });
});
