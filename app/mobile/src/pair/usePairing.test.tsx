import { describe, it, expect, vi, beforeEach, afterEach } from 'vitest';
import { renderHook, act } from '@testing-library/react';
import { usePairing } from './usePairing';

// ── Stub fetch ────────────────────────────────────────────────────────────────

function makeFetchMock(responses: Array<{ ok: boolean; json: unknown }>) {
  let call = 0;
  return vi.fn().mockImplementation(() => {
    const resp = responses[call++] ?? { ok: false, json: { error: 'no more responses' } };
    return Promise.resolve({
      ok: resp.ok,
      json: () => Promise.resolve(resp.json),
    });
  });
}

// ── QR payload helpers ────────────────────────────────────────────────────────

// 32-byte all-zeros public key, base64-encoded — valid format, invalid crypto
const FAKE_PUB_B64 = btoa(String.fromCharCode(...new Uint8Array(32)));

function makeQrUrl(overrides: Partial<Record<string, string>> = {}): string {
  const params: Record<string, string> = {
    v: '1',
    rid: 'robot-abc123',
    pub: FAKE_PUB_B64,
    tok: 'pairing-token-1',
    lan: '192.0.2.10',
    cf: 'r-robot-abc123.tunnel.example.com',
    ...overrides,
  };
  const qs = new URLSearchParams(params).toString();
  return `mowgli://pair?${qs}`;
}

// ── Tests ─────────────────────────────────────────────────────────────────────

describe('usePairing state machine', () => {
  let originalFetch: typeof fetch;

  beforeEach(() => {
    originalFetch = globalThis.fetch;
  });

  afterEach(() => {
    globalThis.fetch = originalFetch;
    vi.restoreAllMocks();
  });

  it('starts in idle state with empty robotName', () => {
    const { result } = renderHook(() => usePairing());
    expect(result.current.state.step).toBe('idle');
    expect(result.current.state.robotName).toBe('My Mowgli');
    expect(result.current.state.error).toBeNull();
  });

  it('startScanning transitions to scanning', () => {
    const { result } = renderHook(() => usePairing());
    act(() => result.current.startScanning());
    expect(result.current.state.step).toBe('scanning');
  });

  it('onCancel from scanning returns to idle', () => {
    const { result } = renderHook(() => usePairing());
    act(() => result.current.startScanning());
    act(() => result.current.onCancel());
    expect(result.current.state.step).toBe('idle');
  });

  it('setRobotName updates the name field', () => {
    const { result } = renderHook(() => usePairing());
    act(() => result.current.setRobotName('My Mowgli'));
    expect(result.current.state.robotName).toBe('My Mowgli');
  });

  it('invalid QR URL transitions to error state', async () => {
    const { result } = renderHook(() => usePairing());
    act(() => result.current.startScanning());

    await act(async () => {
      await result.current.onQrScanned('https://not-a-mowgli-url.example.com');
    });

    expect(result.current.state.step).toBe('error');
    expect(result.current.state.error).toMatch(/invalid/i);
  });

  it('QR with wrong version transitions to error state', async () => {
    const { result } = renderHook(() => usePairing());
    act(() => result.current.startScanning());

    await act(async () => {
      await result.current.onQrScanned(makeQrUrl({ v: '2' }));
    });

    expect(result.current.state.step).toBe('error');
    expect(result.current.state.error).toBeDefined();
  });

  it('successful pair/start transitions to confirm with a code', async () => {
    globalThis.fetch = makeFetchMock([
      { ok: true, json: { confirmCode: '7319' } },
    ]);

    const { result } = renderHook(() => usePairing());
    act(() => result.current.startScanning());

    await act(async () => {
      await result.current.onQrScanned(makeQrUrl());
    });

    expect(result.current.state.step).toBe('confirm');
    expect(result.current.state.confirmCode).not.toBeNull();
    expect(typeof result.current.state.confirmCode).toBe('string');
  });

  it('failed pair/start transitions to error', async () => {
    globalThis.fetch = makeFetchMock([
      { ok: false, json: { error: 'token expired' } },
    ]);

    const { result } = renderHook(() => usePairing());
    act(() => result.current.startScanning());

    await act(async () => {
      await result.current.onQrScanned(makeQrUrl());
    });

    expect(result.current.state.step).toBe('error');
    expect(result.current.state.error).toBeDefined();
  });

  it('successful pair/confirm transitions to done', async () => {
    globalThis.fetch = makeFetchMock([
      { ok: true, json: { confirmCode: '7319' } }, // pair/start
      { ok: true, json: {} },                           // pair/confirm
    ]);

    const { result } = renderHook(() => usePairing());
    act(() => result.current.startScanning());

    await act(async () => {
      await result.current.onQrScanned(makeQrUrl());
    });

    expect(result.current.state.step).toBe('confirm');

    await act(async () => {
      await result.current.onConfirm();
    });

    expect(result.current.state.step).toBe('done');
  });

  it('failed pair/confirm transitions to error', async () => {
    globalThis.fetch = makeFetchMock([
      { ok: true, json: { confirmCode: '7319' } }, // pair/start
      { ok: false, json: { error: 'code mismatch' } }, // pair/confirm
    ]);

    const { result } = renderHook(() => usePairing());
    act(() => result.current.startScanning());

    await act(async () => {
      await result.current.onQrScanned(makeQrUrl());
    });

    await act(async () => {
      await result.current.onConfirm();
    });

    expect(result.current.state.step).toBe('error');
    expect(result.current.state.error).toBeDefined();
  });
});
