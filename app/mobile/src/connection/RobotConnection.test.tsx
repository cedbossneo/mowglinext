import { describe, it, expect, beforeEach } from 'vitest';
import { render, screen, act } from '@testing-library/react';
import { createElement, type ReactNode } from 'react';
import {
  RobotConnectionProvider,
  useRobotConnection,
  evictSession,
  type ConnectionState,
} from './RobotConnection';

// ── Helpers ───────────────────────────────────────────────────────────────────

const FAKE_PUB_KEY = new Uint8Array(32); // all-zeros — handshake will fail gracefully

function makeWrapper(rid: string) {
  return function Wrapper({ children }: { children: ReactNode }) {
    return createElement(RobotConnectionProvider, {
      rid,
      tunnelHostname: 'r-test.tunnel.example.com',
      robotPubKey: FAKE_PUB_KEY,
      children,
    });
  };
}

// Probe component — renders current connection state as a data-testid span
function StateProbe() {
  const { state } = useRobotConnection();
  return createElement('span', { 'data-testid': 'state' }, state);
}

// ── Tests ─────────────────────────────────────────────────────────────────────

describe('RobotConnectionProvider', () => {
  beforeEach(() => {
    evictSession('test-robot-01');
    evictSession('test-robot-02');
  });

  it('renders children without crashing', async () => {
    await act(async () => {
      render(
        createElement(
          makeWrapper('test-robot-01'),
          null,
          createElement('span', { 'data-testid': 'child' }, 'hello'),
        ),
      );
    });

    expect(screen.getByTestId('child')).toBeInTheDocument();
  });

  it('initial state is a valid ConnectionState value', async () => {
    await act(async () => {
      render(
        createElement(makeWrapper('test-robot-01'), null, createElement(StateProbe)),
      );
    });

    const state = screen.getByTestId('state').textContent as ConnectionState;
    const valid: ConnectionState[] = [
      'connecting',
      'connected',
      'reconnecting',
      'closed',
      'error',
    ];
    expect(valid).toContain(state);
  });

  it('initial state is connecting before WS opens', async () => {
    // MockWebSocket never auto-fires onopen, so the provider stays in 'connecting'
    await act(async () => {
      render(
        createElement(makeWrapper('test-robot-02'), null, createElement(StateProbe)),
      );
    });

    expect(screen.getByTestId('state').textContent).toBe('connecting');
  });

  it('evictSession for active or unknown rid does not throw', () => {
    expect(() => evictSession('test-robot-01')).not.toThrow();
    expect(() => evictSession('nonexistent-000')).not.toThrow();
  });

  it('useRobotConnection throws when used outside provider', () => {
    function Bare() {
      try {
        useRobotConnection();
        return createElement('span', null, 'ok');
      } catch {
        return createElement('span', { 'data-testid': 'err' }, 'threw');
      }
    }

    render(createElement(Bare));
    expect(screen.getByTestId('err')).toBeInTheDocument();
  });
});

describe('evictSession', () => {
  it('is idempotent — calling twice does not throw', () => {
    evictSession('robot-idempotent-test');
    expect(() => evictSession('robot-idempotent-test')).not.toThrow();
  });
});
