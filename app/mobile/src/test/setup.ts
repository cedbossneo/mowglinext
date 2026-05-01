import '@testing-library/jest-dom/vitest';

// ── Stub Capacitor plugins ────────────────────────────────────────────────────
// jsdom does not provide native bridge; stub the modules used in tests.

vi.mock('@capacitor/preferences', () => ({
  Preferences: {
    get: vi.fn().mockResolvedValue({ value: null }),
    set: vi.fn().mockResolvedValue(undefined),
    remove: vi.fn().mockResolvedValue(undefined),
  },
}));

vi.mock('@capacitor/haptics', () => ({
  Haptics: {
    impact: vi.fn().mockResolvedValue(undefined),
    notification: vi.fn().mockResolvedValue(undefined),
  },
  ImpactStyle: { Light: 'LIGHT', Medium: 'MEDIUM', Heavy: 'HEAVY' },
  NotificationType: { Success: 'SUCCESS', Warning: 'WARNING', Error: 'ERROR' },
}));

vi.mock('@capacitor/push-notifications', () => ({
  PushNotifications: {
    checkPermissions: vi.fn().mockResolvedValue({ receive: 'granted' }),
    requestPermissions: vi.fn().mockResolvedValue({ receive: 'granted' }),
    register: vi.fn().mockResolvedValue(undefined),
    addListener: vi.fn().mockReturnValue({ remove: vi.fn() }),
  },
}));

vi.mock('@capacitor-mlkit/barcode-scanning', () => ({
  BarcodeScanner: {
    isSupported: vi.fn().mockResolvedValue({ supported: true }),
    checkPermissions: vi.fn().mockResolvedValue({ camera: 'granted' }),
    requestPermissions: vi.fn().mockResolvedValue({ camera: 'granted' }),
    isGoogleBarcodeScannerModuleAvailable: vi
      .fn()
      .mockResolvedValue({ available: true }),
    scan: vi.fn().mockResolvedValue({ barcodes: [] }),
  },
}));

// ── Stub Firebase ─────────────────────────────────────────────────────────────

vi.mock('@/firebase', () => ({
  auth: {},
  db: {},
  functions: {},
}));

// ── Stub AuthProvider ─────────────────────────────────────────────────────────

vi.mock('@/auth/AuthProvider', () => ({
  useAuth: () => ({
    user: { uid: 'test-uid', email: 'test@example.com', displayName: 'Test User' },
    loading: false,
    idToken: () => Promise.resolve('mock-id-token'),
    signOut: vi.fn().mockResolvedValue(undefined),
  }),
  AuthProvider: ({ children }: { children: unknown }) => children,
}));

// ── Stub window.matchMedia (required by Antd responsive utilities) ────────────

Object.defineProperty(window, 'matchMedia', {
  writable: true,
  value: (query: string) => ({
    matches: false,
    media: query,
    onchange: null,
    addListener: vi.fn(),
    removeListener: vi.fn(),
    addEventListener: vi.fn(),
    removeEventListener: vi.fn(),
    dispatchEvent: vi.fn(),
  }),
});

// ── Stub WebSocket ────────────────────────────────────────────────────────────
// NoiseClient uses the global WebSocket constructor; replace with a minimal spy.

class MockWebSocket extends EventTarget {
  static CONNECTING = 0 as const;
  static OPEN = 1 as const;
  static CLOSING = 2 as const;
  static CLOSED = 3 as const;

  readonly url: string;
  readyState: number = MockWebSocket.CONNECTING;
  binaryType: BinaryType = 'arraybuffer';

  onopen: ((ev: Event) => void) | null = null;
  onmessage: ((ev: MessageEvent) => void) | null = null;
  onclose: ((ev: CloseEvent) => void) | null = null;
  onerror: ((ev: Event) => void) | null = null;

  readonly sentMessages: ArrayBuffer[] = [];

  constructor(url: string) {
    super();
    this.url = url;
  }

  send(data: ArrayBuffer) {
    this.sentMessages.push(data);
  }

  close(code?: number, reason?: string) {
    this.readyState = MockWebSocket.CLOSED;
    const ev = new CloseEvent('close', { code: code ?? 1000, reason: reason ?? '' });
    this.onclose?.(ev);
    this.dispatchEvent(ev);
  }

  /** Test helper: simulate server sending binary data */
  simulateMessage(data: ArrayBuffer) {
    const ev = new MessageEvent('message', { data });
    this.onmessage?.(ev);
    this.dispatchEvent(ev);
  }

  /** Test helper: simulate connection established */
  simulateOpen() {
    this.readyState = MockWebSocket.OPEN;
    const ev = new Event('open');
    this.onopen?.(ev);
    this.dispatchEvent(ev);
  }
}

vi.stubGlobal('WebSocket', MockWebSocket);
