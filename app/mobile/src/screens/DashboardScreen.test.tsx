import { describe, it, expect, vi, beforeEach } from 'vitest';
import { render, screen, act } from '@testing-library/react';
import { DashboardScreen } from './DashboardScreen';

// ── Mock dependencies ─────────────────────────────────────────────────────────

const mockSubscribe = vi.fn();
const mockRequest = vi.fn();

vi.mock('@/connection/RobotConnection', () => ({
  useRobotConnection: () => ({
    state: 'connected',
    subscribe: mockSubscribe,
    request: mockRequest,
  }),
}));

vi.mock('@/theme/ThemeProvider', () => ({
  useThemeMode: () => ({
    colors: {
      bgBase: '#0F1210',
      panel: '#1A1F1C',
      bgSubtle: '#222',
      border: '#333',
      text: '#F0F5F1',
      textMuted: '#6B7C72',
      textSecondary: '#A8B8AF',
      primary: '#3EE084',
      primaryBg: '#1A3D2A',
      danger: '#FF4D4F',
      dangerBg: '#2A1010',
      success: '#52C41A',
      amber: '#FAAD14',
      sky: '#40A9FF',
      accentSoft: '#1A3D2A',
    },
    mode: 'dark',
  }),
}));

// ── Helpers ───────────────────────────────────────────────────────────────────

function setupSubscribeMock(
  hlData: Record<string, unknown> = {},
  powerData: Record<string, unknown> = {},
  gpsData: Record<string, unknown> = {},
) {
  mockSubscribe.mockImplementation(
    (path: string, cb: (data: unknown) => void) => {
      if (path.includes('high_level_status')) {
        cb({
          state: 1,
          state_name: 'IDLE',
          battery_voltage: 24.5,
          charging: false,
          emergency: false,
          emergency_reason: '',
          mow_progress: 0,
          current_area: 0,
          current_strip: 0,
          ...hlData,
        });
      } else if (path.includes('power')) {
        cb({
          voltage: 24.3,
          current: 0.5,
          charging: false,
          battery_empty: false,
          ...powerData,
        });
      } else if (path.includes('gps')) {
        cb({
          status: { status: 2 },
          latitude: 48.8566,
          longitude: 2.3522,
          ...gpsData,
        });
      }
      return vi.fn(); // unsubscribe noop
    },
  );
}

// ── Tests ─────────────────────────────────────────────────────────────────────

describe('DashboardScreen', () => {
  beforeEach(() => {
    mockSubscribe.mockReset();
    mockRequest.mockReset();
    mockRequest.mockResolvedValue({ version: '1.2.3', uptime: 3725 });
  });

  it('renders state name when connected and data arrives', async () => {
    setupSubscribeMock();

    await act(async () => {
      render(<DashboardScreen />);
    });

    expect(screen.getByText('IDLE')).toBeInTheDocument();
  });

  it('renders GPS label for RTK fix', async () => {
    setupSubscribeMock();

    await act(async () => {
      render(<DashboardScreen />);
    });

    expect(screen.getByText('RTK / GBAS')).toBeInTheDocument();
  });

  it('renders power voltage', async () => {
    setupSubscribeMock();

    await act(async () => {
      render(<DashboardScreen />);
    });

    expect(screen.getByText(/24\.3 V/)).toBeInTheDocument();
  });

  it('renders system version from request', async () => {
    setupSubscribeMock();

    await act(async () => {
      render(<DashboardScreen />);
    });

    expect(screen.getByText('1.2.3')).toBeInTheDocument();
  });

  it('renders mow progress bar when progress > 0', async () => {
    setupSubscribeMock({ mow_progress: 42 });

    await act(async () => {
      render(<DashboardScreen />);
    });

    expect(screen.getByText('42%')).toBeInTheDocument();
    expect(screen.getByText('Mow progress')).toBeInTheDocument();
  });

  it('renders emergency banner when emergency is active', async () => {
    setupSubscribeMock({ emergency: true, emergency_reason: 'blade stuck' });

    await act(async () => {
      render(<DashboardScreen />);
    });

    expect(screen.getByText(/blade stuck/i)).toBeInTheDocument();
  });

  it('does not render mow progress section when progress is 0', async () => {
    setupSubscribeMock({ mow_progress: 0 });

    await act(async () => {
      render(<DashboardScreen />);
    });

    expect(screen.queryByText('Mow progress')).not.toBeInTheDocument();
  });
});
