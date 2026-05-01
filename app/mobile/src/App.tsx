import { useEffect } from 'react';
import {
  createBrowserRouter,
  RouterProvider,
  Navigate,
  Outlet,
  useNavigate,
} from 'react-router-dom';
import { App as AntApp, ConfigProvider, theme as antTheme } from 'antd';
import { useAuth } from '@/auth/AuthProvider';
import { useThemeMode } from '@/theme/ThemeProvider';
import { AuthScreen } from '@/auth/AuthScreen';
import { PairScreen } from '@/pair/PairScreen';
import { RobotsListScreen } from '@/screens/RobotsListScreen';
import { RobotShell } from '@/screens/RobotShell';
import { DashboardScreen } from '@/screens/DashboardScreen';
import { ControlScreen } from '@/screens/ControlScreen';
import { MapScreen } from '@/screens/MapScreen';
import { ScheduleScreen } from '@/screens/ScheduleScreen';
import { SettingsScreen } from '@/screens/SettingsScreen';

// ── Auth guards ───────────────────────────────────────────────────────────────

function RequireAuth() {
  const { user, loading } = useAuth();
  const navigate = useNavigate();

  useEffect(() => {
    if (!loading && !user) {
      void navigate('/auth', { replace: true });
    }
  }, [loading, user, navigate]);

  if (loading) return null;
  if (!user) return null;
  return <Outlet />;
}

// RedirectIfAuthed is the counterpart guard for /auth: once Firebase
// completes sign-in (or restores a cached session), this navigates to the
// robots list so the user does not stay stuck on the auth screen.
function RedirectIfAuthed() {
  const { user, loading } = useAuth();
  const navigate = useNavigate();

  useEffect(() => {
    if (!loading && user) {
      void navigate('/robots', { replace: true });
    }
  }, [loading, user, navigate]);

  if (loading) return null;
  return <AuthScreen />;
}

// ── Root layout — ConfigProvider + AntApp ─────────────────────────────────────

function RootLayout() {
  const { colors, mode } = useThemeMode();

  const algorithm =
    mode === 'dark' ? antTheme.darkAlgorithm : antTheme.defaultAlgorithm;

  return (
    <ConfigProvider
      theme={{
        algorithm,
        token: {
          colorPrimary: colors.primary,
          colorBgBase: colors.bgBase,
          colorTextBase: colors.text,
          colorBorder: colors.border,
          borderRadius: 12,
          fontFamily:
            "-apple-system, BlinkMacSystemFont, 'SF Pro Text', 'Segoe UI', sans-serif",
        },
        components: {
          Button: {
            borderRadius: 12,
          },
          Card: {
            borderRadius: 16,
          },
          Input: {
            borderRadius: 12,
          },
        },
      }}
    >
      <AntApp>
        <Outlet />
      </AntApp>
    </ConfigProvider>
  );
}

// ── Root redirect — depends on auth state ─────────────────────────────────────

function RootRedirect() {
  const { user, loading } = useAuth();
  if (loading) return null;
  return <Navigate to={user ? '/robots' : '/auth'} replace />;
}

// ── Router definition ─────────────────────────────────────────────────────────

const router = createBrowserRouter([
  {
    element: <RootLayout />,
    children: [
      {
        path: '/auth',
        element: <RedirectIfAuthed />,
      },
      {
        element: <RequireAuth />,
        children: [
          {
            path: '/robots',
            element: <RobotsListScreen />,
          },
          {
            path: '/pair',
            element: <PairScreen />,
          },
          {
            path: '/r/:rid',
            element: <RobotShell />,
            children: [
              {
                index: true,
                element: <Navigate to="dashboard" replace />,
              },
              {
                path: 'dashboard',
                element: <DashboardScreen />,
              },
              {
                path: 'control',
                element: <ControlScreen />,
              },
              {
                path: 'map',
                element: <MapScreen />,
              },
              {
                path: 'schedule',
                element: <ScheduleScreen />,
              },
              {
                path: 'settings',
                element: <SettingsScreen />,
              },
            ],
          },
        ],
      },
      {
        path: '/',
        element: <RootRedirect />,
      },
      {
        path: '*',
        element: <Navigate to="/" replace />,
      },
    ],
  },
]);

// ── App export ────────────────────────────────────────────────────────────────

export function App() {
  return <RouterProvider router={router} />;
}
