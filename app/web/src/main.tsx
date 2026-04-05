import React from 'react'
import ReactDOM from 'react-dom/client'
import { createHashRouter, RouterProvider } from "react-router-dom";
import Root from "./routes/root";
import SettingsPage from "./pages/SettingsPage";
import LogsPage from "./pages/LogsPage";
import OpenMowerPage from "./pages/OpenMowerPage";
import MapPage from "./pages/MapPage";
import OnboardingPage from "./pages/OnboardingPage";
import SchedulePage from "./pages/SchedulePage";
import { App, ConfigProvider, theme } from "antd";
import { Spinner, ThemeProvider, useThemeMode } from "@mowglinext/common";
import { WebConnectionProvider } from "./connection/WebConnectionProvider";

const router = createHashRouter([
    {
        path: "/",
        element: <Root />,
        children: [
            { element: <SettingsPage />, path: "/settings" },
            { element: <LogsPage />, path: "/logs" },
            { element: <OpenMowerPage />, path: "/openmower" },
            { element: <MapPage />, path: "/map" },
            { element: <OnboardingPage />, path: "/onboarding" },
            { element: <SchedulePage />, path: "/schedule" },
        ]
    },
]);

function ThemedApp() {
    const { mode, colors } = useThemeMode();
    return (
        <ConfigProvider theme={{
            algorithm: mode === 'dark' ? theme.darkAlgorithm : theme.defaultAlgorithm,
            token: {
                colorPrimary: colors.primary,
                colorSuccess: colors.success,
                colorWarning: colors.warning,
                colorError: colors.danger,
                colorInfo: colors.info,
                colorBgContainer: colors.bgCard,
                colorBgLayout: colors.bgBase,
                colorBorder: colors.border,
                colorText: colors.text,
                colorTextSecondary: colors.textSecondary,
                borderRadius: 12,
                fontFamily: '"DM Sans", "DM Sans Variable", sans-serif',
                fontFamilyCode: '"DM Mono", monospace',
            },
            components: {
                Card: { colorBorderSecondary: mode === 'dark' ? 'transparent' : colors.border },
                Button: { borderRadius: 8 },
                Input: { colorBgContainer: colors.bgElevated, activeBorderColor: colors.accent, hoverBorderColor: colors.accent },
                Select: { colorBgContainer: colors.bgElevated },
            },
        }}>
            <App style={{ height: "100%" }}>
                <React.Suspense fallback={<Spinner />}>
                    <RouterProvider router={router} />
                </React.Suspense>
            </App>
        </ConfigProvider>
    );
}

ReactDOM.createRoot(document.getElementById('root')!).render(
    <React.StrictMode>
        <ThemeProvider>
            <WebConnectionProvider>
                <ThemedApp />
            </WebConnectionProvider>
        </ThemeProvider>
    </React.StrictMode>,
);
