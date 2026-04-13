// wdyr disabled — @welldone-software/why-did-you-render UMD build incompatible with Vite 8 + React 19
// import './wdyr';
import React from 'react'
import ReactDOM from 'react-dom/client'
import {createHashRouter, RouterProvider,} from "react-router-dom";
import Root from "./routes/root.tsx";
import SettingsPage from "./pages/SettingsPage.tsx";
import LogsPage from "./pages/LogsPage.tsx";
import MowgliNextPage from "./pages/MowgliNextPage.tsx";
import MapPage from "./pages/MapPage.tsx";
import OnboardingPage from "./pages/OnboardingPage.tsx";
import SchedulePage from "./pages/SchedulePage.tsx";
import DiagnosticsPage from "./pages/DiagnosticsPage.tsx";
import StatisticsPage from "./pages/StatisticsPage.tsx";
import {App, ConfigProvider, theme} from "antd";
import {Spinner} from "./components/Spinner.tsx";
import {ThemeProvider, useThemeMode} from "./theme/ThemeContext.tsx";

const router = createHashRouter([
    {
        path: "/",
        element: <Root/>,
        children: [
            {
                element: <SettingsPage/>,
                path: "/settings",
            },
            {
                element: <LogsPage/>,
                path: "/logs",
            },
            {
                element: <MowgliNextPage/>,
                path: "/mowglinext",
            },
            {
                element: <MapPage/>,
                path: "/map",
            },
            {
                element: <OnboardingPage/>,
                path: "/onboarding",
            },
            {
                element: <SchedulePage/>,
                path: "/schedule",
            },
            {
                element: <DiagnosticsPage/>,
                path: "/diagnostics",
            },
            {
                element: <StatisticsPage/>,
                path: "/statistics",
            }
        ]
    },
]);

function ThemedApp() {
    const {mode, colors} = useThemeMode();

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
                Card: {
                    colorBorderSecondary: mode === 'dark' ? 'transparent' : colors.border,
                },
                Button: {
                    borderRadius: 8,
                },
                Input: {
                    colorBgContainer: colors.bgElevated,
                    activeBorderColor: colors.accent,
                    hoverBorderColor: colors.accent,
                },
                Select: {
                    colorBgContainer: colors.bgElevated,
                },
            },
        }}>
            <App style={{height: "100%"}}>
                <React.Suspense fallback={<Spinner/>}>
                    <RouterProvider router={router}/>
                </React.Suspense>
            </App>
        </ConfigProvider>
    );
}

ReactDOM.createRoot(document.getElementById('root')!).render(
  <React.StrictMode>
      <ThemeProvider>
          <ThemedApp/>
      </ThemeProvider>
  </React.StrictMode>,
)
