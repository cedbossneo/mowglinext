import React from "react";
import ReactDOM from "react-dom/client";
import { App as AntApp, ConfigProvider, theme } from "antd";
import { ThemeProvider, useThemeMode } from "@mowglinext/common";
import { MobileConnectionProvider } from "./connection/ConnectionContext";
import { App } from "./App";

function ThemedApp() {
  const { mode, colors } = useThemeMode();
  return (
    <ConfigProvider
      theme={{
        algorithm: mode === "dark" ? theme.darkAlgorithm : theme.defaultAlgorithm,
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
          fontFamily: "-apple-system, BlinkMacSystemFont, 'Segoe UI', sans-serif",
        },
        components: {
          Card: { colorBorderSecondary: mode === "dark" ? "transparent" : colors.border },
          Button: { borderRadius: 8 },
        },
      }}
    >
      <AntApp style={{ height: "100%" }}>
        <App />
      </AntApp>
    </ConfigProvider>
  );
}

ReactDOM.createRoot(document.getElementById("root")!).render(
  <React.StrictMode>
    <ThemeProvider>
      <MobileConnectionProvider>
        <ThemedApp />
      </MobileConnectionProvider>
    </ThemeProvider>
  </React.StrictMode>
);
