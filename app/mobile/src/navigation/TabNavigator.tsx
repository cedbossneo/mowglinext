import type { ReactNode } from "react";
import { useNavigate, useLocation } from "react-router-dom";
import { useThemeMode } from "@mowglinext/common";
import {
  HomeOutlined,
  EnvironmentOutlined,
  ControlOutlined,
  ClockCircleOutlined,
  SettingOutlined,
} from "@ant-design/icons";
import { TABS } from "./routes";

const TAB_ICONS: Record<string, ReactNode> = {
  "/dashboard": <HomeOutlined />,
  "/map": <EnvironmentOutlined />,
  "/control": <ControlOutlined />,
  "/schedule": <ClockCircleOutlined />,
  "/settings": <SettingOutlined />,
};

export function TabNavigator({ children }: { children: ReactNode }) {
  const navigate = useNavigate();
  const location = useLocation();
  const { colors } = useThemeMode();

  return (
    <div style={{ height: "100%", display: "flex", flexDirection: "column" }}>
      <div style={{ flex: 1, overflow: "hidden" }}>{children}</div>
      <div
        style={{
          display: "flex",
          justifyContent: "space-around",
          alignItems: "center",
          height: 56,
          borderTop: `1px solid ${colors.border}`,
          background: colors.bgCard,
          paddingBottom: "env(safe-area-inset-bottom, 0px)",
          position: "fixed",
          bottom: 0,
          left: 0,
          right: 0,
          zIndex: 100,
        }}
      >
        {TABS.map((tab) => {
          const isActive = location.pathname === tab.path;
          return (
            <div
              key={tab.path}
              onClick={() => navigate(tab.path)}
              style={{
                display: "flex",
                flexDirection: "column",
                alignItems: "center",
                cursor: "pointer",
                padding: "6px 12px",
                color: isActive ? colors.primary : colors.muted,
                fontSize: 20,
                transition: "color 0.2s",
              }}
            >
              {TAB_ICONS[tab.path]}
              <span style={{ fontSize: 10, marginTop: 2 }}>{tab.label}</span>
            </div>
          );
        })}
      </div>
    </div>
  );
}
