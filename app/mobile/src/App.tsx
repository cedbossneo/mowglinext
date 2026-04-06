import { BrowserRouter, Routes, Route, Navigate } from "react-router-dom";
import { useConnection } from "@mowglinext/common";
import { ConnectionSetup } from "./connection/ConnectionSetup";
import { TabNavigator } from "./navigation/TabNavigator";
import { DashboardScreen } from "./screens/DashboardScreen";
import { MapScreen } from "./screens/MapScreen";
import { ControlScreen } from "./screens/ControlScreen";
import { ScheduleScreen } from "./screens/ScheduleScreen";
import { SettingsScreen } from "./screens/SettingsScreen";

export function App() {
  const { isConnected } = useConnection();

  if (!isConnected) {
    return <ConnectionSetup />;
  }

  return (
    <BrowserRouter>
      <TabNavigator>
        <Routes>
          <Route path="/" element={<Navigate to="/dashboard" replace />} />
          <Route path="/dashboard" element={<DashboardScreen />} />
          <Route path="/map" element={<MapScreen />} />
          <Route path="/control" element={<ControlScreen />} />
          <Route path="/schedule" element={<ScheduleScreen />} />
          <Route path="/settings" element={<SettingsScreen />} />
        </Routes>
      </TabNavigator>
    </BrowserRouter>
  );
}
