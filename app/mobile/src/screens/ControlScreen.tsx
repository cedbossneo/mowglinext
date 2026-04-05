import { useState, useCallback } from "react";
import { Button, Card, Modal, Space, Typography, message } from "antd";
import { WarningOutlined, StopOutlined, ThunderboltOutlined } from "@ant-design/icons";
import {
  useEmergency,
  useHighLevelStatus,
  useMowerAction,
  useThemeMode,
  useConnection,
} from "@mowglinext/common";
import { Joystick } from "react-joystick-component";
import type { IJoystickUpdateEvent } from "react-joystick-component/build/lib/Joystick";
import { triggerHaptic } from "../native/haptics";

const { Text } = Typography;

export function ControlScreen() {
  const { colors } = useThemeMode();
  const { baseWsUrl, authToken } = useConnection();
  const { highLevelStatus } = useHighLevelStatus();
  const emergency = useEmergency();
  const mowerAction = useMowerAction();
  const [bladeConfirm, setBladeConfirm] = useState(false);

  const isEmergency = highLevelStatus.Emergency ?? emergency.ActiveEmergency ?? false;

  const handleJoystick = useCallback(
    (event: IJoystickUpdateEvent) => {
      if (!baseWsUrl) return;
      // Joystick values: x = -1..1 (left/right), y = -1..1 (back/forward)
      const linear = (event.y ?? 0) * 0.5; // max 0.5 m/s
      const angular = -(event.x ?? 0) * 1.0; // max 1.0 rad/s
      // Publish cmd_vel via WS — would need a direct WS publish mechanism
      // For now, use the API call approach
    },
    [baseWsUrl]
  );

  const handleEmergency = async () => {
    await triggerHaptic("heavy");
    if (isEmergency) {
      await mowerAction("emergency", { Emergency: 0 })();
    } else {
      await mowerAction("emergency", { Emergency: 1 })();
    }
  };

  const handleBladeToggle = () => {
    if (!bladeConfirm) {
      setBladeConfirm(true);
      return;
    }
    setBladeConfirm(false);
    mowerAction("mow_enabled", { MowEnabled: 1, MowDirection: 0 })();
    triggerHaptic("medium");
  };

  const handleBladeOff = async () => {
    setBladeConfirm(false);
    await mowerAction("mow_enabled", { MowEnabled: 0, MowDirection: 0 })();
    await triggerHaptic("medium");
  };

  return (
    <div style={{ padding: 16, paddingBottom: 80, height: "100%", overflowY: "auto" }}>
      <Space direction="vertical" size="middle" style={{ width: "100%" }}>
        {/* Emergency Stop — Big Red Button */}
        <Button
          danger={!isEmergency}
          type="primary"
          size="large"
          block
          icon={<StopOutlined />}
          onClick={handleEmergency}
          style={{
            height: 64,
            fontSize: 18,
            fontWeight: 700,
            background: isEmergency ? colors.warning : colors.danger,
            borderColor: isEmergency ? colors.warning : colors.danger,
          }}
        >
          {isEmergency ? "Clear Emergency" : "EMERGENCY STOP"}
        </Button>

        {/* Joystick */}
        <Card title="Teleop" size="small">
          <div style={{ display: "flex", justifyContent: "center", padding: 16 }}>
            <Joystick
              size={150}
              baseColor={colors.bgElevated}
              stickColor={colors.primary}
              move={handleJoystick}
              stop={() => handleJoystick({ type: "stop", x: 0, y: 0 } as IJoystickUpdateEvent)}
            />
          </div>
          <Text style={{ color: colors.textSecondary, textAlign: "center", display: "block" }}>
            Drag to control movement
          </Text>
        </Card>

        {/* Blade Control */}
        <Card title="Blade" size="small">
          <Space style={{ width: "100%" }}>
            <Button
              icon={<ThunderboltOutlined />}
              type={bladeConfirm ? "primary" : "default"}
              danger={bladeConfirm}
              onClick={handleBladeToggle}
            >
              {bladeConfirm ? "Confirm Blade ON" : "Blade On"}
            </Button>
            <Button danger icon={<WarningOutlined />} onClick={handleBladeOff}>
              Blade Off
            </Button>
          </Space>
          {bladeConfirm && (
            <Text style={{ color: colors.danger, marginTop: 8, display: "block" }}>
              Tap again to confirm blade activation
            </Text>
          )}
        </Card>
      </Space>
    </div>
  );
}
