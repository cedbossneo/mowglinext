import { Card, Space, Typography, Row, Col, Progress } from "antd";
import {
  useHighLevelStatus,
  usePower,
  useGPS,
  useEmergency,
  MowerActions,
  useThemeMode,
  type AbsolutePoseFlags,
} from "@mowglinext/common";

const { Text, Title } = Typography;

export function DashboardScreen() {
  const { colors } = useThemeMode();
  const { highLevelStatus } = useHighLevelStatus();
  const power = usePower();
  const gps = useGPS();
  const emergency = useEmergency();

  const batteryPercent = (() => {
    if (highLevelStatus.BatteryPercent != null && highLevelStatus.BatteryPercent > 0) {
      return Math.round(highLevelStatus.BatteryPercent * 100);
    }
    if (power.VBattery) {
      const pct = ((power.VBattery - 23.0) / (28.5 - 23.0)) * 100;
      return Math.round(Math.max(0, Math.min(100, pct)));
    }
    return 0;
  })();

  const gpsPercent = (() => {
    if (highLevelStatus.GpsQualityPercent != null && highLevelStatus.GpsQualityPercent > 0) {
      return Math.round(highLevelStatus.GpsQualityPercent * 100);
    }
    return 0;
  })();

  const stateName = highLevelStatus.StateName ?? "UNKNOWN";
  const isEmergency = highLevelStatus.Emergency ?? emergency.ActiveEmergency ?? false;
  const isCharging = highLevelStatus.IsCharging ?? false;

  const stateColor = (() => {
    if (isEmergency) return colors.danger;
    switch (stateName) {
      case "MOWING":
      case "DOCKING":
      case "UNDOCKING":
        return colors.primary;
      case "IDLE":
      case "CHARGING":
        return colors.warning;
      default:
        return colors.danger;
    }
  })();

  return (
    <div style={{ padding: 16, paddingBottom: 80, overflowY: "auto", height: "100%" }}>
      <Space direction="vertical" size="middle" style={{ width: "100%" }}>
        {/* Status Card */}
        <Card size="small">
          <Row align="middle" gutter={16}>
            <Col>
              <div
                style={{
                  width: 12,
                  height: 12,
                  borderRadius: "50%",
                  background: stateColor,
                  boxShadow: `0 0 8px ${stateColor}`,
                }}
              />
            </Col>
            <Col flex="auto">
              <Title level={4} style={{ margin: 0, color: colors.text }}>
                {stateName}
              </Title>
              {highLevelStatus.SubStateName && (
                <Text style={{ color: colors.textSecondary, fontSize: 12 }}>
                  {highLevelStatus.SubStateName}
                </Text>
              )}
            </Col>
          </Row>
        </Card>

        {/* Battery & GPS */}
        <Row gutter={12}>
          <Col span={12}>
            <Card size="small" title="Battery">
              <Progress
                percent={batteryPercent}
                status={batteryPercent < 20 ? "exception" : "normal"}
                strokeColor={isCharging ? colors.primary : undefined}
              />
              {power.VBattery && (
                <Text style={{ color: colors.textSecondary, fontSize: 12 }}>
                  {power.VBattery.toFixed(1)}V
                  {power.ChargeCurrent ? ` / ${power.ChargeCurrent.toFixed(1)}A` : ""}
                </Text>
              )}
            </Card>
          </Col>
          <Col span={12}>
            <Card size="small" title="GPS">
              <Progress
                percent={gpsPercent}
                status={gpsPercent < 50 ? "exception" : "normal"}
                strokeColor={gpsPercent >= 80 ? colors.primary : undefined}
              />
              <Text style={{ color: colors.textSecondary, fontSize: 12 }}>
                {gpsPercent >= 100 ? "RTK Fixed" : gpsPercent >= 50 ? "RTK Float" : "Low quality"}
              </Text>
            </Card>
          </Col>
        </Row>

        {/* Mowing Progress */}
        {stateName === "MOWING" && highLevelStatus.CurrentPath != null && highLevelStatus.CurrentPath > 0 && (
          <Card size="small" title={`Mowing Area ${(highLevelStatus.CurrentArea ?? 0) + 1}`}>
            <Progress
              percent={Math.round(((highLevelStatus.CurrentPathIndex ?? 0) / highLevelStatus.CurrentPath) * 100)}
              strokeColor={colors.primary}
            />
          </Card>
        )}

        {/* Emergency Warning */}
        {isEmergency && (
          <Card
            size="small"
            style={{ background: colors.dangerBg, borderColor: colors.danger }}
          >
            <Text style={{ color: colors.danger, fontWeight: 600 }}>
              Emergency Active: {emergency.Reason || "Unknown reason"}
            </Text>
          </Card>
        )}

        {/* Actions */}
        <MowerActions />
      </Space>
    </div>
  );
}
