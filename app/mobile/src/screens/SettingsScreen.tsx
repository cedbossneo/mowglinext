import { Card, Space, Typography, Button, Switch, Divider, List } from "antd";
import {
  WifiOutlined,
  LogoutOutlined,
  BgColorsOutlined,
  BellOutlined,
  InfoCircleOutlined,
} from "@ant-design/icons";
import { useConnection, useThemeMode } from "@mowglinext/common";
import { clearSavedConnection } from "../native/preferences";

const { Text, Title } = Typography;

export function SettingsScreen() {
  const { colors, mode, toggleMode } = useThemeMode();
  const { baseHttpUrl, robotId, setConnection } = useConnection();

  const disconnect = async () => {
    await clearSavedConnection();
    setConnection({
      baseHttpUrl: "",
      baseWsUrl: "",
      authToken: null,
      isConnected: false,
      robotId: null,
    });
  };

  return (
    <div style={{ padding: 16, paddingBottom: 80, height: "100%", overflowY: "auto" }}>
      <Space direction="vertical" size="middle" style={{ width: "100%" }}>
        <Title level={4} style={{ margin: 0, color: colors.text }}>
          Settings
        </Title>

        {/* Connection Info */}
        <Card size="small" title="Connection">
          <List>
            <List.Item>
              <List.Item.Meta
                avatar={<WifiOutlined style={{ color: colors.primary }} />}
                title="Connected to"
                description={baseHttpUrl || "Local"}
              />
            </List.Item>
            {robotId && (
              <List.Item>
                <List.Item.Meta title="Robot ID" description={robotId} />
              </List.Item>
            )}
          </List>
          <Button
            danger
            icon={<LogoutOutlined />}
            onClick={disconnect}
            block
            style={{ marginTop: 12 }}
          >
            Disconnect
          </Button>
        </Card>

        {/* Appearance */}
        <Card size="small" title="Appearance">
          <div style={{ display: "flex", justifyContent: "space-between", alignItems: "center" }}>
            <Space>
              <BgColorsOutlined style={{ color: colors.textSecondary }} />
              <Text style={{ color: colors.text }}>Dark Mode</Text>
            </Space>
            <Switch checked={mode === "dark"} onChange={toggleMode} />
          </div>
        </Card>

        {/* Notifications */}
        <Card size="small" title="Notifications">
          <div style={{ display: "flex", justifyContent: "space-between", alignItems: "center" }}>
            <Space>
              <BellOutlined style={{ color: colors.textSecondary }} />
              <Text style={{ color: colors.text }}>Push Notifications</Text>
            </Space>
            <Switch defaultChecked />
          </div>
        </Card>

        {/* About */}
        <Card size="small" title="About">
          <List>
            <List.Item>
              <List.Item.Meta
                avatar={<InfoCircleOutlined style={{ color: colors.textSecondary }} />}
                title="MowgliNext Mobile"
                description="v0.0.1"
              />
            </List.Item>
          </List>
        </Card>
      </Space>
    </div>
  );
}
