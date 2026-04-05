import { useState } from "react";
import { Button, Input, Card, Space, Typography, message, Tabs } from "antd";
import { WifiOutlined, CloudOutlined } from "@ant-design/icons";
import { useConnection, useThemeMode } from "@mowglinext/common";
import { saveConnection, getSavedConnections } from "../native/preferences";
import { useEffect } from "react";

const { Title, Text } = Typography;

interface SavedConn {
  name: string;
  host: string;
  isCloud: boolean;
}

export function ConnectionSetup() {
  const { setConnection } = useConnection();
  const { colors } = useThemeMode();
  const [host, setHost] = useState("");
  const [email, setEmail] = useState("");
  const [password, setPassword] = useState("");
  const [loading, setLoading] = useState(false);
  const [savedConnections, setSavedConnections] = useState<SavedConn[]>([]);

  useEffect(() => {
    getSavedConnections().then(setSavedConnections);
  }, []);

  const connectLocal = async () => {
    if (!host) return;
    setLoading(true);
    try {
      const httpUrl = host.startsWith("http") ? host : `http://${host}`;
      const wsProtocol = httpUrl.startsWith("https") ? "wss" : "ws";
      const hostPart = httpUrl.replace(/^https?:\/\//, "");
      const wsUrl = `${wsProtocol}://${hostPart}`;

      // Test connection
      const res = await fetch(`${httpUrl}/api/system/info`);
      if (!res.ok) throw new Error("Cannot reach robot");

      const conn = {
        baseHttpUrl: httpUrl,
        baseWsUrl: wsUrl,
        authToken: null,
        isConnected: true,
        robotId: null,
      };
      setConnection(conn);
      await saveConnection({ name: hostPart, host: hostPart, isCloud: false }, conn);
    } catch (e: any) {
      message.error(e.message || "Connection failed");
    } finally {
      setLoading(false);
    }
  };

  const connectCloud = async () => {
    if (!email || !password) return;
    setLoading(true);
    try {
      const proxyUrl = "https://proxy.mowgli.garden";
      const res = await fetch(`${proxyUrl}/api/auth/login`, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ email, password }),
      });
      if (!res.ok) throw new Error("Login failed");
      const data = await res.json();

      const conn = {
        baseHttpUrl: proxyUrl,
        baseWsUrl: "wss://proxy.mowgli.garden",
        authToken: data.access_token,
        isConnected: true,
        robotId: data.robots?.[0]?.id || null,
      };
      setConnection(conn);
      await saveConnection(
        { name: "Cloud", host: "proxy.mowgli.garden", isCloud: true },
        conn
      );
    } catch (e: any) {
      message.error(e.message || "Login failed");
    } finally {
      setLoading(false);
    }
  };

  const connectSaved = (saved: SavedConn) => {
    if (saved.isCloud) {
      // Show cloud login for saved cloud connections
      message.info("Please log in to your cloud account");
    } else {
      setHost(saved.host);
      connectLocal();
    }
  };

  return (
    <div
      style={{
        height: "100%",
        display: "flex",
        flexDirection: "column",
        justifyContent: "center",
        alignItems: "center",
        padding: 24,
        background: colors.bgBase,
      }}
    >
      <Title level={3} style={{ color: colors.text, marginBottom: 8 }}>
        MowgliNext
      </Title>
      <Text style={{ color: colors.textSecondary, marginBottom: 24 }}>
        Connect to your robot mower
      </Text>

      {savedConnections.length > 0 && (
        <Card
          size="small"
          title="Saved Connections"
          style={{ width: "100%", maxWidth: 400, marginBottom: 16 }}
        >
          <Space direction="vertical" style={{ width: "100%" }}>
            {savedConnections.map((c, i) => (
              <Button
                key={i}
                block
                icon={c.isCloud ? <CloudOutlined /> : <WifiOutlined />}
                onClick={() => connectSaved(c)}
              >
                {c.name}
              </Button>
            ))}
          </Space>
        </Card>
      )}

      <Card style={{ width: "100%", maxWidth: 400 }}>
        <Tabs
          items={[
            {
              key: "local",
              label: (
                <span>
                  <WifiOutlined /> Local
                </span>
              ),
              children: (
                <Space direction="vertical" style={{ width: "100%" }} size="middle">
                  <Input
                    placeholder="Robot IP (e.g. 192.168.1.50)"
                    value={host}
                    onChange={(e) => setHost(e.target.value)}
                    onPressEnter={connectLocal}
                    size="large"
                  />
                  <Button
                    type="primary"
                    block
                    size="large"
                    loading={loading}
                    onClick={connectLocal}
                  >
                    Connect
                  </Button>
                </Space>
              ),
            },
            {
              key: "cloud",
              label: (
                <span>
                  <CloudOutlined /> Cloud
                </span>
              ),
              children: (
                <Space direction="vertical" style={{ width: "100%" }} size="middle">
                  <Input
                    placeholder="Email"
                    value={email}
                    onChange={(e) => setEmail(e.target.value)}
                    size="large"
                  />
                  <Input.Password
                    placeholder="Password"
                    value={password}
                    onChange={(e) => setPassword(e.target.value)}
                    onPressEnter={connectCloud}
                    size="large"
                  />
                  <Button
                    type="primary"
                    block
                    size="large"
                    loading={loading}
                    onClick={connectCloud}
                  >
                    Log In
                  </Button>
                </Space>
              ),
            },
          ]}
        />
      </Card>
    </div>
  );
}
