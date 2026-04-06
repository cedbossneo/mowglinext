import { useEffect, useState } from "react";
import { Card, Switch, Space, Typography, Button, Empty } from "antd";
import { PlusOutlined, ClockCircleOutlined } from "@ant-design/icons";
import { useApi, useThemeMode } from "@mowglinext/common";

const { Text, Title } = Typography;

interface Schedule {
  id: string;
  name: string;
  enabled: boolean;
  cron: string;
  command: string;
}

export function ScheduleScreen() {
  const { colors } = useThemeMode();
  const api = useApi();
  const [schedules, setSchedules] = useState<Schedule[]>([]);
  const [loading, setLoading] = useState(true);

  useEffect(() => {
    loadSchedules();
  }, []);

  const loadSchedules = async () => {
    try {
      // The schedule API endpoint
      const res = await api.request<Schedule[]>({
        path: "/schedules",
        method: "GET",
        format: "json",
      });
      setSchedules(res.data || []);
    } catch {
      // Schedules may not be configured yet
      setSchedules([]);
    } finally {
      setLoading(false);
    }
  };

  const toggleSchedule = async (id: string, enabled: boolean) => {
    try {
      await api.request({
        path: `/schedules/${id}`,
        method: "PUT",
        body: { enabled },
        format: "json",
      });
      setSchedules((prev) =>
        prev.map((s) => (s.id === id ? { ...s, enabled } : s))
      );
    } catch {
      // ignore
    }
  };

  return (
    <div style={{ padding: 16, paddingBottom: 80, height: "100%", overflowY: "auto" }}>
      <Space direction="vertical" size="middle" style={{ width: "100%" }}>
        <div style={{ display: "flex", justifyContent: "space-between", alignItems: "center" }}>
          <Title level={4} style={{ margin: 0, color: colors.text }}>
            Schedules
          </Title>
          <Button type="primary" icon={<PlusOutlined />} size="small">
            Add
          </Button>
        </div>

        {schedules.length === 0 && !loading && (
          <Empty description="No schedules configured" />
        )}

        {schedules.map((schedule) => (
          <Card key={schedule.id} size="small">
            <div style={{ display: "flex", justifyContent: "space-between", alignItems: "center" }}>
              <Space direction="vertical" size={2}>
                <Text strong style={{ color: colors.text }}>
                  {schedule.name}
                </Text>
                <Space size={4}>
                  <ClockCircleOutlined style={{ color: colors.textSecondary, fontSize: 12 }} />
                  <Text style={{ color: colors.textSecondary, fontSize: 12 }}>
                    {schedule.cron}
                  </Text>
                </Space>
              </Space>
              <Switch
                checked={schedule.enabled}
                onChange={(checked) => toggleSchedule(schedule.id, checked)}
              />
            </div>
          </Card>
        ))}
      </Space>
    </div>
  );
}
