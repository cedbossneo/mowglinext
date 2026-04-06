import {Card, Col, Row, Statistic, Space} from "antd";
import {DashboardOutlined} from "@ant-design/icons";
import {useEffect, useState} from "react";
import {useApi} from "@mowglinext/common";

interface SystemInfo {
    cpuTemperature?: number;
}

export function SystemInfoComponent() {
    const guiApi = useApi();
    const [info, setInfo] = useState<SystemInfo>({});

    useEffect(() => {
        const fetchInfo = async () => {
            try {
                const response = await guiApi.request<SystemInfo>({
                    path: "/system/info",
                    method: "GET",
                    format: "json",
                });
                setInfo(response.data);
            } catch {
                // System info may not be available on all platforms
            }
        };
        fetchInfo();
        const interval = setInterval(fetchInfo, 10000);
        return () => clearInterval(interval);
    }, []);

    if (info.cpuTemperature == null) {
        return null;
    }

    const tempColor = (info.cpuTemperature ?? 0) > 70 ? "#ff4d4f" : (info.cpuTemperature ?? 0) > 55 ? "#faad14" : undefined;

    return (
        <Card title={<Space><DashboardOutlined/> System</Space>} size="small">
            <Row gutter={[16, 16]}>
                <Col span={8}>
                    <Statistic
                        title="CPU Temp"
                        value={info.cpuTemperature}
                        precision={1}
                        suffix="°C"
                        valueStyle={tempColor ? {color: tempColor} : undefined}
                    />
                </Col>
            </Row>
        </Card>
    );
}
