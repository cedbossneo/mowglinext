import React from "react";
import { Card, Switch, Typography } from "antd";
import { RadarChartOutlined } from "@ant-design/icons";
import { RobotComponentEditor } from "../RobotComponentEditor.tsx";

const { Text, Paragraph } = Typography;

type Props = {
    values: Record<string, any>;
    onChange: (key: string, value: any) => void;
};

export const SensorsSection: React.FC<Props> = ({ values, onChange }) => {
    return (
        <div>
            {/* LiDAR toggle */}
            <Card size="small" style={{ marginBottom: 16 }}>
                <div style={{ display: "flex", justifyContent: "space-between", alignItems: "center" }}>
                    <div>
                        <Text strong style={{ fontSize: 14 }}>
                            <RadarChartOutlined style={{ marginRight: 6 }} />
                            LiDAR Sensor
                        </Text>
                        <Paragraph type="secondary" style={{ margin: "4px 0 0" }}>
                            Enable if your robot has a LiDAR. Activates fusion_graph scan-matching (when use_fusion_graph is on) and collision detection.
                        </Paragraph>
                    </div>
                    <Switch
                        checked={values.use_lidar ?? false}
                        onChange={(v) => onChange("use_lidar", v)}
                    />
                </div>
            </Card>

            {/* Sensor placement visual editor */}
            <RobotComponentEditor values={values} onChange={onChange} />
        </div>
    );
};
