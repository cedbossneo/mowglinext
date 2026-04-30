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
    // LiDAR drives use_fusion_graph: factor-graph scan-matching is the only
    // reason to keep fusion_graph on, and ekf_map_node is the right default
    // when no LiDAR is mounted. Operators can still override use_fusion_graph
    // independently from the Localization tab (e.g. to use the graph without
    // scan factors), but the defaults stay sensible.
    const handleLidarToggle = (enabled: boolean) => {
        onChange("use_lidar", enabled);
        onChange("use_fusion_graph", enabled);
    };

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
                            Enable if your robot has a LiDAR. Also flips
                            {" "}<Text code>use_fusion_graph</Text> so scan-matching factors and
                            loop-closure are active. Override that default in the Localization tab
                            if you want a graph without LiDAR (rare).
                        </Paragraph>
                    </div>
                    <Switch
                        checked={values.use_lidar ?? false}
                        onChange={handleLidarToggle}
                    />
                </div>
            </Card>

            {/* Sensor placement visual editor */}
            <RobotComponentEditor values={values} onChange={onChange} />
        </div>
    );
};
