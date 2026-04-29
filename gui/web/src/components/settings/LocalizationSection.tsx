import React from "react";
import { Alert, Card, Col, Row, Space, Switch, Tag, Typography } from "antd";
import { NodeIndexOutlined } from "@ant-design/icons";
import { useThemeMode } from "../../theme/ThemeContext.tsx";

const { Text, Paragraph } = Typography;

type Props = {
    values: Record<string, any>;
    onChange: (key: string, value: any) => void;
};

const asBool = (v: any): boolean => v === true || v === "true";

type Toggle = {
    key: string;
    title: string;
    summary: string;
    detail: string;
    impliesGraph?: boolean; // true when the option only kicks in with use_fusion_graph
};

const TOGGLES: Toggle[] = [
    {
        key: "use_fusion_graph",
        title: "Fusion Graph (iSAM2)",
        summary: "Use the GTSAM factor-graph localizer for the map frame.",
        detail:
            "Replaces ekf_map_node with fusion_graph_node. Same inputs " +
            "(wheel + IMU + GPS + COG/mag yaw) plus optional LiDAR scan-matching " +
            "and loop-closure factors. Required to carry the map-frame estimate " +
            "through multi-minute RTK-Float windows.",
    },
    {
        key: "use_scan_matching",
        title: "LiDAR scan matching",
        summary: "Add ICP between-factors from /scan in the graph.",
        detail:
            "Each new graph node runs ICP against the previous scan and adds a " +
            "BetweenFactor with the relative motion. Fights GPS dropouts but " +
            "costs ~5 ms/tick at 10 Hz.",
        impliesGraph: true,
    },
    {
        key: "use_loop_closure",
        title: "Loop closure",
        summary: "Search past scans for revisits and add LC factors.",
        detail:
            "Triggers a candidate search around each new node (5 m radius, " +
            "10 min minimum age). Successful loop closures pull drift back to " +
            "the originally-mapped pose, even mid-session.",
        impliesGraph: true,
    },
    {
        key: "use_magnetometer",
        title: "Magnetometer yaw",
        summary: "Fuse tilt-compensated mag yaw as a yaw unary factor.",
        detail:
            "Off by default — motor-induced bias makes the magnetometer " +
            "unreliable on most chassis. Enable only after running mag " +
            "calibration with motors-off and validating a stable |B|.",
    },
];

export const LocalizationSection: React.FC<Props> = ({ values, onChange }) => {
    const { colors } = useThemeMode();
    const fusionOn = asBool(values.use_fusion_graph);

    return (
        <div>
            <Alert
                type="info"
                showIcon
                style={{ marginBottom: 16 }}
                message="Map-frame localizer choice"
                description={
                    <span>
                        With <Text code>use_fusion_graph</Text> off, the legacy
                        robot_localization dual-EKF runs as the map-frame fuser.
                        With it on, the GTSAM iSAM2 factor-graph node takes over —
                        identical inputs/outputs but adds optional LiDAR factors
                        below. Restart ROS2 after changing.
                    </span>
                }
            />

            {TOGGLES.map((t) => {
                const enabled = asBool(values[t.key]);
                const inactive = t.impliesGraph && !fusionOn;
                return (
                    <Card
                        key={t.key}
                        size="small"
                        style={{ marginBottom: 12, opacity: inactive ? 0.6 : 1 }}
                    >
                        <Row align="middle" gutter={[16, 8]} wrap={false}>
                            <Col flex="auto">
                                <Space>
                                    <Text strong style={{ fontSize: 14 }}>
                                        <NodeIndexOutlined style={{ marginRight: 6, color: colors.accent }} />
                                        {t.title}
                                    </Text>
                                    {inactive && (
                                        <Tag color="default">requires use_fusion_graph</Tag>
                                    )}
                                </Space>
                                <Paragraph style={{ margin: "4px 0 0", fontSize: 12 }}>
                                    {t.summary}
                                </Paragraph>
                                <Paragraph type="secondary" style={{ margin: "4px 0 0", fontSize: 11 }}>
                                    {t.detail}
                                </Paragraph>
                            </Col>
                            <Col flex="none">
                                <Switch
                                    checked={enabled}
                                    onChange={(v) => onChange(t.key, v)}
                                    disabled={inactive}
                                />
                            </Col>
                        </Row>
                    </Card>
                );
            })}
        </div>
    );
};
