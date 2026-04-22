import React from "react";
import { Card, Col, Form, InputNumber, Row, Space, Typography } from "antd";
import { CloudOutlined } from "@ant-design/icons";
import { useThemeMode } from "../../theme/ThemeContext.tsx";

const { Text, Paragraph } = Typography;

type Props = {
    values: Record<string, any>;
    onChange: (key: string, value: any) => void;
};

const RAIN_MODES = [
    { value: 0, label: "Ignore", description: "Keep mowing regardless of rain", color: "#8c8c8c" },
    { value: 1, label: "Dock", description: "Return to dock when rain detected", color: "#1890ff" },
    { value: 2, label: "Dock Until Dry", description: "Dock and wait for rain to stop + delay", color: "#13c2c2" },
    { value: 3, label: "Pause Auto", description: "Pause automatic mode entirely", color: "#722ed1" },
];

export const RainSection: React.FC<Props> = ({ values, onChange }) => {
    const { colors } = useThemeMode();
    const currentMode = values.rain_mode ?? 2;

    return (
        <div>
            <Card size="small" style={{ marginBottom: 16 }}>
                <Space direction="vertical" size={12} style={{ width: "100%" }}>
                    <div>
                        <Text strong style={{ fontSize: 14 }}>
                            <CloudOutlined style={{ marginRight: 6 }} />
                            Rain Behavior
                        </Text>
                        <Paragraph type="secondary" style={{ margin: "4px 0 0" }}>
                            What the robot should do when it detects rain.
                        </Paragraph>
                    </div>

                    <Row gutter={[8, 8]}>
                        {RAIN_MODES.map((mode) => {
                            const isSelected = currentMode === mode.value;
                            return (
                                <Col xs={12} sm={6} key={mode.value}>
                                    <Card
                                        hoverable
                                        size="small"
                                        onClick={() => onChange("rain_mode", mode.value)}
                                        style={{
                                            border: isSelected
                                                ? `2px solid ${mode.color}`
                                                : `1px solid ${colors.border}`,
                                            background: isSelected ? `${mode.color}10` : undefined,
                                            height: "100%",
                                            cursor: "pointer",
                                        }}
                                        styles={{ body: { padding: "8px 10px" } }}
                                    >
                                        <Text strong style={{ fontSize: 12, color: isSelected ? mode.color : undefined }}>
                                            {mode.label}
                                        </Text>
                                        <br />
                                        <Text type="secondary" style={{ fontSize: 11 }}>
                                            {mode.description}
                                        </Text>
                                    </Card>
                                </Col>
                            );
                        })}
                    </Row>
                </Space>
            </Card>

            {currentMode > 0 && (
                <Card size="small" title="Timing" style={{ marginBottom: 16 }}>
                    <Form layout="vertical" size="small">
                        <Row gutter={[16, 0]}>
                            <Col xs={12}>
                                <Form.Item label="Resume Delay" tooltip="Minutes to wait after rain stops before resuming">
                                    <InputNumber
                                        value={values.rain_delay_minutes}
                                        onChange={(v) => onChange("rain_delay_minutes", v)}
                                        min={0} max={240} step={5} precision={0}
                                        style={{ width: "100%" }} addonAfter="min"
                                    />
                                </Form.Item>
                            </Col>
                            <Col xs={12}>
                                <Form.Item label="Debounce" tooltip="Seconds of continuous rain signal before triggering">
                                    <InputNumber
                                        value={values.rain_debounce_sec}
                                        onChange={(v) => onChange("rain_debounce_sec", v)}
                                        min={1} max={60} step={5} precision={0}
                                        style={{ width: "100%" }} addonAfter="s"
                                    />
                                </Form.Item>
                            </Col>
                        </Row>
                    </Form>
                </Card>
            )}
        </div>
    );
};
