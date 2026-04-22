import React from "react";
import { Alert, Card, Col, Form, InputNumber, Row, Space, Switch, Typography } from "antd";
import { SafetyOutlined, WarningOutlined } from "@ant-design/icons";
import { useThemeMode } from "../../theme/ThemeContext.tsx";

const { Text, Paragraph } = Typography;

type Props = {
    values: Record<string, any>;
    onChange: (key: string, value: any) => void;
};

export const SafetySection: React.FC<Props> = ({ values, onChange }) => {
    const { colors } = useThemeMode();

    return (
        <div>
            <Alert
                type="warning"
                showIcon
                icon={<WarningOutlined />}
                message="Safety settings protect the robot and its surroundings"
                description="Firmware is the sole safety authority. These settings configure firmware behavior — disabling them reduces protection."
                style={{ marginBottom: 16 }}
            />

            {/* Emergency stops */}
            <Card size="small" style={{ marginBottom: 16 }}>
                <Space direction="vertical" size={16} style={{ width: "100%" }}>
                    <div>
                        <Text strong style={{ fontSize: 14 }}>
                            <SafetyOutlined style={{ marginRight: 6 }} />
                            Emergency Stop Triggers
                        </Text>
                    </div>

                    <div style={{
                        display: "flex", justifyContent: "space-between", alignItems: "center",
                        padding: "8px 12px", borderRadius: 8,
                        background: values.emergency_stop_on_tilt ? colors.primaryBg : colors.bgSubtle,
                        border: `1px solid ${values.emergency_stop_on_tilt ? colors.primary : colors.borderSubtle}`,
                    }}>
                        <div>
                            <Text strong>Tilt Detection</Text>
                            <br />
                            <Text type="secondary" style={{ fontSize: 12 }}>
                                Emergency stop when robot tilts beyond safe angle
                            </Text>
                        </div>
                        <Switch
                            checked={values.emergency_stop_on_tilt ?? true}
                            onChange={(v) => onChange("emergency_stop_on_tilt", v)}
                        />
                    </div>

                    <div style={{
                        display: "flex", justifyContent: "space-between", alignItems: "center",
                        padding: "8px 12px", borderRadius: 8,
                        background: values.emergency_stop_on_lift ? colors.primaryBg : colors.bgSubtle,
                        border: `1px solid ${values.emergency_stop_on_lift ? colors.primary : colors.borderSubtle}`,
                    }}>
                        <div>
                            <Text strong>Lift Detection</Text>
                            <br />
                            <Text type="secondary" style={{ fontSize: 12 }}>
                                Emergency stop when robot is lifted off ground
                            </Text>
                        </div>
                        <Switch
                            checked={values.emergency_stop_on_lift ?? true}
                            onChange={(v) => onChange("emergency_stop_on_lift", v)}
                        />
                    </div>
                </Space>
            </Card>

            {/* Temperature */}
            <Card size="small" title="Motor Temperature Limits" style={{ marginBottom: 16 }}>
                <Paragraph type="secondary" style={{ fontSize: 12, marginBottom: 12 }}>
                    Blade motor stops above the high threshold and resumes below the low threshold (hysteresis).
                </Paragraph>
                <Form layout="vertical" size="small">
                    <Row gutter={[16, 0]}>
                        <Col xs={12}>
                            <Form.Item
                                label={<Text style={{ color: "#f5222d", fontSize: 12 }}>Stop Above</Text>}
                                tooltip="Stop blade motor above this temperature"
                            >
                                <InputNumber
                                    value={values.motor_temp_high_c}
                                    onChange={(v) => onChange("motor_temp_high_c", v)}
                                    min={40} max={120} step={5} precision={0}
                                    style={{ width: "100%" }} addonAfter="C"
                                />
                            </Form.Item>
                        </Col>
                        <Col xs={12}>
                            <Form.Item
                                label={<Text style={{ color: "#52c41a", fontSize: 12 }}>Resume Below</Text>}
                                tooltip="Resume blade motor below this temperature"
                            >
                                <InputNumber
                                    value={values.motor_temp_low_c}
                                    onChange={(v) => onChange("motor_temp_low_c", v)}
                                    min={20} max={80} step={5} precision={0}
                                    style={{ width: "100%" }} addonAfter="C"
                                />
                            </Form.Item>
                        </Col>
                    </Row>
                </Form>
            </Card>

            {/* Obstacle avoidance */}
            <Card size="small" title="Obstacle Avoidance" style={{ marginBottom: 16 }}>
                <Form layout="vertical" size="small">
                    <Row gutter={[16, 0]}>
                        <Col xs={12} sm={8}>
                            <Form.Item label="Max Detour Distance" tooltip="Maximum distance to detour around an obstacle before giving up">
                                <InputNumber
                                    value={values.max_obstacle_avoidance_distance}
                                    onChange={(v) => onChange("max_obstacle_avoidance_distance", v)}
                                    min={0.5} max={10} step={0.5} precision={1}
                                    style={{ width: "100%" }} addonAfter="m"
                                />
                            </Form.Item>
                        </Col>
                    </Row>
                </Form>
            </Card>
        </div>
    );
};
