import React from "react";
import { Card, Col, Form, InputNumber, Row, Space, Switch, Typography } from "antd";
import { HomeOutlined } from "@ant-design/icons";

const { Text, Paragraph } = Typography;

type Props = {
    values: Record<string, any>;
    onChange: (key: string, value: any) => void;
};

export const DockingSection: React.FC<Props> = ({ values, onChange }) => {
    return (
        <div>
            <Card size="small" style={{ marginBottom: 16 }}>
                <Space direction="vertical" size={12} style={{ width: "100%" }}>
                    <div>
                        <Text strong style={{ fontSize: 14 }}>
                            <HomeOutlined style={{ marginRight: 6 }} />
                            Docking Behavior
                        </Text>
                        <Paragraph type="secondary" style={{ margin: "4px 0 0" }}>
                            Configure how the robot approaches and leaves the dock.
                            Dock position (X, Y, Yaw) is set from the Map editor.
                        </Paragraph>
                    </div>
                    <Form layout="vertical" size="small">
                        <Row gutter={[16, 0]}>
                            <Col xs={12} sm={8}>
                                <Form.Item label="Undock Distance" tooltip="How far to reverse when leaving the dock">
                                    <InputNumber
                                        value={values.undock_distance}
                                        onChange={(v) => onChange("undock_distance", v)}
                                        min={0.5} max={3.0} step={0.1} precision={2}
                                        style={{ width: "100%" }} addonAfter="m"
                                    />
                                </Form.Item>
                            </Col>
                            <Col xs={12} sm={8}>
                                <Form.Item label="Undock Speed" tooltip="Reverse speed when undocking">
                                    <InputNumber
                                        value={values.undock_speed}
                                        onChange={(v) => onChange("undock_speed", v)}
                                        min={0.05} max={0.3} step={0.05} precision={2}
                                        style={{ width: "100%" }} addonAfter="m/s"
                                    />
                                </Form.Item>
                            </Col>
                            <Col xs={12} sm={8}>
                                <Form.Item label="Approach Distance" tooltip="Staging point distance in front of dock">
                                    <InputNumber
                                        value={values.dock_approach_distance}
                                        onChange={(v) => onChange("dock_approach_distance", v)}
                                        min={0.5} max={3.0} step={0.1} precision={2}
                                        style={{ width: "100%" }} addonAfter="m"
                                    />
                                </Form.Item>
                            </Col>
                            <Col xs={12} sm={8}>
                                <Form.Item label="Max Retries" tooltip="How many docking attempts before giving up">
                                    <InputNumber
                                        value={values.dock_max_retries}
                                        onChange={(v) => onChange("dock_max_retries", v)}
                                        min={1} max={10} step={1} precision={0}
                                        style={{ width: "100%" }}
                                    />
                                </Form.Item>
                            </Col>
                            <Col xs={12} sm={8}>
                                <Form.Item label="Charger Detection" tooltip="Use charging voltage to confirm successful dock">
                                    <Switch
                                        checked={values.dock_use_charger_detection ?? true}
                                        onChange={(v) => onChange("dock_use_charger_detection", v)}
                                    />
                                </Form.Item>
                            </Col>
                        </Row>
                    </Form>
                </Space>
            </Card>
        </div>
    );
};
