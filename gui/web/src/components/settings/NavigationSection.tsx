import React from "react";
import { Card, Col, Form, InputNumber, Row, Space, Typography } from "antd";
import { CompassOutlined } from "@ant-design/icons";

const { Text, Paragraph } = Typography;

type Props = {
    values: Record<string, any>;
    onChange: (key: string, value: any) => void;
};

export const NavigationSection: React.FC<Props> = ({ values, onChange }) => {
    return (
        <div>
            <Card size="small" style={{ marginBottom: 16 }}>
                <Space direction="vertical" size={12} style={{ width: "100%" }}>
                    <div>
                        <Text strong style={{ fontSize: 14 }}>
                            <CompassOutlined style={{ marginRight: 6 }} />
                            Goal Tolerances
                        </Text>
                        <Paragraph type="secondary" style={{ margin: "4px 0 0" }}>
                            How close the robot needs to get to a target position before considering it reached.
                            Tighter tolerances mean more precision but may cause oscillation.
                        </Paragraph>
                    </div>
                    <Form layout="vertical" size="small">
                        <Row gutter={[16, 0]}>
                            <Col xs={12} sm={8}>
                                <Form.Item label="Transit XY Tolerance" tooltip="Position tolerance for transit waypoints">
                                    <InputNumber
                                        value={values.xy_goal_tolerance}
                                        onChange={(v) => onChange("xy_goal_tolerance", v)}
                                        min={0.1} max={2.0} step={0.1} precision={2}
                                        style={{ width: "100%" }} addonAfter="m"
                                    />
                                </Form.Item>
                            </Col>
                            <Col xs={12} sm={8}>
                                <Form.Item label="Yaw Tolerance" tooltip="Heading tolerance at goal (radians)">
                                    <InputNumber
                                        value={values.yaw_goal_tolerance}
                                        onChange={(v) => onChange("yaw_goal_tolerance", v)}
                                        min={0.1} max={3.14} step={0.1} precision={2}
                                        style={{ width: "100%" }} addonAfter="rad"
                                    />
                                </Form.Item>
                            </Col>
                            <Col xs={12} sm={8}>
                                <Form.Item label="Coverage XY Tolerance" tooltip="Position tolerance during mowing strips">
                                    <InputNumber
                                        value={values.coverage_xy_tolerance}
                                        onChange={(v) => onChange("coverage_xy_tolerance", v)}
                                        min={0.05} max={1.0} step={0.05} precision={2}
                                        style={{ width: "100%" }} addonAfter="m"
                                    />
                                </Form.Item>
                            </Col>
                        </Row>
                    </Form>
                </Space>
            </Card>

            <Card size="small" title="Recovery" style={{ marginBottom: 16 }}>
                <Form layout="vertical" size="small">
                    <Row gutter={[16, 0]}>
                        <Col xs={12} sm={8}>
                            <Form.Item label="Progress Timeout" tooltip="Seconds without forward progress before recovery kicks in">
                                <InputNumber
                                    value={values.progress_timeout_sec}
                                    onChange={(v) => onChange("progress_timeout_sec", v)}
                                    min={10} max={300} step={10} precision={0}
                                    style={{ width: "100%" }} addonAfter="s"
                                />
                            </Form.Item>
                        </Col>
                    </Row>
                </Form>
            </Card>
        </div>
    );
};
