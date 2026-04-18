import React from "react";
import { Alert, Button, Card, Form, Select } from "antd";
import type { BackendSettingsValues } from "../hooks/useBackendSettings.ts";

type BackendSettingsCardProps = {
    values: BackendSettingsValues;
    onChange: (key: keyof BackendSettingsValues, value: BackendSettingsValues[keyof BackendSettingsValues]) => void;
    onSave?: () => Promise<void> | void;
    loading?: boolean;
};

export const BackendSettingsCard: React.FC<BackendSettingsCardProps> = ({
    values,
    onChange,
    onSave,
    loading = false,
}) => {
    return (
        <Card
            title="Hardware Backend"
            style={{ marginBottom: 16 }}
            extra={
                onSave ? (
                    <Button type="primary" onClick={() => void onSave()} loading={loading}>
                        Save Backend
                    </Button>
                ) : undefined
            }
        >
            <Form layout="vertical">
                <Form.Item
                    label="Backend"
                    tooltip="Select which hardware control stack should be used at runtime."
                >
                    <Select
                        value={values.HARDWARE_BACKEND}
                        onChange={(value) => onChange("HARDWARE_BACKEND", value)}
                        options={[
                            { label: "Mowgli STM32 board", value: "mowgli" },
                            { label: "Pixhawk via MAVROS", value: "mavros" },
                        ]}
                    />
                </Form.Item>

                {values.HARDWARE_BACKEND === "mavros" && (
                    <>
                        <Form.Item
                            label="Autopilot Stack"
                            tooltip="Select the autopilot stack used by the Pixhawk MAVROS container."
                        >
                            <Select
                                value={values.MAVROS_AUTOPILOT}
                                onChange={(value) => onChange("MAVROS_AUTOPILOT", value)}
                                options={[
                                    { label: "ArduPilot / ArduRover", value: "ardupilot" },
                                    { label: "PX4", value: "px4" },
                                ]}
                            />
                        </Form.Item>

                        <Alert
                            type="info"
                            showIcon
                            message="MAVROS backend is still provisional"
                            description="Some features remain provisional until final hardware validation, including manual control mapping, charging feedback, and blade control behavior."
                        />
                    </>
                )}
            </Form>
        </Card>
    );
};
