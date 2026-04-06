import React, { useCallback, useEffect, useState } from "react";
import {
    Button, Card, Col, Row, Steps, Typography, Select, Space, Alert,
    Input, InputNumber, Switch, Form, Divider, Tag, Result,
} from "antd";
import {
    RocketOutlined, SettingOutlined, GlobalOutlined,
    AimOutlined, ThunderboltOutlined, CheckCircleOutlined,
    ArrowLeftOutlined, ArrowRightOutlined, SaveOutlined,
    EnvironmentOutlined, WifiOutlined,
} from "@ant-design/icons";
import { useIsMobile } from "../hooks/useIsMobile";
import { useSettingsSchema } from "../hooks/useSettingsSchema";
import { RobotComponentEditor } from "../components/RobotComponentEditor";
import { FlashBoardComponent } from "../components/FlashBoardComponent";
import {useThemeMode, useApi} from "@mowglinext/common";

const { Title, Text, Paragraph } = Typography;

// ── Step 0: Welcome ─────────────────────────────────────────────────────

const WelcomeStep: React.FC<{ onNext: () => void }> = ({ onNext }) => {
    const { colors } = useThemeMode();
    return (
        <div style={{ textAlign: "center", maxWidth: 600, margin: "0 auto", padding: "24px 0" }}>
            <div style={{
                width: 80, height: 80, borderRadius: "50%",
                background: colors.primaryBg, display: "flex",
                alignItems: "center", justifyContent: "center",
                margin: "0 auto 24px",
            }}>
                <RocketOutlined style={{ fontSize: 36, color: colors.primary }} />
            </div>
            <Title level={2} style={{ marginBottom: 8 }}>Welcome to Mowgli</Title>
            <Paragraph type="secondary" style={{ fontSize: 16, marginBottom: 32 }}>
                Let's set up your robot mower in a few simple steps.
                You can always change these settings later.
            </Paragraph>

            <Row gutter={[16, 16]} style={{ textAlign: "left", marginBottom: 32 }}>
                <Col span={24}>
                    <Card size="small">
                        <Space>
                            <SettingOutlined style={{ color: colors.primary, fontSize: 20 }} />
                            <div>
                                <Text strong>Choose your robot</Text>
                                <br />
                                <Text type="secondary">Select your mower model and firmware</Text>
                            </div>
                        </Space>
                    </Card>
                </Col>
                <Col span={24}>
                    <Card size="small">
                        <Space>
                            <GlobalOutlined style={{ color: colors.primary, fontSize: 20 }} />
                            <div>
                                <Text strong>Set up GPS</Text>
                                <br />
                                <Text type="secondary">Configure your position and RTK corrections</Text>
                            </div>
                        </Space>
                    </Card>
                </Col>
                <Col span={24}>
                    <Card size="small">
                        <Space>
                            <AimOutlined style={{ color: colors.primary, fontSize: 20 }} />
                            <div>
                                <Text strong>Place your sensors</Text>
                                <br />
                                <Text type="secondary">Visually position LiDAR, IMU, and GPS on the robot</Text>
                            </div>
                        </Space>
                    </Card>
                </Col>
            </Row>

            <Button type="primary" size="large" onClick={onNext} icon={<ArrowRightOutlined />}>
                Get Started
            </Button>
        </div>
    );
};

// ── Step 1: Robot Model ─────────────────────────────────────────────────

type RobotModelStepProps = {
    values: Record<string, any>;
    onChange: (key: string, value: any) => void;
};

const MOWER_MODELS = [
    {
        value: "YardForce500",
        label: "YardForce Classic 500",
        description: "Most common model. 28V battery, 18cm blade, rear-wheel drive.",
        tag: "Popular",
        defaults: {
            wheel_radius: 0.04475, wheel_track: 0.325, wheel_x_offset: -0.18,
            blade_radius: 0.09, tool_width: 0.18, ticks_per_revolution: 1050,
            battery_full_voltage: 28.5, battery_empty_voltage: 24.0,
            battery_critical_voltage: 23.0,
        },
    },
    {
        value: "YardForce500B",
        label: "YardForce 500B",
        description: "500 B variant with different blade motor UART and panel layout.",
        defaults: {
            wheel_radius: 0.04475, wheel_track: 0.325, wheel_x_offset: -0.18,
            blade_radius: 0.09, tool_width: 0.18, ticks_per_revolution: 1050,
            battery_full_voltage: 28.5, battery_empty_voltage: 24.0,
            battery_critical_voltage: 23.0,
        },
    },
    {
        value: "YardForceSA650",
        label: "YardForce SA650",
        description: "Larger model with higher encoder resolution (1964 ticks/m).",
        defaults: {
            wheel_radius: 0.04475, wheel_track: 0.325, wheel_x_offset: -0.18,
            blade_radius: 0.09, tool_width: 0.18, ticks_per_revolution: 1050,
            battery_full_voltage: 28.5, battery_empty_voltage: 24.0,
            battery_critical_voltage: 23.0,
        },
    },
    {
        value: "YardForce900ECO",
        label: "YardForce 900 ECO",
        description: "Larger 900 ECO model with wider chassis.",
        defaults: {
            wheel_radius: 0.04475, wheel_track: 0.325, wheel_x_offset: -0.18,
            blade_radius: 0.09, tool_width: 0.18, ticks_per_revolution: 1050,
            battery_full_voltage: 28.5, battery_empty_voltage: 24.0,
            battery_critical_voltage: 23.0,
        },
    },
    {
        value: "LUV1000RI",
        label: "YardForce LUV1000RI",
        description: "LUV1000RI model with narrower wheelbase (0.285m) and ultrasonic sensor.",
        defaults: {
            wheel_radius: 0.04475, wheel_track: 0.285, wheel_x_offset: -0.18,
            blade_radius: 0.09, tool_width: 0.18, ticks_per_revolution: 1050,
            battery_full_voltage: 28.5, battery_empty_voltage: 24.0,
            battery_critical_voltage: 23.0,
        },
    },
    {
        value: "Sabo",
        label: "Sabo",
        description: "Sabo mower with wider wheelbase (0.45m) and higher charge current.",
        defaults: {
            wheel_radius: 0.04475, wheel_track: 0.45, wheel_x_offset: -0.18,
            blade_radius: 0.09, tool_width: 0.18, ticks_per_revolution: 1050,
            battery_full_voltage: 28.5, battery_empty_voltage: 21.0,
            battery_critical_voltage: 20.0,
        },
    },
    {
        value: "CUSTOM",
        label: "Custom Robot",
        description: "Manually configure all hardware parameters for a custom build.",
        defaults: {},
    },
];

const RobotModelStep: React.FC<RobotModelStepProps> = ({ values, onChange }) => {
    const { colors } = useThemeMode();
    const selectedModel = values.mower_model || "YardForce500";

    const handleModelSelect = (model: string) => {
        onChange("mower_model", model);
        const preset = MOWER_MODELS.find((m) => m.value === model);
        if (preset?.defaults) {
            for (const [k, v] of Object.entries(preset.defaults)) {
                onChange(k, v);
            }
        }
    };

    return (
        <div style={{ maxWidth: 800, margin: "0 auto" }}>
            <Title level={4}>
                <SettingOutlined /> Choose Your Robot
            </Title>
            <Paragraph type="secondary">
                Select your mower model. This pre-fills hardware parameters like wheel size, battery voltage, and blade dimensions.
            </Paragraph>

            <Row gutter={[12, 12]}>
                {MOWER_MODELS.map((model) => {
                    const isSelected = selectedModel === model.value;
                    return (
                        <Col xs={12} sm={8} md={6} key={model.value}>
                            <Card
                                hoverable
                                size="small"
                                onClick={() => handleModelSelect(model.value)}
                                style={{
                                    border: isSelected
                                        ? `2px solid ${colors.primary}`
                                        : `1px solid ${colors.border}`,
                                    background: isSelected ? colors.primaryBg : undefined,
                                    height: "100%",
                                    cursor: "pointer",
                                }}
                            >
                                <Space direction="vertical" size={4} style={{ width: "100%" }}>
                                    <Space>
                                        <Text strong>{model.label}</Text>
                                        {(model as any).tag && (
                                            <Tag color="green">{(model as any).tag}</Tag>
                                        )}
                                    </Space>
                                    <Text type="secondary" style={{ fontSize: 12 }}>
                                        {model.description}
                                    </Text>
                                </Space>
                            </Card>
                        </Col>
                    );
                })}
            </Row>

            {selectedModel === "CUSTOM" && (
                <>
                    <Divider />
                    <Alert
                        type="info"
                        showIcon
                        message="Custom configuration"
                        description="You can fine-tune all hardware parameters in Settings after completing onboarding."
                        style={{ marginBottom: 16 }}
                    />
                    <Form layout="vertical">
                        <Row gutter={[16, 0]}>
                            <Col xs={12} sm={8}>
                                <Form.Item label="Wheel Radius" tooltip="Drive wheel radius in metres">
                                    <InputNumber
                                        value={values.wheel_radius ?? 0.04475}
                                        onChange={(v) => onChange("wheel_radius", v)}
                                        step={0.001} precision={5} style={{ width: "100%" }}
                                        addonAfter="m"
                                    />
                                </Form.Item>
                            </Col>
                            <Col xs={12} sm={8}>
                                <Form.Item label="Wheel Track" tooltip="Centre-to-centre wheel distance">
                                    <InputNumber
                                        value={values.wheel_track ?? 0.325}
                                        onChange={(v) => onChange("wheel_track", v)}
                                        step={0.001} precision={3} style={{ width: "100%" }}
                                        addonAfter="m"
                                    />
                                </Form.Item>
                            </Col>
                            <Col xs={12} sm={8}>
                                <Form.Item label="Blade Radius" tooltip="Mowing blade radius">
                                    <InputNumber
                                        value={values.blade_radius ?? 0.09}
                                        onChange={(v) => onChange("blade_radius", v)}
                                        step={0.01} precision={3} style={{ width: "100%" }}
                                        addonAfter="m"
                                    />
                                </Form.Item>
                            </Col>
                        </Row>
                    </Form>
                </>
            )}
        </div>
    );
};

// ── Step 2: GPS Configuration ───────────────────────────────────────────

const GpsStep: React.FC<RobotModelStepProps> = ({ values, onChange }) => {
    const ntripEnabled = values.ntrip_enabled ?? true;

    return (
        <div style={{ maxWidth: 640, margin: "0 auto" }}>
            <Title level={4}>
                <GlobalOutlined /> GPS & Positioning
            </Title>
            <Paragraph type="secondary">
                Your robot uses RTK GPS for centimetre-level accuracy. Set the map origin (datum) near your docking station,
                and configure NTRIP corrections if you have an RTK base station or network.
            </Paragraph>

            <Card size="small" title={<Space><EnvironmentOutlined /> Map Origin (Datum)</Space>} style={{ marginBottom: 16 }}>
                <Paragraph type="secondary" style={{ fontSize: 12, marginBottom: 12 }}>
                    Set this to a GPS coordinate near your docking station. The robot uses this as the (0, 0) origin of its local map.
                    You can get coordinates from Google Maps by right-clicking on your dock location.
                </Paragraph>
                <Form layout="vertical">
                    <Row gutter={16}>
                        <Col xs={12}>
                            <Form.Item label="Latitude">
                                <InputNumber
                                    value={values.datum_lat ?? 0}
                                    onChange={(v) => onChange("datum_lat", v)}
                                    step={0.000001} precision={8} style={{ width: "100%" }}
                                    placeholder="48.8796"
                                />
                            </Form.Item>
                        </Col>
                        <Col xs={12}>
                            <Form.Item label="Longitude">
                                <InputNumber
                                    value={values.datum_lon ?? 0}
                                    onChange={(v) => onChange("datum_lon", v)}
                                    step={0.000001} precision={8} style={{ width: "100%" }}
                                    placeholder="2.1728"
                                />
                            </Form.Item>
                        </Col>
                    </Row>
                </Form>
            </Card>

            <Card size="small" title={<Space><WifiOutlined /> GPS Receiver</Space>} style={{ marginBottom: 16 }}>
                <Form layout="vertical">
                    <Row gutter={16}>
                        <Col xs={12}>
                            <Form.Item label="Protocol">
                                <Select
                                    value={values.gps_protocol ?? "UBX"}
                                    onChange={(v) => onChange("gps_protocol", v)}
                                    options={[
                                        { label: "UBX (u-blox)", value: "UBX" },
                                        { label: "NMEA", value: "NMEA" },
                                    ]}
                                />
                            </Form.Item>
                        </Col>
                        <Col xs={12}>
                            <Form.Item label="Serial Port">
                                <Input
                                    value={values.gps_port ?? "/dev/gps"}
                                    onChange={(e) => onChange("gps_port", e.target.value)}
                                />
                            </Form.Item>
                        </Col>
                    </Row>
                </Form>
            </Card>

            <Card
                size="small"
                title={
                    <Space>
                        <WifiOutlined />
                        <span>NTRIP Corrections (RTK)</span>
                        <Switch
                            size="small"
                            checked={ntripEnabled}
                            onChange={(v) => onChange("ntrip_enabled", v)}
                        />
                    </Space>
                }
            >
                {ntripEnabled ? (
                    <>
                        <Paragraph type="secondary" style={{ fontSize: 12, marginBottom: 12 }}>
                            NTRIP provides RTK corrections for centimetre-level accuracy.
                            Free networks like Centipede (France) or SAPOS (Germany) are available in many countries.
                        </Paragraph>
                        <Form layout="vertical">
                            <Row gutter={16}>
                                <Col xs={16}>
                                    <Form.Item label="Host">
                                        <Input
                                            value={values.ntrip_host ?? ""}
                                            onChange={(e) => onChange("ntrip_host", e.target.value)}
                                            placeholder="caster.centipede.fr"
                                        />
                                    </Form.Item>
                                </Col>
                                <Col xs={8}>
                                    <Form.Item label="Port">
                                        <InputNumber
                                            value={values.ntrip_port ?? 2101}
                                            onChange={(v) => onChange("ntrip_port", v)}
                                            style={{ width: "100%" }}
                                        />
                                    </Form.Item>
                                </Col>
                                <Col xs={24}>
                                    <Form.Item label="Mountpoint">
                                        <Input
                                            value={values.ntrip_mountpoint ?? ""}
                                            onChange={(e) => onChange("ntrip_mountpoint", e.target.value)}
                                            placeholder="OUIL"
                                        />
                                    </Form.Item>
                                </Col>
                                <Col xs={12}>
                                    <Form.Item label="Username">
                                        <Input
                                            value={values.ntrip_user ?? ""}
                                            onChange={(e) => onChange("ntrip_user", e.target.value)}
                                            placeholder="centipede"
                                        />
                                    </Form.Item>
                                </Col>
                                <Col xs={12}>
                                    <Form.Item label="Password">
                                        <Input.Password
                                            value={values.ntrip_password ?? ""}
                                            onChange={(e) => onChange("ntrip_password", e.target.value)}
                                            placeholder="centipede"
                                        />
                                    </Form.Item>
                                </Col>
                            </Row>
                        </Form>
                    </>
                ) : (
                    <Paragraph type="secondary">
                        NTRIP is disabled. Your GPS will operate without RTK corrections (lower accuracy, ~1-2m).
                    </Paragraph>
                )}
            </Card>
        </div>
    );
};

// ── Step 3: Sensor Placement ────────────────────────────────────────────

const SensorStep: React.FC<RobotModelStepProps> = ({ values, onChange }) => {
    return (
        <div style={{ maxWidth: 900, margin: "0 auto" }}>
            <Title level={4}>
                <AimOutlined /> Sensor Placement
            </Title>
            <Paragraph type="secondary" style={{ marginBottom: 16 }}>
                Position your sensors on the robot. Drag them on the top-down view or use the precise numeric inputs.
                These positions tell the robot exactly where each sensor is mounted relative to the chassis centre.
            </Paragraph>
            <RobotComponentEditor values={values} onChange={onChange} />
        </div>
    );
};

// ── Step 4: Firmware ────────────────────────────────────────────────────

const FirmwareStep: React.FC<{ onNext: () => void }> = ({ onNext }) => {
    const { colors } = useThemeMode();
    const [showFlash, setShowFlash] = useState(false);

    if (showFlash) {
        return (
            <Card title="Flash Firmware">
                <FlashBoardComponent onNext={onNext} />
            </Card>
        );
    }

    return (
        <div style={{ maxWidth: 600, margin: "0 auto", textAlign: "center", padding: "24px 0" }}>
            <div style={{
                width: 64, height: 64, borderRadius: "50%",
                background: colors.primaryBg, display: "flex",
                alignItems: "center", justifyContent: "center",
                margin: "0 auto 16px",
            }}>
                <ThunderboltOutlined style={{ fontSize: 28, color: colors.primary }} />
            </div>
            <Title level={4}>Firmware</Title>
            <Paragraph type="secondary" style={{ marginBottom: 24 }}>
                If this is a new build, you need to flash the Mowgli firmware onto your motherboard.
                If your firmware is already up to date, you can skip this step.
            </Paragraph>

            <Space size="middle">
                <Button type="primary" size="large" onClick={() => setShowFlash(true)}>
                    Flash Firmware
                </Button>
                <Button size="large" onClick={onNext}>
                    Skip — Already Flashed
                </Button>
            </Space>

            <Alert
                type="warning"
                showIcon
                message="Flashing will rewrite your motherboard firmware"
                description="Make sure your mower is connected via USB and powered on. Wrong voltage settings can damage hardware."
                style={{ marginTop: 24, textAlign: "left" }}
            />
        </div>
    );
};

// ── Step 5: Complete ────────────────────────────────────────────────────

const CompleteStep: React.FC = () => {
    const { colors } = useThemeMode();
    const guiApi = useApi();
    const [restarting, setRestarting] = useState(false);
    const [error, setError] = useState<string | null>(null);

    useEffect(() => {
        // Mark onboarding as completed and restart the OpenMower container
        (async () => {
            setRestarting(true);
            try {
                // Mark onboarding done in DB so we don't redirect again
                const base = import.meta.env.DEV ? 'http://localhost:4006' : '';
                await fetch(`${base}/api/settings/status`, { method: 'POST' });

                const res = await guiApi.containers.containersList();
                if (res.error) throw new Error(res.error.error);
                const container = res.data.containers?.find(
                    (c) => c.labels?.app === "openmower" || c.names?.includes("/openmower")
                );
                if (container?.id) {
                    const restartRes = await guiApi.containers.containersCreate(container.id, "restart");
                    if (restartRes.error) throw new Error(restartRes.error.error);
                }
            } catch (e: any) {
                setError(e.message);
            } finally {
                setRestarting(false);
            }
        })();
    }, []);

    if (restarting) {
        return (
            <Result
                icon={<RocketOutlined style={{ color: colors.primary }} spin />}
                title="Applying configuration..."
                subTitle="Restarting the mower service with your new settings. This takes a few seconds."
            />
        );
    }

    return (
        <Result
            icon={<CheckCircleOutlined style={{ color: colors.primary }} />}
            title="You're all set!"
            subTitle="Your mower is configured and ready to go. Head to the Map to draw your first mowing area, or check the Dashboard to monitor your robot."
            extra={[
                <Button
                    key="map"
                    type="primary"
                    size="large"
                    icon={<EnvironmentOutlined />}
                    onClick={() => { window.location.href = "/#/map"; }}
                >
                    Draw Mowing Area
                </Button>,
                <Button
                    key="dashboard"
                    size="large"
                    onClick={() => { window.location.href = "/#/openmower"; }}
                >
                    Go to Dashboard
                </Button>,
            ]}
        >
            {error && (
                <Alert
                    type="warning"
                    showIcon
                    message="Could not restart the mower service"
                    description={`${error}. You may need to restart it manually.`}
                    style={{ maxWidth: 500, margin: "0 auto" }}
                />
            )}
        </Result>
    );
};

// ── Main Setup Wizard ───────────────────────────────────────────────────

const STEP_ICONS = [
    <RocketOutlined />,
    <SettingOutlined />,
    <GlobalOutlined />,
    <AimOutlined />,
    <ThunderboltOutlined />,
    <CheckCircleOutlined />,
];

const STEP_TITLES = [
    "Welcome",
    "Robot Model",
    "GPS",
    "Sensors",
    "Firmware",
    "Complete",
];

const OnboardingWizard: React.FC = () => {
    const { colors } = useThemeMode();
    const isMobile = useIsMobile();
    const { values: savedValues, saveValues, loading } = useSettingsSchema();
    const [currentStep, setCurrentStep] = useState(0);
    const [localValues, setLocalValues] = useState<Record<string, any>>({});
    const [saving, setSaving] = useState(false);

    useEffect(() => {
        if (savedValues) {
            setLocalValues(savedValues);
        }
    }, [savedValues]);

    const handleChange = useCallback((key: string, value: any) => {
        setLocalValues((prev) => ({ ...prev, [key]: value }));
    }, []);

    const handleNext = useCallback(async () => {
        // Save settings when leaving config steps (1, 2, 3)
        if (currentStep >= 1 && currentStep <= 3) {
            setSaving(true);
            await saveValues(localValues);
            setSaving(false);
        }
        setCurrentStep((s) => Math.min(s + 1, STEP_TITLES.length - 1));
    }, [currentStep, localValues, saveValues]);

    const handlePrev = useCallback(() => {
        setCurrentStep((s) => Math.max(s - 1, 0));
    }, []);

    const isFirstStep = currentStep === 0;
    const isLastStep = currentStep === STEP_TITLES.length - 1;
    const isFirmwareStep = currentStep === 4;

    return (
        <Row gutter={[0, isMobile ? 12 : 20]}>
            {/* Steps indicator */}
            <Col span={24}>
                <Steps
                    current={currentStep}
                    size={isMobile ? "small" : "default"}
                    responsive={false}
                    items={STEP_TITLES.map((title, i) => ({
                        title: isMobile ? undefined : title,
                        icon: STEP_ICONS[i],
                    }))}
                    style={{ maxWidth: 700, margin: "0 auto" }}
                />
            </Col>

            {/* Step content */}
            <Col
                span={24}
                style={{
                    height: isMobile ? "auto" : "calc(100vh - 220px)",
                    overflowY: isMobile ? undefined : "auto",
                    paddingBottom: isLastStep || isFirmwareStep ? 16 : 80,
                }}
            >
                {currentStep === 0 && <WelcomeStep onNext={handleNext} />}
                {currentStep === 1 && <RobotModelStep values={localValues} onChange={handleChange} />}
                {currentStep === 2 && <GpsStep values={localValues} onChange={handleChange} />}
                {currentStep === 3 && <SensorStep values={localValues} onChange={handleChange} />}
                {currentStep === 4 && <FirmwareStep onNext={handleNext} />}
                {currentStep === 5 && <CompleteStep />}
            </Col>

            {/* Navigation bar (hidden on welcome, complete, and firmware steps) */}
            {!isFirstStep && !isLastStep && !isFirmwareStep && (
                <Col
                    span={24}
                    style={{
                        position: "fixed",
                        bottom: isMobile ? "calc(56px + env(safe-area-inset-bottom, 0px))" : 20,
                        left: isMobile ? 0 : undefined,
                        right: isMobile ? 0 : undefined,
                        padding: isMobile ? "8px 12px" : undefined,
                        background: isMobile ? colors.bgCard : undefined,
                        borderTop: isMobile ? `1px solid ${colors.border}` : undefined,
                        zIndex: 50,
                    }}
                >
                    <Space>
                        <Button icon={<ArrowLeftOutlined />} onClick={handlePrev}>
                            Back
                        </Button>
                        <Button
                            type="primary"
                            icon={currentStep < 3 ? <ArrowRightOutlined /> : <SaveOutlined />}
                            onClick={handleNext}
                            loading={saving || loading}
                        >
                            {currentStep < 3 ? "Next" : "Save & Continue"}
                        </Button>
                    </Space>
                </Col>
            )}
        </Row>
    );
};

export default OnboardingWizard;
