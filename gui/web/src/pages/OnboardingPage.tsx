import React, { useCallback, useEffect, useState } from "react";
import {
    Button, Card, Col, Row, Steps, Typography, Select, Space, Alert,
    Input, InputNumber, Switch, Form, Divider, Tag, Result,
} from "antd";
import {
    RocketOutlined, SettingOutlined, GlobalOutlined,
    AimOutlined, ThunderboltOutlined, CheckCircleOutlined,
    ArrowLeftOutlined, ArrowRightOutlined, SaveOutlined,
    EnvironmentOutlined, WifiOutlined, MobileOutlined,
} from "@ant-design/icons";
import { useThemeMode } from "../theme/ThemeContext.tsx";
import { useIsMobile } from "../hooks/useIsMobile";
import { useSettingsSchema } from "../hooks/useSettingsSchema.ts";
import { useApi } from "../hooks/useApi.ts";
import { useGPS } from "../hooks/useGPS.ts";
import { useCalibrationStatus } from "../hooks/useCalibrationStatus.ts";
import { useImuYawCalibration } from "../hooks/useImuYawCalibration.ts";
import { AbsolutePoseConstants as GpsFlags } from "../types/ros.ts";
import { CompassOutlined } from "@ant-design/icons";
import { RobotComponentEditor } from "../components/RobotComponentEditor.tsx";
import { FlashBoardComponent } from "../components/FlashBoardComponent.tsx";
import PairingQR from "../components/PairingQR.tsx";
import { usePairingStatus } from "../hooks/usePairingStatus.ts";
import { MOWER_MODELS } from "../constants/mowerModels.ts";
import { restartRos2, restartGui } from "../utils/containers.ts";

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

// MOWER_MODELS imported from constants/mowerModels.ts

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
    const guiApi = useApi();
    const [datumLoading, setDatumLoading] = useState(false);

    // Live GPS state — used to gate "Use current GPS position" on RTK-Fixed.
    // SBAS or 3D fix gives σ ~50 cm to ~5 m, which is enough to put the
    // datum metres off and silently break every later mow. RTK-Float drifts
    // when the receiver loses corrections briefly. Only RTK-Fixed (σ ~3 mm)
    // is safe to anchor the map frame to.
    const gps = useGPS();
    const gpsFlags = gps.flags ?? 0;
    const isRtkFixed = (gpsFlags & GpsFlags.FLAG_GPS_RTK_FIXED) !== 0;
    const isRtkFloat = (gpsFlags & GpsFlags.FLAG_GPS_RTK_FLOAT) !== 0;
    const isPlainFix = (gpsFlags & GpsFlags.FLAG_GPS_RTK) !== 0;
    const fixLabel = isRtkFixed ? "RTK FIX" : isRtkFloat ? "RTK FLOAT" : isPlainFix ? "GPS FIX" : "no fix";

    const setDatumFromGps = async () => {
        setDatumLoading(true);
        try {
            const res = await guiApi.mowglinext.callCreate("set_datum", {});
            if (res.error) throw new Error(res.error.error);
            const msg: string = (res.data as any)?.message ?? "";
            const parts = msg.split(",");
            if (parts.length === 2) {
                onChange("datum_lat", parseFloat(parts[0]));
                onChange("datum_lon", parseFloat(parts[1]));
            }
        } catch (e: any) {
            alert(e.message || "Failed to set datum from GPS");
        } finally {
            setDatumLoading(false);
        }
    };

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
                    You can get coordinates from Google Maps by right-clicking on your dock location, or use the
                    button below to set it from the robot's current GPS position (requires RTK fix).
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
                    <Button
                        icon={<AimOutlined />}
                        loading={datumLoading}
                        onClick={setDatumFromGps}
                        disabled={!isRtkFixed}
                        style={{ marginTop: -8 }}
                    >
                        Use current GPS position {isRtkFixed ? "" : "(waiting for RTK Fix)"}
                    </Button>
                    {!isRtkFixed && (
                        <Alert
                            type="warning"
                            showIcon
                            message={`Current GPS quality: ${fixLabel}`}
                            description={
                                isRtkFloat
                                    ? "RTK Float gives ~10–20 cm accuracy and drifts when corrections lapse. Wait for RTK Fix (σ ~3 mm) before anchoring the datum."
                                    : "Without RTK Fix the datum can be metres off, which silently breaks every later mow. Make sure NTRIP corrections are flowing and the antenna has clear sky."
                            }
                            style={{ marginTop: 12 }}
                        />
                    )}
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

// ── Step 4: IMU Yaw Calibration ─────────────────────────────────────────

// A wizard step that walks the operator through the IMU mounting yaw
// auto-calibration — previously buried as a tooltip-only compass icon
// inside the Sensors step's RobotComponentEditor. Without this step, a
// brand-new robot can finish onboarding with imu_yaw=0 and silently
// drift in odom.
//
// The step also captures dock_pose_x/y/yaw as a side effect when the
// robot starts the calibration on the dock (the ROS service's dock
// pre-phase writes it directly into mowgli_robot.yaml — see
// CalibrateImuYaw.srv response fields).

const ImuYawStep: React.FC<RobotModelStepProps> = ({values, onChange}) => {
    const {colors} = useThemeMode();
    const {status: calibrationStatus, refresh: refreshCalibrationStatus} = useCalibrationStatus();
    const {
        calibRunning,
        calibResult,
        resetCalibration,
        startCalibration,
        applyCalibration,
    } = useImuYawCalibration({
        onApplyValue: (key, value) => {
            onChange(key, value);
            // Dock pose is persisted server-side into mowgli_robot.yaml; refresh
            // the status panel so the operator sees the updated dock pose card.
            refreshCalibrationStatus();
        },
        currentImuYawRad: values.imu_yaw,
    });

    const dockPresent = !!calibrationStatus?.dock?.present;
    const imuPresent = !!calibrationStatus?.imu?.present;
    const currentImuYawDeg = (values.imu_yaw ?? 0) * 180 / Math.PI;

    return (
        <div style={{maxWidth: 700, margin: "0 auto"}}>
            <Title level={4}>
                <CompassOutlined/> IMU Mounting Calibration
            </Title>
            <Paragraph type="secondary" style={{marginBottom: 16}}>
                The IMU is mounted at an angle relative to the robot chassis. The robot needs to know that
                angle to integrate gyro yaw correctly — without it, the robot drifts in odom and can dock
                at an angle. This step drives the robot ~0.6 m forward then back to estimate the mounting
                yaw from the accelerometer.
            </Paragraph>

            <Card size="small" style={{marginBottom: 16}}>
                <Paragraph strong style={{marginBottom: 8}}>Pre-flight checklist</Paragraph>
                <ul style={{paddingLeft: 20, marginBottom: 0, color: colors.textSecondary, fontSize: 13}}>
                    <li>Robot is undocked (the service refuses to run while charging — but if you start it on the dock, the dock pre-phase computes <Text code>dock_pose_yaw</Text> as a bonus).</li>
                    <li>At least 1 m of clear space in front and behind the robot.</li>
                    <li>No active emergency.</li>
                    <li>Collision monitor stays armed — obstacles will stop the motion.</li>
                </ul>
            </Card>

            <Card size="small" style={{marginBottom: 16}}>
                <Row gutter={[16, 8]}>
                    <Col xs={12}>
                        <Text type="secondary" style={{fontSize: 11}}>Current imu_yaw</Text>
                        <div style={{fontSize: 18, fontWeight: 500}}>{currentImuYawDeg.toFixed(2)}°</div>
                    </Col>
                    <Col xs={12}>
                        <Text type="secondary" style={{fontSize: 11}}>Dock pose</Text>
                        <div style={{fontSize: 18, fontWeight: 500}}>
                            {dockPresent ? <Tag color="success">Present</Tag> : <Tag color="warning">Missing</Tag>}
                        </div>
                    </Col>
                    <Col xs={12}>
                        <Text type="secondary" style={{fontSize: 11}}>IMU bias</Text>
                        <div style={{fontSize: 18, fontWeight: 500}}>
                            {imuPresent ? <Tag color="success">Present</Tag> : <Tag color="warning">Missing</Tag>}
                        </div>
                    </Col>
                </Row>
            </Card>

            <div style={{textAlign: "center", marginBottom: 16}}>
                <Button
                    type="primary"
                    size="large"
                    icon={<CompassOutlined/>}
                    onClick={startCalibration}
                    loading={calibRunning}
                    disabled={calibRunning}
                >
                    {calibResult ? "Re-run calibration" : "Start IMU yaw calibration"}
                </Button>
            </div>

            {calibRunning && (
                <Alert
                    type="info"
                    showIcon
                    message="Calibration running — robot is driving itself"
                    description="Forward leg, pause, backward leg. Stand clear. Motion stops automatically. May take up to 2 minutes (longer when the dock pre-phase runs)."
                    style={{marginBottom: 16}}
                />
            )}

            {calibResult && calibResult.success && (
                <Alert
                    type="success"
                    showIcon
                    message={`imu_yaw = ${calibResult.imu_yaw_deg.toFixed(2)}° (σ ±${calibResult.std_dev_deg.toFixed(2)}°)`}
                    description={
                        <>
                            <div>From {calibResult.samples_used} valid motion samples.</div>
                            {calibResult.dock_valid && (
                                <div>
                                    Dock pose updated: yaw={calibResult.dock_pose_yaw_deg?.toFixed(2)}°
                                    (σ {calibResult.dock_yaw_sigma_deg?.toFixed(2)}°,
                                    displacement {calibResult.dock_undock_displacement_m?.toFixed(2) ?? "?"} m).
                                </div>
                            )}
                            <div style={{marginTop: 12}}>
                                <Space>
                                    <Button type="primary" onClick={applyCalibration}>Apply to settings</Button>
                                    <Button onClick={resetCalibration}>Discard</Button>
                                </Space>
                            </div>
                        </>
                    }
                    style={{marginBottom: 16}}
                />
            )}

            {calibResult && !calibResult.success && (
                <Alert
                    type="error"
                    showIcon
                    message="Calibration failed"
                    description={
                        <>
                            <div>{calibResult.message}</div>
                            <div style={{marginTop: 8, color: colors.textSecondary}}>
                                Hint: drive faster or longer so the accelerometer sees a clear forward and
                                backward impulse along the body X axis.
                            </div>
                            <div style={{marginTop: 12}}>
                                <Button onClick={resetCalibration}>Reset</Button>
                            </div>
                        </>
                    }
                    style={{marginBottom: 16}}
                />
            )}

            <Alert
                type="info"
                showIcon
                message="You can skip this step"
                description="If you have already calibrated this robot from the Diagnostics page (or know your imu_yaw value), use the Next button. The Complete step will warn you if calibration is still missing."
                style={{maxWidth: 500, margin: "0 auto"}}
            />
        </div>
    );
};

// ── Step 5: Firmware ────────────────────────────────────────────────────

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

// ── Step 6: Pair Mobile App ─────────────────────────────────────────────

const PairMobileAppStep: React.FC<{ onNext: () => void }> = ({ onNext }) => {
    const { colors } = useThemeMode();
    const isMobile = useIsMobile();
    const { status, qr, error, refresh, confirm, reset } = usePairingStatus(1500);
    const qrSize = isMobile ? 220 : 280;

    // Auto-advance to the next step two seconds after a successful pairing
    useEffect(() => {
        if (status?.state === "paired") {
            const timer = setTimeout(onNext, 2000);
            return () => clearTimeout(timer);
        }
    }, [status?.state, onNext]);

    if (error && !qr) {
        return (
            <div style={{ maxWidth: 560, margin: "0 auto", textAlign: "center", padding: "24px 0" }}>
                <Result
                    status="error"
                    title="Could not load pairing QR"
                    subTitle={error}
                    extra={[
                        <Button key="retry" type="primary" onClick={refresh}>
                            Retry
                        </Button>,
                        <Button key="skip" onClick={onNext}>
                            Skip — I'll pair later
                        </Button>,
                    ]}
                />
            </div>
        );
    }

    if (status?.state === "paired") {
        return (
            <div style={{ maxWidth: 560, margin: "0 auto", textAlign: "center", padding: "24px 0" }}>
                <Result
                    status="success"
                    title="Mobile app paired!"
                    subTitle={
                        status.ownerName
                            ? `${status.ownerName}'s app is now connected to this robot.`
                            : "The mobile app is now connected to this robot."
                    }
                    extra={
                        <Text type="secondary">Continuing in a moment…</Text>
                    }
                />
            </div>
        );
    }

    if (status?.state === "pending") {
        return (
            <div style={{ maxWidth: 560, margin: "0 auto", padding: "24px 0" }}>
                <Title level={4} style={{ textAlign: "center" }}>
                    <MobileOutlined /> Confirm Pairing
                </Title>
                <Paragraph type="secondary" style={{ textAlign: "center", marginBottom: 24 }}>
                    {status.ownerName ? `${status.ownerName} is pairing this robot.` : "Someone is pairing this robot."}
                    {" "}Confirm the 4-digit code matches what you see in the app.
                </Paragraph>

                <Card style={{ textAlign: "center", marginBottom: 16 }}>
                    <Text type="secondary" style={{ fontSize: 13 }}>Robot ID</Text>
                    <div style={{ fontFamily: "monospace", fontSize: 18, fontWeight: 600, marginBottom: 16 }}>
                        {status.robotID}
                    </div>
                    <Text type="secondary" style={{ fontSize: 13 }}>Confirmation code</Text>
                    <div style={{
                        fontFamily: "monospace",
                        fontSize: 48,
                        fontWeight: 700,
                        letterSpacing: "0.25em",
                        color: colors.primary,
                        margin: "8px 0 20px",
                    }}>
                        {status.confirmCode ?? "----"}
                    </div>

                    <Space direction={isMobile ? "vertical" : "horizontal"} style={{ width: isMobile ? "100%" : undefined }}>
                        <Button
                            type="primary"
                            size="large"
                            style={{ width: isMobile ? "100%" : undefined }}
                            onClick={confirm}
                        >
                            Confirm on robot
                        </Button>
                        <Button
                            size="large"
                            style={{ width: isMobile ? "100%" : undefined }}
                            onClick={reset}
                        >
                            Cancel
                        </Button>
                    </Space>
                </Card>

                {error && (
                    <Alert type="error" showIcon message={error} style={{ marginBottom: 16 }} />
                )}

                <div style={{ textAlign: "center" }}>
                    <Button type="link" onClick={onNext}>Skip — I'll pair later</Button>
                </div>
            </div>
        );
    }

    if (status?.state === "failed") {
        return (
            <div style={{ maxWidth: 560, margin: "0 auto", textAlign: "center", padding: "24px 0" }}>
                <Result
                    status="error"
                    title="Pairing failed"
                    subTitle="Something went wrong during pairing. You can retry or skip and pair later from the Settings page."
                    extra={[
                        <Button key="retry" type="primary" onClick={refresh}>
                            Retry
                        </Button>,
                        <Button key="skip" onClick={onNext}>
                            Skip — I'll pair later
                        </Button>,
                    ]}
                />
            </div>
        );
    }

    // Default view: show the QR code (states: unstarted, null/loading)
    return (
        <div style={{ maxWidth: 560, margin: "0 auto", padding: "24px 0" }}>
            <Title level={4} style={{ textAlign: "center" }}>
                <MobileOutlined /> Pair the Mobile App
            </Title>
            <Paragraph type="secondary" style={{ textAlign: "center", marginBottom: 24 }}>
                Scan this QR code with the MowgliNext mobile app to pair it with this robot.
                The app and robot must be on the same Wi-Fi for the first connection.
            </Paragraph>

            <Card style={{ textAlign: "center", marginBottom: 16 }}>
                {qr ? (
                    <>
                        <div style={{ display: "inline-block", padding: 12, background: "#fff", borderRadius: 8, marginBottom: 16 }}>
                            <PairingQR value={qr.payload} size={qrSize} />
                        </div>
                        <div style={{ marginBottom: 8 }}>
                            <Text type="secondary" style={{ fontSize: 13 }}>Robot ID</Text>
                            <div style={{ fontFamily: "monospace", fontSize: 16, fontWeight: 600 }}>
                                {qr.robotID}
                            </div>
                        </div>
                        <Paragraph type="secondary" style={{ fontSize: 11, maxWidth: 380, margin: "12px auto 16px", textAlign: "left" }}>
                            The QR code encodes this robot's encryption public key, a one-time setup token,
                            its local IP address, and its future remote hostname — everything the app needs
                            to establish a secure, end-to-end encrypted channel.
                        </Paragraph>
                    </>
                ) : (
                    <div style={{ height: qrSize + 32, display: "flex", alignItems: "center", justifyContent: "center" }}>
                        <Text type="secondary">Loading QR code…</Text>
                    </div>
                )}

                {error && (
                    <Alert type="warning" showIcon message={error} style={{ marginBottom: 12, textAlign: "left" }} />
                )}

                <Space direction={isMobile ? "vertical" : "horizontal"} style={{ width: isMobile ? "100%" : undefined }}>
                    <Button
                        type="link"
                        style={{ width: isMobile ? "100%" : undefined }}
                        onClick={onNext}
                    >
                        Skip — I'll pair later
                    </Button>
                </Space>
            </Card>

            <Paragraph type="secondary" style={{ fontSize: 12, textAlign: "center" }}>
                Waiting for the app to scan…
            </Paragraph>
        </div>
    );
};

// ── Step 7: Complete ────────────────────────────────────────────────────

const CompleteStep: React.FC = () => {
    const { colors } = useThemeMode();
    const guiApi = useApi();
    const [restarting, setRestarting] = useState(false);
    const [error, setError] = useState<string | null>(null);

    // Calibration completeness check — see docs/ONBOARDING_IMPROVEMENTS.md
    // gap analysis. The wizard never gates on these, so a brand-new robot
    // can finish "configured" with no dock pose, no IMU mounting calibration
    // and no magnetometer. Here we surface what is actually missing and
    // deep-link the operator to the Diagnostics page where they can run
    // each calibration without restarting the wizard.
    const { status: calibrationStatus } = useCalibrationStatus();
    const missingCalibrations: string[] = [];
    if (calibrationStatus) {
        if (!calibrationStatus.dock?.present) missingCalibrations.push("dock pose");
        if (!calibrationStatus.imu?.present) missingCalibrations.push("IMU bias + mounting");
        // Magnetometer is optional — only warn when use_magnetometer is on
        // (no good signal client-side yet, so we just don't flag mag here).
    }

    useEffect(() => {
        // Mark onboarding as completed and restart ROS2 + GUI containers
        (async () => {
            setRestarting(true);
            try {
                // Mark onboarding done in DB so we don't redirect again
                const base = import.meta.env.DEV ? 'http://localhost:4006' : '';
                await fetch(`${base}/api/settings/status`, { method: 'POST' });

                // Restart ROS2 container first (picks up new mowgli_robot.yaml)
                await restartRos2(guiApi);
                // Then restart GUI container
                await restartGui(guiApi);
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
                    onClick={() => { window.location.href = "/#/mowglinext"; }}
                >
                    Go to Dashboard
                </Button>,
            ]}
        >
            {missingCalibrations.length > 0 && (
                <Alert
                    type="warning"
                    showIcon
                    message="Calibration steps still pending"
                    description={
                        <>
                            <Text>
                                Your robot is configured, but{" "}
                                <Text strong>{missingCalibrations.join(" and ")}</Text>{" "}
                                {missingCalibrations.length === 1 ? "is" : "are"} not calibrated yet.
                                Without these the robot will drift in odom and may dock at an angle.
                            </Text>
                            <br />
                            <Button
                                type="link"
                                style={{ paddingLeft: 0 }}
                                onClick={() => { window.location.href = "/#/diagnostics"; }}
                            >
                                Open Diagnostics → run calibrations →
                            </Button>
                        </>
                    }
                    style={{ maxWidth: 540, margin: "0 auto 12px", textAlign: "left" }}
                />
            )}
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
    <CompassOutlined />,
    <ThunderboltOutlined />,
    <MobileOutlined />,
    <CheckCircleOutlined />,
];

const STEP_TITLES = [
    "Welcome",
    "Robot Model",
    "GPS",
    "Sensors",
    "IMU Yaw",
    "Firmware",
    "Pair App",
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
        // Save settings when leaving config steps (Robot Model, GPS, Sensors,
        // IMU Yaw — i.e. 1, 2, 3, 4). Apply-from-calibration writes through
        // onChange but doesn't auto-save; this is the one batch save point.
        if (currentStep >= 1 && currentStep <= 4) {
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
    const isFirmwareStep = currentStep === 5;
    const isPairStep = currentStep === 6;

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
                    paddingBottom: isLastStep || isFirmwareStep || isPairStep ? 16 : 80,
                }}
            >
                {currentStep === 0 && <WelcomeStep onNext={handleNext} />}
                {currentStep === 1 && <RobotModelStep values={localValues} onChange={handleChange} />}
                {currentStep === 2 && <GpsStep values={localValues} onChange={handleChange} />}
                {currentStep === 3 && <SensorStep values={localValues} onChange={handleChange} />}
                {currentStep === 4 && <ImuYawStep values={localValues} onChange={handleChange} />}
                {currentStep === 5 && <FirmwareStep onNext={handleNext} />}
                {currentStep === 6 && <PairMobileAppStep onNext={handleNext} />}
                {currentStep === 7 && <CompleteStep />}
            </Col>

            {/* Navigation bar (hidden on welcome, complete, firmware, and pair steps) */}
            {!isFirstStep && !isLastStep && !isFirmwareStep && !isPairStep && (
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
                            icon={currentStep < 4 ? <ArrowRightOutlined /> : <SaveOutlined />}
                            onClick={handleNext}
                            loading={saving || loading}
                        >
                            {currentStep < 4 ? "Next" : "Save & Continue"}
                        </Button>
                    </Space>
                </Col>
            )}
        </Row>
    );
};

export default OnboardingWizard;
