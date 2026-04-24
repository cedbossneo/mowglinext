import {
    Alert,
    Button,
    Card,
    Col,
    Collapse,
    Flex,
    Progress,
    Row,
    Space,
    Statistic,
    Table,
    Tag,
    Typography,
} from "antd";
import {
    ApiOutlined,
    CloudServerOutlined,
    CompassOutlined,
    DashboardOutlined,
    ReloadOutlined,
    SoundOutlined,
    ThunderboltOutlined,
    WarningOutlined,
    WifiOutlined,
} from "@ant-design/icons";
import {useHighLevelStatus} from "../hooks/useHighLevelStatus.ts";
import {useEmergency} from "../hooks/useEmergency.ts";
import {usePower} from "../hooks/usePower.ts";
import {useStatus} from "../hooks/useStatus.ts";
import {useGPS} from "../hooks/useGPS.ts";
import {useFusionOdom} from "../hooks/useFusionOdom.ts";
import {useBTLog} from "../hooks/useBTLog.ts";
import {useImu} from "../hooks/useImu.ts";
import {useCogHeading} from "../hooks/useCogHeading.ts";
import {useMagYaw} from "../hooks/useMagYaw.ts";
import {useWheelTicks} from "../hooks/useWheelTicks.ts";
import {useDiagnosticsSnapshot} from "../hooks/useDiagnosticsSnapshot.ts";
import {useDiagnostics} from "../hooks/useDiagnostics.ts";
import {useThemeMode} from "../theme/ThemeContext.tsx";
import {useIsMobile} from "../hooks/useIsMobile";
import {AbsolutePoseConstants} from "../types/ros.ts";
import {useEffect, useMemo, useState} from "react";
import {useSettings} from "../hooks/useSettings.ts";
import {computeBatteryPercent} from "../utils/battery.ts";

// ── helpers ─────────────────────────────────────────────────────────────────

function yawFromQuaternion(x = 0, y = 0, z = 0, w = 1): number {
    return Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)) * (180 / Math.PI);
}

function rollFromQuaternion(x = 0, y = 0, z = 0, w = 1): number {
    return Math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)) * (180 / Math.PI);
}

function pitchFromQuaternion(x = 0, y = 0, z = 0, w = 1): number {
    const sinp = 2 * (w * y - z * x);
    return Math.abs(sinp) >= 1 ? (Math.sign(sinp) * 90) : Math.asin(sinp) * (180 / Math.PI);
}

function secondsAgo(timestamp: string): number {
    return Math.floor((Date.now() - new Date(timestamp).getTime()) / 1000);
}

const DIAG_LEVEL_COLORS: Record<number, string> = {0: "success", 1: "warning", 2: "error", 3: "default"};
const DIAG_LEVEL_LABELS: Record<number, string> = {0: "OK", 1: "WARN", 2: "ERROR", 3: "STALE"};

// ESC status codes from mowgli_interfaces/msg/ESCStatus.msg
const ESC_STATUS: Record<number, {label: string; color: string}> = {
    0:   {label: "Off", color: "default"},
    99:  {label: "Disconnected", color: "warning"},
    100: {label: "Error", color: "error"},
    150: {label: "Stalled", color: "error"},
    200: {label: "OK", color: "success"},
    201: {label: "Running", color: "success"},
};

// ── sub-components ───────────────────────────────────────────────────────────

function HealthBadge({label, color}: {label: string; color: string}) {
    return <Tag color={color} style={{fontSize: 12, padding: "2px 8px"}}>{label}</Tag>;
}

// ── main page ────────────────────────────────────────────────────────────────

export const DiagnosticsPage = () => {
    const {colors} = useThemeMode();
    const isMobile = useIsMobile();

    const {highLevelStatus} = useHighLevelStatus();
    const emergency = useEmergency();
    const power = usePower();
    const status = useStatus();
    const gps = useGPS();
    const pose = useFusionOdom();
    const btNodeStates = useBTLog();
    const imu = useImu();
    const {imu: cogImu, lastMessageAt: cogLastAt} = useCogHeading();
    const {imu: magImu, lastMessageAt: magLastAt} = useMagYaw();
    const wheelTicks = useWheelTicks();

    // Tick state once a second so the "Live/Stale" tags update even when no
    // new message has arrived (staleness is time-based, not message-driven).
    const [nowMs, setNowMs] = useState(Date.now());
    useEffect(() => {
        const id = setInterval(() => setNowMs(Date.now()), 1000);
        return () => clearInterval(id);
    }, []);
    const {snapshot, loading, refresh} = useDiagnosticsSnapshot();
    const {diagnostics} = useDiagnostics();
    const {settings} = useSettings();

    // ── derived values ───────────────────────────────────────────────────────

    const batteryPercent = useMemo(
        () => computeBatteryPercent(highLevelStatus.battery_percent, power.v_battery, settings),
        [highLevelStatus.battery_percent, power.v_battery, settings],
    );

    const gpsFlags = gps.flags ?? 0;
    const gpsFixType = useMemo(() => {
        if (gpsFlags & AbsolutePoseConstants.FLAG_GPS_RTK_FIXED) return "RTK FIX";
        if (gpsFlags & AbsolutePoseConstants.FLAG_GPS_RTK_FLOAT) return "RTK FLOAT";
        if (gpsFlags & AbsolutePoseConstants.FLAG_GPS_RTK) return "GPS FIX";
        return "No Fix";
    }, [gpsFlags]);

    const orientation = pose.pose?.pose?.orientation;
    const qx = orientation?.x ?? 0;
    const qy = orientation?.y ?? 0;
    const qz = orientation?.z ?? 0;
    const qw = orientation?.w ?? 1;
    const yaw = yawFromQuaternion(qx, qy, qz, qw);
    const roll = rollFromQuaternion(qx, qy, qz, qw);
    const pitch = pitchFromQuaternion(qx, qy, qz, qw);
    const poseZ = pose.pose?.pose?.position?.z ?? 0;

    const allContainersOk = !snapshot?.containers?.length || snapshot.containers.every(c => c.state === "running");
    const gpsOk = gpsFlags > 0 && (gps.position_accuracy ?? 999) <= 0.1;
    const gpsWarn = gpsFlags > 0 && (gps.position_accuracy ?? 999) > 0.1;
    const cpuTemp = snapshot?.system?.cpu_temperature ?? 0;

    const alerts = useMemo(
        () => (diagnostics.status ?? []).filter(s =>
            s.level >= 1 &&
            // Filter out transient "no data since last update" from ublox driver
            !s.message?.toLowerCase().includes("no data since last update")
        ),
        [diagnostics.status]
    );

    // ── Health Summary Bar ───────────────────────────────────────────────────

    const healthBar = (
        <Card size="small" style={{marginBottom: 12}}>
            <Flex wrap gap="small" align="center">
                <Typography.Text type="secondary" style={{fontSize: 12, marginRight: 4}}>Health</Typography.Text>
                <HealthBadge
                    label={allContainersOk ? "Containers OK" : "Container Issue"}
                    color={allContainersOk ? "success" : "error"}
                />
                <HealthBadge
                    label={`GPS: ${gpsFixType}`}
                    color={gpsOk ? "success" : gpsWarn ? "warning" : "error"}
                />
                <HealthBadge
                    label={`Battery: ${batteryPercent.toFixed(0)}%`}
                    color={batteryPercent > 50 ? "success" : batteryPercent > 20 ? "warning" : "error"}
                />
                <HealthBadge
                    label={emergency.active_emergency ? "EMERGENCY" : "No Emergency"}
                    color={emergency.active_emergency ? "error" : "success"}
                />
                <HealthBadge
                    label={cpuTemp > 0 ? `CPU: ${cpuTemp.toFixed(1)}°C` : "CPU: --"}
                    color={cpuTemp > 70 ? "error" : cpuTemp > 55 ? "warning" : "success"}
                />
            </Flex>
        </Card>
    );

    // ── Section 1: System ────────────────────────────────────────────────────

    const containerColumns = [
        {
            title: "Name",
            dataIndex: "name",
            key: "name",
            render: (v: string) => <Typography.Text code style={{fontSize: 12}}>{v}</Typography.Text>,
        },
        {
            title: "State",
            dataIndex: "state",
            key: "state",
            render: (v: string) => <Tag color={v === "running" ? "success" : "error"}>{v}</Tag>,
        },
        {
            title: "Status",
            dataIndex: "status",
            key: "status",
            render: (v: string) => <Typography.Text style={{fontSize: 12}}>{v}</Typography.Text>,
        },
        {
            title: "Started",
            dataIndex: "started_at",
            key: "started_at",
            render: (v: string) => (
                <Typography.Text style={{fontSize: 12}}>
                    {v ? new Date(v).toLocaleTimeString() : "--"}
                </Typography.Text>
            ),
        },
    ];

    const sectionSystem = (
        <Row gutter={[12, 12]}>
            <Col span={24}>
                <Card
                    title={<Space><CloudServerOutlined/> Containers</Space>}
                    size="small"
                    extra={
                        <Button
                            size="small"
                            icon={<ReloadOutlined spin={loading}/>}
                            onClick={refresh}
                        >
                            Refresh
                        </Button>
                    }
                >
                    <Table
                        size="small"
                        dataSource={snapshot?.containers ?? []}
                        columns={containerColumns}
                        rowKey="name"
                        pagination={false}
                        locale={{emptyText: "No container data"}}
                    />
                </Card>
            </Col>
            <Col xs={24} lg={8}>
                <Card title={<Space><DashboardOutlined/> CPU</Space>} size="small">
                    <Statistic
                        title="Temperature"
                        value={cpuTemp > 0 ? cpuTemp : undefined}
                        precision={1}
                        suffix="°C"
                        valueStyle={{
                            color: cpuTemp > 70 ? colors.danger : cpuTemp > 55 ? colors.warning : undefined,
                        }}
                       
                    />
                </Card>
            </Col>
            {snapshot?.timestamp && (
                <Col span={24}>
                    <Typography.Text type="secondary" style={{fontSize: 12}}>
                        Last snapshot: {secondsAgo(snapshot.timestamp)}s ago
                    </Typography.Text>
                </Col>
            )}
        </Row>
    );

    // ── Section 2: Localization ──────────────────────────────────────────────

    const zDriftColor = poseZ > 2 ? colors.danger : poseZ > 0.5 ? colors.warning : undefined;
    const flatCheck = Math.abs(roll) < 5 && Math.abs(pitch) < 5;
    const gpsFixColor = gpsFixType === "RTK FIX"
        ? colors.primary
        : gpsFixType === "RTK FLOAT"
            ? colors.warning
            : colors.danger;

    const sectionLocalization = (
        <Row gutter={[12, 12]}>
            <Col xs={24} lg={12}>
                <Card title={<Space><CompassOutlined/> Filtered Pose (map frame)</Space>} size="small"
                      extra={pose.pose?.pose?.position ? <Tag color="success">Live</Tag> : <Tag>Waiting...</Tag>}>
                    <Row gutter={[12, 12]}>
                        <Col span={8}>
                            <Statistic
                                title="X (m)"
                                value={pose.pose?.pose?.position?.x ?? "-"}
                                precision={pose.pose?.pose?.position ? 3 : undefined}

                            />
                        </Col>
                        <Col span={8}>
                            <Statistic
                                title="Y (m)"
                                value={pose.pose?.pose?.position?.y ?? "-"}
                                precision={pose.pose?.pose?.position ? 3 : undefined}

                            />
                        </Col>
                        <Col span={8}>
                            <Statistic
                                title="Z (m)"
                                value={pose.pose?.pose?.position ? poseZ : "-"}
                                precision={pose.pose?.pose?.position ? 3 : undefined}
                                valueStyle={zDriftColor ? {color: zDriftColor} : undefined}

                            />
                        </Col>
                        <Col span={8}>
                            <Statistic
                                title="Yaw (deg)"
                                value={yaw}
                                precision={1}
                                suffix="°"
                               
                            />
                        </Col>
                        <Col span={8}>
                            <Statistic
                                title="Roll (deg)"
                                value={roll}
                                precision={1}
                                suffix="°"
                               
                            />
                        </Col>
                        <Col span={8}>
                            <Statistic
                                title="Pitch (deg)"
                                value={pitch}
                                precision={1}
                                suffix="°"
                               
                            />
                        </Col>
                        <Col span={12}>
                            <Statistic
                                title="Z Drift"
                                value={poseZ.toFixed(3)}
                                suffix="m"
                                valueStyle={zDriftColor ? {color: zDriftColor} : undefined}
                            />
                        </Col>
                        <Col span={12}>
                            <Statistic
                                title="Flat Check"
                                value={flatCheck ? "OK" : "DRIFT"}
                                valueStyle={{color: flatCheck ? undefined : colors.warning}}
                            />
                        </Col>
                    </Row>
                </Card>
            </Col>
            <Col xs={24} lg={12}>
                <Card title={<Space><WifiOutlined/> GPS</Space>} size="small">
                    <Row gutter={[12, 12]}>
                        <Col span={24}>
                            <Space>
                                <Typography.Text type="secondary" style={{fontSize: 12}}>Fix Type</Typography.Text>
                                <Tag color={gpsFixColor === colors.primary ? "blue" : gpsFixColor === colors.warning ? "warning" : "error"}>
                                    {gpsFixType}
                                </Tag>
                            </Space>
                        </Col>
                        <Col span={12}>
                            <Statistic
                                title="X (m)"
                                value={gps.pose?.pose?.position?.x}
                                precision={3}
                               
                            />
                        </Col>
                        <Col span={12}>
                            <Statistic
                                title="Y (m)"
                                value={gps.pose?.pose?.position?.y}
                                precision={3}
                               
                            />
                        </Col>
                        <Col span={12}>
                            <Statistic
                                title="Altitude (m)"
                                value={gps.pose?.pose?.position?.z}
                                precision={3}
                               
                            />
                        </Col>
                        <Col span={12}>
                            <Statistic
                                title="Accuracy (m)"
                                value={gps.position_accuracy}
                                precision={3}
                                valueStyle={
                                    (gps.position_accuracy ?? 0) > 0.1
                                        ? {color: colors.warning}
                                        : undefined
                                }
                               
                            />
                        </Col>
                    </Row>
                </Card>
            </Col>
        </Row>
    );

    // ── Section 2b: Heading Sources ──────────────────────────────────────────
    // Shows the two synthetic absolute-yaw Imu publishers fused by ekf_map
    // alongside the filter output for comparison. Staleness threshold: 5 s.

    const STALE_MS = 5000;
    const cogStale = cogLastAt === null || (nowMs - cogLastAt) > STALE_MS;
    const magStale = magLastAt === null || (nowMs - magLastAt) > STALE_MS;

    const cogYawDeg = cogImu?.orientation
        ? yawFromQuaternion(cogImu.orientation.x, cogImu.orientation.y, cogImu.orientation.z, cogImu.orientation.w)
        : null;
    const magYawDeg = magImu?.orientation
        ? yawFromQuaternion(magImu.orientation.x, magImu.orientation.y, magImu.orientation.z, magImu.orientation.w)
        : null;

    // orientation_covariance is a flat length-9 row-major 3×3; yaw variance
    // sits at index 8 (same convention used by cog_to_imu.py / mag_yaw_publisher.py).
    const cogYawVar = cogImu?.orientation_covariance?.[8];
    const magYawVar = magImu?.orientation_covariance?.[8];
    const cogSigmaDeg = (cogYawVar !== undefined && cogYawVar > 0) ? Math.sqrt(cogYawVar) * (180 / Math.PI) : null;
    const magSigmaDeg = (magYawVar !== undefined && magYawVar > 0) ? Math.sqrt(magYawVar) * (180 / Math.PI) : null;

    // Wrap angle difference into (-180, 180].
    const wrap180 = (d: number) => ((d + 180) % 360 + 360) % 360 - 180;
    const deltaFilterMag = (!magStale && magYawDeg !== null) ? wrap180(yaw - magYawDeg) : null;
    const deltaFilterCog = (!cogStale && cogYawDeg !== null) ? wrap180(yaw - cogYawDeg) : null;

    const sectionHeadingSources = (
        <Row gutter={[12, 12]}>
            <Col span={24}>
                <Card title={<Space><CompassOutlined/> Heading sources (fused by ekf_map)</Space>} size="small">
                    <Row gutter={[12, 12]}>
                        <Col xs={24} md={8}>
                            <Space direction="vertical" style={{width: "100%"}}>
                                <Space>
                                    <Typography.Text strong>Filter</Typography.Text>
                                    <Tag color="success">/odometry/filtered_map</Tag>
                                </Space>
                                <Statistic title="Yaw (deg)" value={yaw} precision={1} suffix="°"/>
                                <Typography.Text type="secondary" style={{fontSize: 11}}>
                                    Reference signal for deltas below.
                                </Typography.Text>
                            </Space>
                        </Col>
                        <Col xs={24} md={8}>
                            <Space direction="vertical" style={{width: "100%"}}>
                                <Space>
                                    <Typography.Text strong>COG (GPS)</Typography.Text>
                                    <Tag color={cogStale ? "default" : "processing"}>
                                        {cogStale ? "Stale" : "Live"}
                                    </Tag>
                                </Space>
                                <Statistic
                                    title="Yaw (deg)"
                                    value={cogYawDeg !== null ? cogYawDeg : "-"}
                                    precision={cogYawDeg !== null ? 1 : undefined}
                                    suffix={cogYawDeg !== null ? "°" : undefined}
                                />
                                <Typography.Text type="secondary" style={{fontSize: 12}}>
                                    σ: {cogSigmaDeg !== null ? `${cogSigmaDeg.toFixed(2)}°` : "—"}
                                </Typography.Text>
                                {deltaFilterCog !== null && (
                                    <Typography.Text type="secondary" style={{fontSize: 12}}>
                                        Δ(filter−cog): {deltaFilterCog.toFixed(1)}°
                                    </Typography.Text>
                                )}
                            </Space>
                        </Col>
                        <Col xs={24} md={8}>
                            <Space direction="vertical" style={{width: "100%"}}>
                                <Space>
                                    <Typography.Text strong>Magnetometer</Typography.Text>
                                    <Tag color={magStale ? "default" : "processing"}>
                                        {magStale ? "Stale" : "Live"}
                                    </Tag>
                                </Space>
                                <Statistic
                                    title="Yaw (deg)"
                                    value={magYawDeg !== null ? magYawDeg : "-"}
                                    precision={magYawDeg !== null ? 1 : undefined}
                                    suffix={magYawDeg !== null ? "°" : undefined}
                                />
                                <Typography.Text type="secondary" style={{fontSize: 12}}>
                                    σ: {magSigmaDeg !== null ? `${magSigmaDeg.toFixed(2)}°` : "—"}
                                </Typography.Text>
                                {deltaFilterMag !== null && (
                                    <Typography.Text type="secondary" style={{fontSize: 12}}>
                                        Δ(filter−mag): {deltaFilterMag.toFixed(1)}°
                                    </Typography.Text>
                                )}
                            </Space>
                        </Col>
                    </Row>
                </Card>
            </Col>
        </Row>
    );

    // ── Section 3: BT State & Coverage ───────────────────────────────────────

    const btStateColor =
        highLevelStatus.state === 0 ? "error" :
        highLevelStatus.state === 2 ? "processing" :
        highLevelStatus.state === 3 ? "warning" :
        highLevelStatus.state === 4 ? "cyan" :
        "default";

    const coverageColumns = [
        {title: "Area", dataIndex: "area_index", key: "area_index"},
        {
            title: "Coverage",
            dataIndex: "coverage_percent",
            key: "coverage_percent",
            render: (v: number) => <Progress percent={Math.round(v * 100) / 100} size="small" style={{minWidth: 80}}/>,
        },
        {title: "Total Cells", dataIndex: "total_cells", key: "total_cells"},
        {title: "Mowed", dataIndex: "mowed_cells", key: "mowed_cells"},
        {title: "Obstacles", dataIndex: "obstacle_cells", key: "obstacle_cells"},
        {title: "Strips Left", dataIndex: "strips_remaining", key: "strips_remaining"},
    ];

    const sectionBtCoverage = (
        <Row gutter={[12, 12]}>
            <Col xs={24} lg={12}>
                <Card title={<Space><ApiOutlined/> BT State</Space>} size="small">
                    <Space direction="vertical" style={{width: "100%"}}>
                        <Space>
                            <Typography.Text type="secondary" style={{fontSize: 12}}>State</Typography.Text>
                            <Tag color={btStateColor} style={{fontSize: 14, padding: "2px 12px"}}>
                                {highLevelStatus.state_name ?? "--"}
                            </Tag>
                        </Space>
                        {highLevelStatus.sub_state_name && (
                            <Space>
                                <Typography.Text type="secondary" style={{fontSize: 12}}>Sub-state</Typography.Text>
                                <Tag>{highLevelStatus.sub_state_name}</Tag>
                            </Space>
                        )}
                    </Space>
                    <Row gutter={[12, 12]} style={{marginTop: 12}}>
                        <Col span={8}>
                            <Statistic
                                title="Battery"
                                value={batteryPercent}
                                precision={0}
                                suffix="%"
                                valueStyle={{
                                    color: batteryPercent < 20 ? colors.danger : batteryPercent < 50 ? colors.warning : undefined,
                                }}
                            />
                        </Col>
                        <Col span={8}>
                            <Statistic
                                title="Voltage"
                                value={power.v_battery}
                                precision={2}
                                suffix="V"
                               
                            />
                        </Col>
                        <Col span={8}>
                            <Statistic
                                title="Charging"
                                value={highLevelStatus.is_charging ? "Yes" : "No"}
                                valueStyle={{
                                    color: highLevelStatus.is_charging ? colors.primary : undefined,
                                }}
                            />
                        </Col>
                    </Row>
                    <div style={{marginTop: 12}}>
                        <Space>
                            <Typography.Text type="secondary" style={{fontSize: 12}}>Emergency</Typography.Text>
                            <Tag color={emergency.active_emergency ? "error" : emergency.latched_emergency ? "warning" : "default"}>
                                {emergency.active_emergency
                                    ? (emergency.reason ?? "ACTIVE")
                                    : emergency.latched_emergency
                                        ? "Latched"
                                        : "Clear"}
                            </Tag>
                        </Space>
                    </div>
                    {btNodeStates.size > 0 && (
                        <div style={{marginTop: 12}}>
                            <Typography.Text type="secondary" style={{fontSize: 12, display: "block", marginBottom: 4}}>Active BT Nodes</Typography.Text>
                            <Flex wrap gap={4}>
                                {Array.from(btNodeStates.entries())
                                    .filter(([, status]) => status === "RUNNING" || status === "SUCCESS")
                                    .map(([name, status]) => (
                                        <Tag
                                            key={name}
                                            color={status === "RUNNING" ? "processing" : status === "SUCCESS" ? "success" : "default"}
                                            style={{fontSize: 11}}
                                        >
                                            {name}
                                        </Tag>
                                    ))}
                            </Flex>
                        </div>
                    )}
                </Card>
            </Col>
            <Col xs={24} lg={12}>
                <Card title="Coverage" size="small">
                    <Table
                        size="small"
                        dataSource={snapshot?.coverage ?? []}
                        columns={coverageColumns}
                        rowKey="area_index"
                        pagination={false}
                        locale={{emptyText: "No coverage data"}}
                    />
                </Card>
            </Col>
        </Row>
    );

    // ── Section 3b: Configuration Cross-checks ──────────────────────────────
    // Note: SLAM (Cartographer) was removed on the feat/kiss-icp branch. The
    // occupancy grid is now published by map_server_node from recorded area
    // polygons, so there is no pbstream to save/delete.

    const crossChecks = snapshot?.cross_checks;
    const crossCheckStatus = crossChecks?.overall_status ?? "ok";

    const sectionCrossChecks = (
        <Row gutter={[12, 12]}>
            <Col xs={24}>
                <Card
                    title="Configuration Cross-checks"
                    size="small"
                    extra={
                        <Tag color={
                            crossCheckStatus === "ok" ? "success" :
                            crossCheckStatus === "warn" ? "warning" : "error"
                        }>
                            {crossCheckStatus.toUpperCase()}
                        </Tag>
                    }
                >
                    {crossChecks?.warnings && crossChecks.warnings.length > 0 ? (
                        <Space direction="vertical" style={{width: "100%", marginBottom: 12}}>
                            {crossChecks.warnings.map((w, i) => (
                                <Alert key={i} type="warning" message={w} showIcon style={{fontSize: 12}}/>
                            ))}
                        </Space>
                    ) : (
                        <Typography.Text type="secondary" style={{fontSize: 12, display: "block", marginBottom: 12}}>
                            No warnings.
                        </Typography.Text>
                    )}
                    {crossChecks?.dock_pose && (
                        <Row gutter={[8, 4]}>
                            <Col span={24}>
                                <Typography.Text type="secondary" style={{fontSize: 11}}>Dock pose</Typography.Text>
                            </Col>
                            <Col span={8}>
                                <Statistic
                                    title="X (m)"
                                    value={crossChecks.dock_pose.configured_x}
                                    precision={3}
                                />
                            </Col>
                            <Col span={8}>
                                <Statistic
                                    title="Y (m)"
                                    value={crossChecks.dock_pose.configured_y}
                                    precision={3}
                                />
                            </Col>
                            <Col span={8}>
                                <Statistic
                                    title="Yaw (deg)"
                                    value={(crossChecks.dock_pose.configured_yaw * 180 / Math.PI).toFixed(1)}
                                    suffix="°"
                                />
                            </Col>
                            <Col span={12}>
                                <Statistic
                                    title="Datum lat"
                                    value={crossChecks.dock_pose.datum_lat}
                                    precision={7}
                                />
                            </Col>
                            <Col span={12}>
                                <Statistic
                                    title="Datum lon"
                                    value={crossChecks.dock_pose.datum_lon}
                                    precision={7}
                                />
                            </Col>
                            <Col span={24}>
                                <Space>
                                    <Typography.Text type="secondary" style={{fontSize: 12}}>Config present</Typography.Text>
                                    <Tag color={crossChecks.dock_pose.has_config ? "success" : "warning"}>
                                        {crossChecks.dock_pose.has_config ? "Yes" : "No"}
                                    </Tag>
                                </Space>
                            </Col>
                        </Row>
                    )}
                </Card>
            </Col>
        </Row>
    );

    // ── Section 4: Sensors ───────────────────────────────────────────────────

    const sectionSensors = (
        <Row gutter={[12, 12]}>
            <Col xs={24} lg={12}>
                <Card title="IMU" size="small">
                    <Row gutter={[12, 8]}>
                        <Col span={8}>
                            <Statistic title="Ang Vel X" value={imu.angular_velocity?.x} precision={4}/>
                        </Col>
                        <Col span={8}>
                            <Statistic title="Ang Vel Y" value={imu.angular_velocity?.y} precision={4}/>
                        </Col>
                        <Col span={8}>
                            <Statistic title="Ang Vel Z" value={imu.angular_velocity?.z} precision={4}/>
                        </Col>
                        <Col span={8}>
                            <Statistic title="Lin Acc X" value={imu.linear_acceleration?.x} precision={4}/>
                        </Col>
                        <Col span={8}>
                            <Statistic title="Lin Acc Y" value={imu.linear_acceleration?.y} precision={4}/>
                        </Col>
                        <Col span={8}>
                            <Statistic title="Lin Acc Z" value={imu.linear_acceleration?.z} precision={4}/>
                        </Col>
                    </Row>
                </Card>
            </Col>
            <Col xs={24} lg={12}>
                <Card title="Wheel Ticks" size="small">
                    <Row gutter={[12, 8]}>
                        <Col span={12}>
                            <Statistic title="Rear Left" value={wheelTicks.wheel_ticks_rl}/>
                        </Col>
                        <Col span={12}>
                            <Statistic title="Rear Right" value={wheelTicks.wheel_ticks_rr}/>
                        </Col>
                        <Col span={12}>
                            <Statistic title="RL Direction" value={wheelTicks.wheel_direction_rl}/>
                        </Col>
                        <Col span={12}>
                            <Statistic title="RR Direction" value={wheelTicks.wheel_direction_rr}/>
                        </Col>
                    </Row>
                </Card>
            </Col>
            <Col span={24}>
                <Card title={<Space><SoundOutlined/> Hardware Status</Space>} size="small">
                    <Row gutter={[12, 8]}>
                        <Col xs={12} lg={4}>
                            <Statistic
                                title="Mower Status"
                                value={status.mower_status === 255 ? "OK" : "Initializing"}
                                valueStyle={{color: status.mower_status === 255 ? undefined : colors.warning}}
                            />
                        </Col>
                        <Col xs={12} lg={4}>
                            <Statistic
                                title="Rain"
                                value={status.rain_detected ? "Detected" : "None"}
                                valueStyle={{color: status.rain_detected ? colors.warning : undefined}}
                            />
                        </Col>
                        <Col xs={12} lg={4}>
                            <Statistic
                                title="ESC Status"
                                value={(ESC_STATUS[status.mower_esc_status ?? 0] ?? {label: `Unknown (${status.mower_esc_status})`}).label}
                                valueStyle={{
                                    color: ESC_STATUS[status.mower_esc_status ?? 0]?.color === "error" ? colors.danger
                                        : ESC_STATUS[status.mower_esc_status ?? 0]?.color === "warning" ? colors.warning
                                        : undefined,
                                }}
                            />
                        </Col>
                        <Col xs={12} lg={4}>
                            <Statistic title="ESC Temp" value={status.mower_esc_temperature} precision={1} suffix="°C"/>
                        </Col>
                        <Col xs={12} lg={4}>
                            <Statistic title="Motor Temp" value={status.mower_motor_temperature} precision={1} suffix="°C"/>
                        </Col>
                        <Col xs={12} lg={4}>
                            <Statistic title="Motor RPM" value={status.mower_motor_rpm} precision={0}/>
                        </Col>
                    </Row>
                    <Flex wrap gap="small" style={{marginTop: 12}}>
                        <Tag color={status.raspberry_pi_power ? "success" : "default"}>RPi Power</Tag>
                        <Tag color={status.esc_power ? "success" : "default"}>ESC Power</Tag>
                        <Tag color={status.ui_board_available ? "success" : "default"}>UI Board</Tag>
                        <Tag color={status.sound_module_available ? "success" : "default"}>Sound Module</Tag>
                        <Tag color={status.mow_enabled ? "success" : "default"}>Mow Enabled</Tag>
                    </Flex>
                </Card>
            </Col>
        </Row>
    );

    // ── Section 5: ROS Diagnostics ───────────────────────────────────────────

    const sectionRosDiagnostics = (
        <Card title="ROS Diagnostics" size="small">
            {(diagnostics.status ?? []).length === 0 ? (
                <Typography.Text type="secondary">No diagnostic messages received.</Typography.Text>
            ) : (
                <Collapse
                    size="small"
                    ghost
                    items={(diagnostics.status ?? []).map((item, idx) => ({
                        key: idx,
                        label: (
                            <Space>
                                <Tag color={DIAG_LEVEL_COLORS[item.level] ?? "default"}>
                                    {DIAG_LEVEL_LABELS[item.level] ?? String(item.level)}
                                </Tag>
                                <Typography.Text style={{fontSize: 13}}>{item.name}</Typography.Text>
                                <Typography.Text type="secondary" style={{fontSize: 12}}>{item.message}</Typography.Text>
                            </Space>
                        ),
                        children: item.values && item.values.length > 0 ? (
                            <div style={{paddingLeft: 8}}>
                                {item.values.map((kv, i) => (
                                    <div key={i} style={{display: "flex", gap: 8, fontSize: 12, marginBottom: 2}}>
                                        <Typography.Text type="secondary">{kv.key}:</Typography.Text>
                                        <Typography.Text code style={{fontSize: 11}}>{kv.value}</Typography.Text>
                                    </div>
                                ))}
                            </div>
                        ) : (
                            <Typography.Text type="secondary" style={{fontSize: 12}}>No key-value pairs.</Typography.Text>
                        ),
                    }))}
                />
            )}
        </Card>
    );

    // ── Section 6: Alerts ────────────────────────────────────────────────────

    const sectionAlerts = alerts.length > 0 ? (
        <Card title={<Space><WarningOutlined/> Alerts</Space>} size="small">
            <Space direction="vertical" style={{width: "100%"}}>
                {alerts.map((item, idx) => (
                    <Alert
                        key={idx}
                        type={item.level === 2 ? "error" : item.level === 3 ? "info" : "warning"}
                        message={item.name}
                        description={item.message}
                        showIcon
                    />
                ))}
            </Space>
        </Card>
    ) : null;

    // ── layout ───────────────────────────────────────────────────────────────

    if (isMobile) {
        return (
            <div style={{display: "flex", flexDirection: "column", gap: 12, paddingBottom: 8}}>
                {healthBar}
                {sectionAlerts}
                <Collapse
                    defaultActiveKey={[]}
                    size="small"
                    items={[
                        {
                            key: "system",
                            label: <Space><CloudServerOutlined/> System</Space>,
                            children: sectionSystem,
                        },
                        {
                            key: "localization",
                            label: <Space><CompassOutlined/> Localization</Space>,
                            children: sectionLocalization,
                        },
                        {
                            key: "heading_sources",
                            label: <Space><CompassOutlined/> Heading sources</Space>,
                            children: sectionHeadingSources,
                        },
                        {
                            key: "bt",
                            label: <Space><ApiOutlined/> BT State & Coverage</Space>,
                            children: sectionBtCoverage,
                        },
                        {
                            key: "cross_checks",
                            label: "Configuration Cross-checks",
                            children: sectionCrossChecks,
                        },
                        {
                            key: "sensors",
                            label: <Space><ThunderboltOutlined/> Sensors</Space>,
                            children: sectionSensors,
                        },
                        {
                            key: "ros",
                            label: "ROS Diagnostics",
                            children: sectionRosDiagnostics,
                        },
                    ]}
                />
            </div>
        );
    }

    return (
        <Row gutter={[16, 16]}>
            <Col span={24}>{healthBar}</Col>
            {sectionAlerts && <Col span={24}>{sectionAlerts}</Col>}
            <Col span={24}>{sectionSystem}</Col>
            <Col span={24}>{sectionLocalization}</Col>
            <Col span={24}>{sectionHeadingSources}</Col>
            <Col span={24}>{sectionBtCoverage}</Col>
            <Col span={24}>{sectionCrossChecks}</Col>
            <Col span={24}>{sectionSensors}</Col>
            <Col span={24}>{sectionRosDiagnostics}</Col>
        </Row>
    );
};

export default DiagnosticsPage;
