import {Card, Col, Row, Statistic, Tag, Space, Flex} from "antd";
import {
    ThunderboltOutlined,
    WarningOutlined,
    ApiOutlined,
    SoundOutlined,
    DashboardOutlined,
} from "@ant-design/icons";
import {useStatus} from "../hooks/useStatus.ts";
import {usePower} from "../hooks/usePower.ts";
import {useEmergency} from "../hooks/useEmergency.ts";
import {useDockingSensor} from "../hooks/useDockingSensor.ts";
import {useThemeMode} from "../theme/ThemeContext.tsx";

const StatusTag = ({label, active, color}: { label: string; active: boolean; color?: string }) => (
    <Tag color={active ? (color ?? "green") : "default"}>{label}</Tag>
);

/**
 * Derive a single charging label from power + status data.
 * Avoids showing 3 separate badges for the same info.
 */
function chargingLabel(chargerEnabled: boolean, isCharging: boolean): { label: string; active: boolean; color: string } {
    if (isCharging) {
        return {label: "Charging", active: true, color: "cyan"};
    }
    if (chargerEnabled) {
        return {label: "Charger On", active: true, color: "green"};
    }
    return {label: "Not Charging", active: false, color: "default"};
}

/**
 * Derive a single emergency label from emergency data.
 */
function emergencyLabel(active: boolean, latched: boolean, reason?: string): { label: string; color: string } {
    if (active) {
        return {label: reason ? `Emergency: ${reason}` : "EMERGENCY", color: "red"};
    }
    if (latched) {
        return {label: reason ?? "Latched (press play to release)", color: "orange"};
    }
    return {label: "Clear", color: "default"};
}

export function StatusComponent({compact}: {compact?: boolean}) {
    const {colors} = useThemeMode();
    const status = useStatus();
    const power = usePower();
    const emergency = useEmergency();
    const dockingSensor = useDockingSensor();

    const mowerStatusLabel = status.mower_status === 255 ? "OK" : "Initializing";
    const charging = chargingLabel(!!power.charger_enabled, !!status.is_charging);
    const emg = emergencyLabel(!!emergency.active_emergency, !!emergency.latched_emergency, emergency.reason);

    if (compact) {
        return (
            <div style={{display: 'flex', flexDirection: 'column', gap: 12}}>
                {/* Combined: System Status + Emergency + Docking */}
                <div style={{
                    background: colors.bgCard,
                    borderRadius: 12,
                    padding: 16,
                }}>
                    <Space style={{marginBottom: 8}}>
                        <ApiOutlined style={{color: colors.textSecondary}}/>
                        <span style={{color: colors.textSecondary, fontSize: 13, fontWeight: 500}}>System & Safety</span>
                    </Space>
                    <Flex wrap gap="small" style={{marginBottom: 8}}>
                        <StatusTag label={`Mower: ${mowerStatusLabel}`} active={status.mower_status === 255}/>
                        <StatusTag label="RPi Power" active={!!status.raspberry_pi_power}/>
                        <StatusTag label="Mow Enabled" active={!!status.mow_enabled}/>
                        <StatusTag label={status.rain_detected ? "Rain" : "No Rain"}
                                   active={!!status.rain_detected} color="blue"/>
                    </Flex>
                    <div style={{borderTop: `1px solid ${colors.borderSubtle}`, paddingTop: 8, marginBottom: 8}}>
                        <Flex wrap gap="small">
                            <Tag color={emg.color}>{emg.label}</Tag>
                            <StatusTag label={dockingSensor.dock_present ? "Dock: Present" : "Dock: Away"}
                                       active={!!dockingSensor.dock_present} color="cyan"/>
                        </Flex>
                    </div>
                </div>

                {/* Combined: Power + Motor */}
                <div style={{
                    background: colors.bgCard,
                    borderRadius: 12,
                    padding: 16,
                }}>
                    <Space style={{marginBottom: 8}}>
                        <ThunderboltOutlined style={{color: colors.textSecondary}}/>
                        <span style={{color: colors.textSecondary, fontSize: 13, fontWeight: 500}}>Power & Motor</span>
                    </Space>
                    <Row gutter={[12, 8]}>
                        <Col span={8}>
                            <Statistic title="Battery" value={power.v_battery} precision={1} suffix="V"
                                       valueStyle={{fontSize: 16}}/>
                        </Col>
                        <Col span={8}>
                            <Statistic title="Charge" value={power.v_charge} precision={1} suffix="V"
                                       valueStyle={{fontSize: 16}}/>
                        </Col>
                        <Col span={8}>
                            <Statistic title="Current" value={power.charge_current} precision={1} suffix="A"
                                       valueStyle={{fontSize: 16}}/>
                        </Col>
                    </Row>
                    <div style={{borderTop: `1px solid ${colors.borderSubtle}`, paddingTop: 8, marginTop: 8}}>
                        <Row gutter={[12, 8]}>
                            <Col span={8}>
                                <Statistic title="RPM" value={status.mower_motor_rpm} precision={0}
                                           valueStyle={{fontSize: 16}}/>
                            </Col>
                            <Col span={8}>
                                <Statistic title="Motor" value={status.mower_motor_temperature} precision={0} suffix="°C"
                                           valueStyle={{fontSize: 16}}/>
                            </Col>
                            <Col span={8}>
                                <Statistic title="ESC" value={status.mower_esc_temperature} precision={0} suffix="°C"
                                           valueStyle={{fontSize: 16}}/>
                            </Col>
                        </Row>
                    </div>
                </div>
            </div>
        );
    }

    return <Row gutter={[16, 16]}>
        {/* System Status */}
        <Col lg={12} xs={24}>
            <Card title={<Space><ApiOutlined/> System Status</Space>} size="small">
                <Flex wrap gap="small" style={{marginBottom: 16}}>
                    <StatusTag label={`Mower: ${mowerStatusLabel}`} active={status.mower_status === 255}/>
                    <StatusTag label="RPi Power" active={!!status.raspberry_pi_power}/>
                    <StatusTag label="ESC Power" active={!!status.esc_power}/>
                    <StatusTag label="UI Board" active={!!status.ui_board_available}/>
                    <StatusTag label="Mow Enabled" active={!!status.mow_enabled}/>
                </Flex>
                <Flex wrap gap="small">
                    <StatusTag label="Sound Module" active={!!status.sound_module_available}/>
                    <StatusTag label={status.sound_module_busy ? "Sound: Busy" : "Sound: Idle"}
                               active={!!status.sound_module_busy} color="orange"/>
                    <StatusTag label={status.rain_detected ? "Rain Detected" : "No Rain"}
                               active={!!status.rain_detected} color="blue"/>
                </Flex>
            </Card>
        </Col>

        {/* Power */}
        <Col lg={12} xs={24}>
            <Card title={<Space><ThunderboltOutlined/> Power</Space>} size="small">
                <Row gutter={[16, 16]}>
                    <Col span={8}>
                        <Statistic title="Battery" value={power.v_battery} precision={2} suffix="V"/>
                    </Col>
                    <Col span={8}>
                        <Statistic title="Charge" value={power.v_charge} precision={2} suffix="V"/>
                    </Col>
                    <Col span={8}>
                        <Statistic title="Current" value={power.charge_current} precision={2} suffix="A"/>
                    </Col>
                </Row>
                <Flex wrap gap="small" style={{marginTop: 12}}>
                    <Tag color={charging.color}>{charging.label}</Tag>
                </Flex>
            </Card>
        </Col>

        {/* Emergency */}
        <Col lg={12} xs={24}>
            <Card title={<Space><WarningOutlined/> Emergency</Space>} size="small"
                  styles={{body: {paddingBottom: 12}}}>
                <Flex wrap gap="small" align="center">
                    <Tag color={emg.color}>{emg.label}</Tag>
                </Flex>
            </Card>
        </Col>

        {/* Docking Sensor */}
        <Col lg={12} xs={24}>
            <Card title={<Space><DashboardOutlined/> Docking Sensor</Space>} size="small"
                  styles={{body: {paddingBottom: 12}}}>
                <Flex wrap gap="small">
                    <StatusTag label={dockingSensor.dock_present ? "Present" : "Away"}
                               active={!!dockingSensor.dock_present} color="cyan"/>
                    <StatusTag label={`Distance: ${dockingSensor.dock_distance?.toFixed(2) ?? "-"} m`}
                               active={(dockingSensor.dock_distance ?? 0) > 0} color="cyan"/>
                </Flex>
            </Card>
        </Col>

        {/* Mower Motor */}
        <Col span={24}>
            <Card title={<Space><SoundOutlined/> Mower Motor</Space>} size="small">
                <Row gutter={[16, 16]}>
                    <Col lg={4} xs={8}>
                        <Statistic title="ESC Status" value={status.mower_esc_status}/>
                    </Col>
                    <Col lg={5} xs={12}>
                        <Statistic title="RPM" value={status.mower_motor_rpm} precision={0}/>
                    </Col>
                    <Col lg={5} xs={12}>
                        <Statistic title="Current" value={status.mower_esc_current} precision={2} suffix="A"/>
                    </Col>
                    <Col lg={5} xs={12}>
                        <Statistic title="ESC Temp" value={status.mower_esc_temperature} precision={1} suffix="°C"/>
                    </Col>
                    <Col lg={5} xs={12}>
                        <Statistic title="Motor Temp" value={status.mower_motor_temperature} precision={1}
                                   suffix="°C"/>
                    </Col>
                </Row>
            </Card>
        </Col>
    </Row>;
}
