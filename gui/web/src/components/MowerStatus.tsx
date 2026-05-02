import {useHighLevelStatus} from "../hooks/useHighLevelStatus.ts";
import {useStatus} from "../hooks/useStatus.ts";
import {useEmergency} from "../hooks/useEmergency.ts";
import {usePower} from "../hooks/usePower.ts";
import {useGPS} from "../hooks/useGPS.ts";
import {useSettings} from "../hooks/useSettings.ts";
import {AbsolutePoseConstants} from "../types/ros.ts";
import {computeBatteryPercent} from "../utils/battery.ts";
import {restartMowgliNext} from "../utils/containers.ts";
import {useContainerRestart} from "../hooks/useContainerRestart.ts";
import {App, Badge, Dropdown, Modal, Space, Typography} from "antd";
import {PoweroffOutlined, ReloadOutlined, DesktopOutlined, WifiOutlined} from "@ant-design/icons"
import {stateRenderer} from "./utils.tsx";
import {useThemeMode} from "../theme/ThemeContext.tsx";
import {useApi} from "../hooks/useApi.ts";
import type {MenuProps} from "antd";

const pulseKeyframes = `
@keyframes mowerPulseGreen {
    0%, 100% { box-shadow: 0 0 0 0 rgba(82, 196, 26, 0.6); }
    50% { box-shadow: 0 0 0 4px rgba(82, 196, 26, 0); }
}
@keyframes mowerPulseRed {
    0%, 100% { box-shadow: 0 0 0 0 rgba(255, 77, 79, 0.6); }
    50% { box-shadow: 0 0 0 4px rgba(255, 77, 79, 0); }
}
`;

// Motion states → primary (active); resting states → warning;
// everything else (emergencies, unknown) → danger.
const MOTION_STATES = new Set([
    "MOWING", "TRANSIT", "UNDOCKING", "RETURNING_HOME", "MANUAL_MOWING",
    "RESUMING_AFTER_RAIN", "RESUMING_UNDOCKING", "BOUNDARY_RECOVERY",
    "LOW_BATTERY_DOCKING", "CRITICAL_BATTERY_DOCKING", "RAIN_DETECTED_DOCKING",
    "COVERAGE_FAILED_DOCKING", "SKIP_STRIP", "PREFLIGHT_CHECK",
    "CALIBRATING_HEADING", "RECORDING",
]);
const RESTING_STATES = new Set([
    "IDLE", "IDLE_DOCKED", "CHARGING", "MOWING_COMPLETE",
    "RECORDING_COMPLETE", "RAIN_WAITING", "RAIN_TIMEOUT",
]);

const statusColor = (state: string | undefined, colors: {primary: string; warning: string; danger: string}): string => {
    if (!state) return colors.danger;
    if (MOTION_STATES.has(state)) return colors.primary;
    if (RESTING_STATES.has(state)) return colors.warning;
    return colors.danger;
};

export const MowerStatus = () => {
    const {colors} = useThemeMode();
    const {highLevelStatus} = useHighLevelStatus();
    const hwStatus = useStatus();
    const emergencyData = useEmergency();
    const power = usePower();
    const gps = useGPS();
    const {settings} = useSettings();
    const guiApi = useApi();
    const {notification} = App.useApp();

    // Derive state with fallbacks
    const isEmergency = highLevelStatus.emergency ?? emergencyData.active_emergency ?? false;
    const isCharging = highLevelStatus.is_charging ?? hwStatus.is_charging ?? false;

    const stateName = highLevelStatus.state_name ?? (
        isEmergency ? "EMERGENCY" :
        isCharging ? "CHARGING" :
        hwStatus.mower_status != null ? "IDLE" :
        undefined
    );

    // GPS quality with fallback
    const gpsPercent = (() => {
        if (highLevelStatus.gps_quality_percent != null && highLevelStatus.gps_quality_percent > 0) {
            return Math.round(highLevelStatus.gps_quality_percent * 100);
        }
        if (gps.flags != null) {
            if (gps.flags & AbsolutePoseConstants.FLAG_GPS_RTK_FIXED) return 100;
            if (gps.flags & AbsolutePoseConstants.FLAG_GPS_RTK_FLOAT) return 50;
            if (gps.flags & AbsolutePoseConstants.FLAG_GPS_RTK) return 25;
        }
        return 0;
    })();

    const batteryPercent = computeBatteryPercent(
        highLevelStatus.battery_percent, power.v_battery, settings,
    );

    const isMowing = stateName === "MOWING" || stateName === "TRANSIT" ||
        stateName === "UNDOCKING" || stateName === "RETURNING_HOME" ||
        stateName === "MANUAL_MOWING";

    const pulseAnimation = isEmergency
        ? 'mowerPulseRed 1.5s ease-in-out infinite'
        : isMowing
            ? 'mowerPulseGreen 2s ease-in-out infinite'
            : 'none';

    const hasArea = highLevelStatus.current_area !== undefined && highLevelStatus.current_area >= 0;
    const hasProgress = isMowing && highLevelStatus.current_path_index !== undefined && highLevelStatus.current_path !== undefined && highLevelStatus.current_path > 0;
    const progressPercent = hasProgress
        ? Math.round(((highLevelStatus.current_path_index ?? 0) / (highLevelStatus.current_path ?? 1)) * 100)
        : null;

    // Long-running: container restart + rosbridge reconnect. Lock the menu
    // item until ROS2 is reachable again to prevent duplicate-click storms.
    const mowgliRestart = useContainerRestart({
        pendingLabel: "Redémarrage Mowgli…",
        successMessage: "Mowgli redémarré",
        errorMessage: "Échec du redémarrage Mowgli",
    });
    const restartMowgli = () => mowgliRestart.run(() => restartMowgliNext(guiApi));

    const rebootSystem = async () => {
        try {
            await guiApi.request({path: "/system/reboot", method: "POST"});
            notification.success({message: "Rebooting..."});
        } catch (e: any) {
            notification.error({message: "Failed to reboot", description: e.message});
        }
    };

    const shutdownSystem = async () => {
        try {
            await guiApi.request({path: "/system/shutdown", method: "POST"});
            notification.success({message: "Shutting down..."});
        } catch (e: any) {
            notification.error({message: "Failed to shutdown", description: e.message});
        }
    };

    const confirmAction = (title: string, content: string, onOk: () => Promise<void>) => {
        Modal.confirm({
            title,
            content,
            okText: "Confirm",
            okType: "danger",
            cancelText: "Cancel",
            onOk,
        });
    };

    const powerMenuItems: MenuProps["items"] = [
        {
            key: "restart-mowgli",
            icon: <ReloadOutlined/>,
            label: mowgliRestart.pending ? mowgliRestart.pendingLabel : "Restart Mowgli",
            disabled: mowgliRestart.pending,
            onClick: () => confirmAction("Restart Mowgli", "This will restart the MowgliNext container.", restartMowgli),
        },
        {type: "divider"},
        {
            key: "reboot",
            icon: <DesktopOutlined/>,
            label: "Reboot Raspberry Pi",
            onClick: () => confirmAction("Reboot Raspberry Pi", "The system will reboot. You will lose connection temporarily.", rebootSystem),
        },
        {
            key: "shutdown",
            icon: <PoweroffOutlined/>,
            label: "Shutdown Raspberry Pi",
            danger: true,
            onClick: () => confirmAction("Shutdown Raspberry Pi", "The system will shut down. You will need physical access to turn it back on.", shutdownSystem),
        },
    ];

    return (
        <>
            <style>{pulseKeyframes}</style>
            <Space size="small" style={{flexShrink: 0}}>
                <Space size={4}>
                    <Badge
                        color={statusColor(stateName, colors)}
                        style={{animation: pulseAnimation, borderRadius: '50%'}}
                    />
                    <Typography.Text style={{fontSize: 12, color: colors.text, whiteSpace: 'nowrap'}}>
                        {stateRenderer(stateName)}
                    </Typography.Text>
                </Space>
                {isMowing && hasArea && (
                    <Typography.Text style={{fontSize: 11, color: colors.primary, whiteSpace: 'nowrap'}}>
                        A{(highLevelStatus.current_area ?? 0) + 1}
                        {progressPercent !== null ? ` ${progressPercent}%` : ''}
                    </Typography.Text>
                )}
                <Space size={4}>
                    <WifiOutlined style={{color: gpsPercent > 0 ? colors.primary : colors.danger, fontSize: 13}}/>
                    <Typography.Text style={{fontSize: 12, color: colors.text}}>
                        {gpsPercent}%
                    </Typography.Text>
                </Space>
                <Dropdown menu={{items: powerMenuItems}} trigger={["click"]} placement="bottomRight">
                    <Space size={4} style={{cursor: "pointer"}}>
                        <PoweroffOutlined style={{
                            color: isCharging ? colors.primary : colors.muted,
                            fontSize: 13,
                        }}/>
                        <Typography.Text style={{fontSize: 12, color: colors.text}}>
                            {batteryPercent}%
                        </Typography.Text>
                    </Space>
                </Dropdown>
            </Space>
        </>
    );
}
