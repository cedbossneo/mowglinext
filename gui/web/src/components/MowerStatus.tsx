import {useHighLevelStatus} from "../hooks/useHighLevelStatus.ts";
import {useStatus} from "../hooks/useStatus.ts";
import {useEmergency} from "../hooks/useEmergency.ts";
import {usePower} from "../hooks/usePower.ts";
import {useGPS} from "../hooks/useGPS.ts";
import {useSettings} from "../hooks/useSettings.ts";
import {AbsolutePoseConstants} from "../types/ros.ts";
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

const statusColor = (state: string | undefined, colors: {primary: string; warning: string; danger: string}): string => {
    switch (state) {
        case "MOWING":
        case "DOCKING":
        case "UNDOCKING":
            return colors.primary;
        case "IDLE":
        case "CHARGING":
            return colors.warning;
        default:
            return colors.danger;
    }
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

    // Battery percent with fallback
    const batteryPercent = (() => {
        if (highLevelStatus.battery_percent != null && highLevelStatus.battery_percent > 0) {
            return Math.round(highLevelStatus.battery_percent * 100);
        }
        if (power.v_battery) {
            const full = parseFloat(settings["battery_full_voltage"] ?? "28.5");
            const empty = parseFloat(settings["battery_empty_voltage"] ?? "23.0");
            const pct = ((power.v_battery - empty) / (full - empty)) * 100;
            return Math.round(Math.max(0, Math.min(100, pct)));
        }
        return 0;
    })();

    const isMowing = stateName === "MOWING" || stateName === "DOCKING" || stateName === "UNDOCKING";

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

    const restartMowgli = async () => {
        try {
            const res = await guiApi.containers.containersList();
            if (res.error) throw new Error(res.error.error);
            const container = res.data.containers?.find(
                (c) => c.labels?.app === "mowglinext" || c.names?.includes("/mowglinext")
            );
            if (container?.id) {
                const cmdRes = await guiApi.containers.containersCreate(container.id, "restart");
                if (cmdRes.error) throw new Error(cmdRes.error.error);
                notification.success({message: "Mowgli restarted"});
            } else {
                throw new Error("MowgliNext container not found");
            }
        } catch (e: any) {
            notification.error({message: "Failed to restart Mowgli", description: e.message});
        }
    };

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
            label: "Restart Mowgli",
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
