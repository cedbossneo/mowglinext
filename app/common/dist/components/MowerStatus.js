import { jsx as _jsx, jsxs as _jsxs, Fragment as _Fragment } from "react/jsx-runtime";
import { useHighLevelStatus } from "../hooks/useHighLevelStatus";
import { useStatus } from "../hooks/useStatus";
import { useEmergency } from "../hooks/useEmergency";
import { usePower } from "../hooks/usePower";
import { useGPS } from "../hooks/useGPS";
import { AbsolutePoseFlags } from "../types/ros";
import { App, Badge, Dropdown, Modal, Space, Typography } from "antd";
import { PoweroffOutlined, ReloadOutlined, DesktopOutlined, WifiOutlined } from "@ant-design/icons";
import { stateRenderer } from "./utils";
import { useThemeMode } from "../theme/ThemeContext";
import { useApi } from "../hooks/useApi";
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
const statusColor = (state, colors) => {
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
/**
 * MowerStatus component.
 *
 * Note: The original web version uses `useSettings()` for battery voltage thresholds.
 * This common version accepts optional settings as a prop to avoid pulling in the
 * web-specific useSettings/useConfig hooks. If not provided, defaults are used.
 */
export const MowerStatus = (props) => {
    const { settings = {} } = props;
    const { colors } = useThemeMode();
    const { highLevelStatus } = useHighLevelStatus();
    const hwStatus = useStatus();
    const emergencyData = useEmergency();
    const power = usePower();
    const gps = useGPS();
    const guiApi = useApi();
    const { notification } = App.useApp();
    // Derive state with fallbacks
    const isEmergency = highLevelStatus.Emergency ?? emergencyData.ActiveEmergency ?? false;
    const isCharging = highLevelStatus.IsCharging ?? hwStatus.IsCharging ?? false;
    const stateName = highLevelStatus.StateName ?? (isEmergency ? "EMERGENCY" :
        isCharging ? "CHARGING" :
            hwStatus.MowerStatus != null ? "IDLE" :
                undefined);
    // GPS quality with fallback
    const gpsPercent = (() => {
        if (highLevelStatus.GpsQualityPercent != null && highLevelStatus.GpsQualityPercent > 0) {
            return Math.round(highLevelStatus.GpsQualityPercent * 100);
        }
        if (gps.Flags != null) {
            if (gps.Flags & AbsolutePoseFlags.FIXED)
                return 100;
            if (gps.Flags & AbsolutePoseFlags.FLOAT)
                return 50;
            if (gps.Flags & AbsolutePoseFlags.RTK)
                return 25;
        }
        return 0;
    })();
    // Battery percent with fallback
    const batteryPercent = (() => {
        if (highLevelStatus.BatteryPercent != null && highLevelStatus.BatteryPercent > 0) {
            return Math.round(highLevelStatus.BatteryPercent * 100);
        }
        if (power.VBattery) {
            const full = parseFloat(settings["battery_full_voltage"] ?? "28.5");
            const empty = parseFloat(settings["battery_empty_voltage"] ?? "23.0");
            const pct = ((power.VBattery - empty) / (full - empty)) * 100;
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
    const hasArea = highLevelStatus.CurrentArea !== undefined && highLevelStatus.CurrentArea >= 0;
    const hasProgress = isMowing && highLevelStatus.CurrentPathIndex !== undefined && highLevelStatus.CurrentPath !== undefined && highLevelStatus.CurrentPath > 0;
    const progressPercent = hasProgress
        ? Math.round(((highLevelStatus.CurrentPathIndex ?? 0) / (highLevelStatus.CurrentPath ?? 1)) * 100)
        : null;
    const restartMowgli = async () => {
        try {
            const res = await guiApi.containers.containersList();
            if (res.error)
                throw new Error(res.error.error);
            const container = res.data.containers?.find((c) => c.labels?.app === "openmower" || c.names?.includes("/openmower"));
            if (container?.id) {
                const cmdRes = await guiApi.containers.containersCreate(container.id, "restart");
                if (cmdRes.error)
                    throw new Error(cmdRes.error.error);
                notification.success({ message: "Mowgli restarted" });
            }
            else {
                throw new Error("OpenMower container not found");
            }
        }
        catch (e) {
            notification.error({ message: "Failed to restart Mowgli", description: e.message });
        }
    };
    const rebootSystem = async () => {
        try {
            await guiApi.request({ path: "/system/reboot", method: "POST" });
            notification.success({ message: "Rebooting..." });
        }
        catch (e) {
            notification.error({ message: "Failed to reboot", description: e.message });
        }
    };
    const shutdownSystem = async () => {
        try {
            await guiApi.request({ path: "/system/shutdown", method: "POST" });
            notification.success({ message: "Shutting down..." });
        }
        catch (e) {
            notification.error({ message: "Failed to shutdown", description: e.message });
        }
    };
    const confirmAction = (title, content, onOk) => {
        Modal.confirm({
            title,
            content,
            okText: "Confirm",
            okType: "danger",
            cancelText: "Cancel",
            onOk,
        });
    };
    const powerMenuItems = [
        {
            key: "restart-mowgli",
            icon: _jsx(ReloadOutlined, {}),
            label: "Restart Mowgli",
            onClick: () => confirmAction("Restart Mowgli", "This will restart the OpenMower container.", restartMowgli),
        },
        { type: "divider" },
        {
            key: "reboot",
            icon: _jsx(DesktopOutlined, {}),
            label: "Reboot Raspberry Pi",
            onClick: () => confirmAction("Reboot Raspberry Pi", "The system will reboot. You will lose connection temporarily.", rebootSystem),
        },
        {
            key: "shutdown",
            icon: _jsx(PoweroffOutlined, {}),
            label: "Shutdown Raspberry Pi",
            danger: true,
            onClick: () => confirmAction("Shutdown Raspberry Pi", "The system will shut down. You will need physical access to turn it back on.", shutdownSystem),
        },
    ];
    return (_jsxs(_Fragment, { children: [_jsx("style", { children: pulseKeyframes }), _jsxs(Space, { size: "small", style: { flexShrink: 0 }, children: [_jsxs(Space, { size: 4, children: [_jsx(Badge, { color: statusColor(stateName, colors), style: { animation: pulseAnimation, borderRadius: '50%' } }), _jsx(Typography.Text, { style: { fontSize: 12, color: colors.text, whiteSpace: 'nowrap' }, children: stateRenderer(stateName) })] }), isMowing && hasArea && (_jsxs(Typography.Text, { style: { fontSize: 11, color: colors.primary, whiteSpace: 'nowrap' }, children: ["A", (highLevelStatus.CurrentArea ?? 0) + 1, progressPercent !== null ? ` ${progressPercent}%` : ''] })), _jsxs(Space, { size: 4, children: [_jsx(WifiOutlined, { style: { color: gpsPercent > 0 ? colors.primary : colors.danger, fontSize: 13 } }), _jsxs(Typography.Text, { style: { fontSize: 12, color: colors.text }, children: [gpsPercent, "%"] })] }), _jsx(Dropdown, { menu: { items: powerMenuItems }, trigger: ["click"], placement: "bottomRight", children: _jsxs(Space, { size: 4, style: { cursor: "pointer" }, children: [_jsx(PoweroffOutlined, { style: {
                                        color: isCharging ? colors.primary : colors.muted,
                                        fontSize: 13,
                                    } }), _jsxs(Typography.Text, { style: { fontSize: 12, color: colors.text }, children: [batteryPercent, "%"] })] }) })] })] }));
};
