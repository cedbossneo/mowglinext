import {useHighLevelStatus} from "../hooks/useHighLevelStatus.ts";
import {useStatus} from "../hooks/useStatus.ts";
import {useEmergency} from "../hooks/useEmergency.ts";
import {usePower} from "../hooks/usePower.ts";
import {useGnssStatus} from "../hooks/useGnssStatus.ts";
import {useSettings} from "../hooks/useSettings.ts";
import {computeBatteryPercent} from "../utils/battery.ts";
import {deriveGpsStatus} from "../utils/gpsStatus.ts";
import {restartMowgliStack} from "../utils/containers.ts";
import {useMowerAction} from "./MowerActions.tsx";
import {useState} from "react";
import {App, Badge, Button, Dropdown, Modal, Space, Spin, Tooltip, Typography} from "antd";
import {PoweroffOutlined, ReloadOutlined, DesktopOutlined, WifiOutlined, AlertOutlined} from "@ant-design/icons"
import {stateRenderer} from "./utils.tsx";
import {useThemeMode} from "../theme/ThemeContext.tsx";
import {useApi} from "../hooks/useApi.ts";
import {limeAlpha} from "../theme/colors.ts";
import {SettingOutlined} from "@ant-design/icons";
import type {MenuProps} from "antd";
import {useTranslation} from "react-i18next";

// Builds the badge pulse keyframes from brand tokens. Green pulse derives
// from the lime hero accent; red pulse derives from colors.danger (rose).
// colors.danger is a hex string ("#FF6B7A"); append an 8-bit alpha suffix
// to get the translucent stops without hardcoding the rose RGB here.
const buildPulseKeyframes = (dangerHex: string) => `
@keyframes mowerPulseGreen {
    0%, 100% { box-shadow: 0 0 0 0 ${limeAlpha(0.6)}; }
    50% { box-shadow: 0 0 0 4px ${limeAlpha(0)}; }
}
@keyframes mowerPulseRed {
    0%, 100% { box-shadow: 0 0 0 0 ${dangerHex}99; }
    50% { box-shadow: 0 0 0 4px ${dangerHex}00; }
}
`;

// Colour by the NUMERIC high-level state (status_nodes.cpp publishes it every
// tick): 2=AUTONOMOUS / 3=RECORDING / 4=MANUAL_MOWING are active (primary);
// 1=IDLE is resting (warning); 0=NULL/EMERGENCY or unknown is danger. The old
// string-allowlist approach mis-coloured legitimate state=2 substates
// (PLANNING, OBSTACLE_BACKOFF, DYNAMIC_OBSTACLE_CLEARED, AREA_UNREACHABLE) as
// danger-red because the set never enumerated them.
const statusColor = (stateNum: number | undefined, isEmergency: boolean, colors: {primary: string; warning: string; danger: string}): string => {
    if (isEmergency) return colors.danger;
    if (stateNum === 2 || stateNum === 3 || stateNum === 4) return colors.primary;
    if (stateNum === 1) return colors.warning;
    return colors.danger;
};

export const MowerStatus = () => {
    const {t} = useTranslation();
    const {colors} = useThemeMode();
    const pulseKeyframes = buildPulseKeyframes(colors.danger);
    const {highLevelStatus} = useHighLevelStatus();
    const hwStatus = useStatus();
    const emergencyData = useEmergency();
    const power = usePower();
    const gnss = useGnssStatus();
    const {settings} = useSettings();
    const guiApi = useApi();
    const {notification, modal} = App.useApp();

    // Derive state with fallbacks
    const isEmergency = highLevelStatus.emergency ?? emergencyData.active_emergency ?? false;
    const isCharging = highLevelStatus.is_charging ?? hwStatus.is_charging ?? false;

    const stateName = highLevelStatus.state_name ?? (
        isEmergency ? "EMERGENCY" :
        isCharging ? "CHARGING" :
        hwStatus.mower_status != null ? "IDLE_DOCKED" :
        undefined
    );
    // Numeric high-level state (reliable, published every tick): 2=AUTONOMOUS,
    // 3=RECORDING, 4=MANUAL_MOWING. Drives colour/pulse/mowing so transient
    // state=2 substates (planning, obstacle backoff) never read as idle.
    const stateNum = highLevelStatus.state ?? -1;

    const gpsStatus = deriveGpsStatus(gnss);
    const gpsColor =
        gpsStatus.fixType === "RTK_FIX" ? colors.primary :
        gpsStatus.fixType === "RTK_FLOAT" ? colors.warning :
        gpsStatus.fixType === "GPS_FIX" ? colors.warning :
        colors.danger;

    const batteryPercent = computeBatteryPercent(
        highLevelStatus.battery_percent, power.v_battery, settings,
    );

    const isMowing = stateNum === 2 || stateNum === 3 || stateNum === 4;

    const pulseAnimation = isEmergency
        ? 'mowerPulseRed 1.5s ease-in-out infinite'
        : isMowing
            ? 'mowerPulseGreen 2s ease-in-out infinite'
            : 'none';

    const hasArea = highLevelStatus.current_area !== undefined && highLevelStatus.current_area >= 0;
    // Coarse sub-path X/Y — the secondary readout.
    const totalSwaths = highLevelStatus.current_path ?? 0;
    const completedSwaths = highLevelStatus.current_path_index ?? 0;
    const hasSwaths = isMowing && totalSwaths > 0;
    // Smooth pose-cursor coverage percent — the PRIMARY %. Read defensively so
    // this compiles before the ROS TS types are regenerated from the updated
    // HighLevelStatus.msg (field added this change).
    const coveragePercent = (highLevelStatus as {coverage_percent?: number}).coverage_percent;
    const hasCoveragePercent = isMowing && coveragePercent !== undefined && coveragePercent > 0;
    // Prefer the smooth coverage_percent; fall back to the coarse swath ratio
    // when it is 0/unset (e.g. at pass start or an older backend).
    const progressPercent = hasCoveragePercent
        ? Math.round(coveragePercent as number)
        : hasSwaths
            ? Math.round((completedSwaths / totalSwaths) * 100)
            : null;

    // "Restart Mowgli" bounces the WHOLE stack (like the `mowgli-restart` CLI),
    // including the GUI container itself — so this backend goes down mid-way.
    // We show a blocking overlay and poll for the GUI to come back, then
    // hard-reload. `stackRestarting` drives both the overlay and the menu lock.
    const [stackRestarting, setStackRestarting] = useState(false);

    // After the whole-stack restart the GUI container bounces too, so poll a
    // lightweight backend endpoint every 15 s and hard-reload once it answers
    // again (it's usually back within an interval or two).
    const watchForGuiAndReload = () => {
        const poll = window.setInterval(async () => {
            try {
                const r = await fetch("/api/system/info", {cache: "no-store"});
                if (r.ok) {
                    window.clearInterval(poll);
                    window.location.reload();
                }
            } catch {
                /* GUI still restarting — keep polling */
            }
        }, 15_000);
    };

    const restartMowgli = async () => {
        setStackRestarting(true);
        try {
            // Restarts every mowgli-* container; the GUI last (fire-and-forget).
            await restartMowgliStack(guiApi);
        } catch (e: any) {
            // Non-GUI restarts failed before we took ourselves down — recover.
            setStackRestarting(false);
            notification.error({message: t('mowerStatus.mowgliRestartFailed'), description: e.message});
            return;
        }
        // GUI is now bouncing; wait for it to return, then reload the page.
        watchForGuiAndReload();
    };

    // Latched-emergency reset: firmware is the safety authority and only
    // clears the latch when the physical trigger is no longer asserted, so
    // this is a fire-and-forget request — surfaced as a global icon button
    // alongside the status badges so the operator never has to dig into the
    // dashboard hero card to clear it (issue #149).
    const mowerAction = useMowerAction();
    const resetEmergencyAction = mowerAction("emergency", {Emergency: 0});
    const rebootBoardAction = mowerAction("reboot_board", {});
    const showResetEmergency =
        emergencyData.active_emergency || emergencyData.latched_emergency || isEmergency;

    const rebootSystem = async () => {
        try {
            await guiApi.request({path: "/system/reboot", method: "POST"});
            notification.success({message: t('mowerStatus.restarting')});
        } catch (e: any) {
            notification.error({message: t('mowerStatus.restartFailed'), description: e.message});
        }
    };

    const shutdownSystem = async () => {
        try {
            await guiApi.request({path: "/system/shutdown", method: "POST"});
            notification.success({message: t('mowerStatus.shuttingDown')});
        } catch (e: any) {
            notification.error({message: t('mowerStatus.shutdownFailed'), description: e.message});
        }
    };

    const confirmAction = (title: string, content: string, onOk: () => Promise<void>) => {
        // Use the App-context modal (App.useApp().modal), NOT the static
        // Modal.confirm: antd v5's static API renders outside the App/
        // ConfigProvider tree, so on React 19 the confirm dialog never
        // appears — the whole power menu (restart/reboot/shutdown) silently
        // did nothing because onOk never fired. The App-context variant
        // inherits the theme, z-index stack, and portal so it shows.
        modal.confirm({
            title,
            content,
            okText: t('mowerStatus.confirm'),
            okType: "danger",
            cancelText: t('mowerStatus.cancel'),
            onOk,
        });
    };

    // Beginner-safe items at the top; destructive system/hardware actions
    // (board reset, Pi reboot, shutdown) are tucked into an "Avancé"
    // submenu so they're not hit by accident. Command logic unchanged.
    const powerMenuItems: MenuProps["items"] = [
        {
            key: "restart-mowgli",
            icon: <ReloadOutlined/>,
            label: stackRestarting ? t('mowerStatus.restartingMowgli') : t('mowerStatus.restartMowgli'),
            disabled: stackRestarting,
            onClick: () => confirmAction(t('mowerStatus.restartMowgli'), t('mowerStatus.restartMowgliConfirm'), restartMowgli),
        },
        {type: "divider"},
        {
            key: "advanced",
            icon: <SettingOutlined/>,
            label: t('mowerStatus.advanced'),
            children: [
                {
                    key: "reboot-board",
                    icon: <ReloadOutlined/>,
                    label: t('mowerStatus.restartBoard'),
                    onClick: () => confirmAction(
                        t('mowerStatus.restartBoard'),
                        t('mowerStatus.restartBoardConfirm'),
                        rebootBoardAction),
                },
                {
                    key: "reboot",
                    icon: <DesktopOutlined/>,
                    label: t('mowerStatus.restartPi'),
                    onClick: () => confirmAction(t('mowerStatus.restartPi'), t('mowerStatus.restartPiConfirm'), rebootSystem),
                },
                {
                    key: "shutdown",
                    icon: <PoweroffOutlined/>,
                    label: t('mowerStatus.shutdownPi'),
                    danger: true,
                    onClick: () => confirmAction(t('mowerStatus.shutdownPi'), t('mowerStatus.shutdownPiConfirm'), shutdownSystem),
                },
            ],
        },
    ];

    return (
        <>
            <style>{pulseKeyframes}</style>
            {/* Blocking overlay while the whole stack (incl. this GUI) restarts.
                The page reconnects and reloads itself via watchForGuiAndReload. */}
            <Modal
                open={stackRestarting}
                closable={false}
                maskClosable={false}
                keyboard={false}
                footer={null}
                title={t('mowerStatus.restartingStackTitle')}
            >
                <Space>
                    <Spin/>
                    <Typography.Text>{t('mowerStatus.restartingStackBody')}</Typography.Text>
                </Space>
            </Modal>
            <Space size="small" style={{flexShrink: 0}}>
                <Space size={4}>
                    <Badge
                        color={statusColor(stateNum, isEmergency, colors)}
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
                        {hasSwaths ? ` · ${completedSwaths}/${totalSwaths}` : ''}
                    </Typography.Text>
                )}
                <Tooltip title={`GPS: ${gpsStatus.label}`}>
                    <Space size={4}>
                        <WifiOutlined style={{color: gpsColor, fontSize: 13}}/>
                        <Typography.Text style={{fontSize: 12, color: colors.text, whiteSpace: 'nowrap'}}>
                            {gpsStatus.percent}% · {gpsStatus.label}
                        </Typography.Text>
                    </Space>
                </Tooltip>
                {showResetEmergency && (
                    <Tooltip title={t('mowerStatus.rearmEmergencyTooltip')}>
                        <Button
                            danger
                            size="small"
                            icon={<AlertOutlined/>}
                            onClick={resetEmergencyAction}
                        >
                            {t('mowerStatus.rearm')}
                        </Button>
                    </Tooltip>
                )}
                <Dropdown menu={{items: powerMenuItems}} trigger={["click"]} placement="bottomRight">
                    {/* Real button (not a Space div) so the power menu is
                        keyboard-focusable/activatable. type="text" keeps the
                        chromeless icon+text look. */}
                    <Button
                        type="text"
                        size="small"
                        aria-label={t('mowerStatus.powerMenuAria')}
                        style={{padding: '0 4px', height: 'auto'}}
                    >
                        <Space size={4}>
                            <PoweroffOutlined style={{
                                color: isCharging ? colors.primary : colors.muted,
                                fontSize: 13,
                            }}/>
                            <Typography.Text style={{fontSize: 12, color: colors.text}}>
                                {batteryPercent}%
                            </Typography.Text>
                        </Space>
                    </Button>
                </Dropdown>
            </Space>
        </>
    );
}
