import {useCallback, useEffect, useMemo, useRef, useState} from "react";
import {App, Button, Tag} from "antd";
import {Joystick} from "react-joystick-component";
import type {IJoystickUpdateEvent} from "react-joystick-component/build/lib/Joystick";
import {CircleDot, Gamepad2, MapPin, Radio, Scissors, Square, SquareDot} from "lucide-react";
import {useTranslation} from "react-i18next";

import {useMowerAction} from "../components/MowerActions.tsx";
import {LiveMapMini, type MiniArea} from "../concept/components/LiveMapMini.tsx";
import {useFusionOdom} from "../hooks/useFusionOdom.ts";
import {useHighLevelStatus} from "../hooks/useHighLevelStatus.ts";
import {useMowingMap} from "../hooks/useMowingMap.ts";
import {useIsMobile} from "../hooks/useIsMobile.ts";
import {useWS} from "../hooks/useWS.ts";
import {useThemeMode} from "../theme/ThemeContext.tsx";
import {limeAlpha} from "../theme/colors.ts";
import type {TwistStamped} from "../types/ros.ts";

const MAX_LINEAR_MPS = 0.25;
const MAX_ANGULAR_RAD_S = 0.6;

type DriveMode = "mow" | "transport" | "record";

function normalisedMap(map: ReturnType<typeof useMowingMap>, odom: ReturnType<typeof useFusionOdom>) {
    const areas = [...(map.working_area ?? []), ...(map.navigation_areas ?? [])]
        .filter(area => (area.area?.points?.length ?? 0) >= 3);
    const points = areas.flatMap(area => area.area?.points ?? []);
    if (points.length === 0) return {polygons: [] as MiniArea[], robot: undefined};

    const xs = points.map(point => point.x ?? 0);
    const ys = points.map(point => point.y ?? 0);
    const x0 = Math.min(...xs);
    const y0 = Math.min(...ys);
    const dx = Math.max(Math.max(...xs) - x0, 0.1);
    const dy = Math.max(Math.max(...ys) - y0, 0.1);
    const normalise = (x: number, y: number) => ({
        x: 0.10 + 0.80 * (x - x0) / dx,
        y: 0.90 - 0.80 * (y - y0) / dy,
    });
    const polygons = areas.map(area => ({
        outer: (area.area?.points ?? []).map(point => normalise(point.x ?? 0, point.y ?? 0)),
        holes: (area.obstacles ?? []).map(hole =>
            (hole.points ?? []).map(point => normalise(point.x ?? 0, point.y ?? 0))),
    }));
    const position = odom.pose?.pose?.position;
    const orientation = odom.pose?.pose?.orientation;
    const heading = orientation
        ? Math.atan2(2 * ((orientation.w ?? 1) * (orientation.z ?? 0) + (orientation.x ?? 0) * (orientation.y ?? 0)),
            1 - 2 * ((orientation.y ?? 0) ** 2 + (orientation.z ?? 0) ** 2)) * 180 / Math.PI
        : 0;
    return {
        polygons,
        robot: position ? {...normalise(position.x ?? x0, position.y ?? y0), heading} : undefined,
    };
}

export const ManualControllerPage = () => {
    const {colors} = useThemeMode();
    const {t} = useTranslation();
    const {modal, notification} = App.useApp();
    const mowerAction = useMowerAction();
    const {highLevelStatus} = useHighLevelStatus();
    const map = useMowingMap();
    const odom = useFusionOdom();
    const isMobile = useIsMobile();
    const joyStream = useWS(() => {}, () => {}, () => {});
    const [mode, setMode] = useState<DriveMode>("transport");
    const [armed, setArmed] = useState(false);
    const [isLandscape, setIsLandscape] = useState(() => typeof window !== "undefined" && window.matchMedia("(orientation: landscape)").matches);
    const [compactLandscape, setCompactLandscape] = useState(() => typeof window !== "undefined" && window.matchMedia("(orientation: landscape) and (max-height: 600px)").matches);
    const linearRef = useRef(0);
    const angularRef = useRef(0);

    useEffect(() => {
        const media = window.matchMedia("(orientation: landscape)");
        const compactMedia = window.matchMedia("(orientation: landscape) and (max-height: 600px)");
        const update = () => {
            setIsLandscape(media.matches);
            setCompactLandscape(compactMedia.matches);
        };
        media.addEventListener("change", update);
        compactMedia.addEventListener("change", update);
        return () => {
            media.removeEventListener("change", update);
            compactMedia.removeEventListener("change", update);
        };
    }, []);

    useEffect(() => {
        if (!armed) return;
        joyStream.start("/api/mowglinext/publish/joy");
        return () => joyStream.stop();
        // The stream is deliberately open only while an explicit mode is active.
        // eslint-disable-next-line react-hooks/exhaustive-deps
    }, [armed]);

    const publishTwist = useCallback((force = false) => {
        if (!armed && !force) return;
        const message: TwistStamped = {
            header: {stamp: {sec: 0, nanosec: 0}, frame_id: ""},
            twist: {
                linear: {x: linearRef.current, y: 0, z: 0},
                angular: {x: 0, y: 0, z: angularRef.current},
            },
        };
        joyStream.sendJsonMessage(message);
        // eslint-disable-next-line react-hooks/exhaustive-deps
    }, [armed, joyStream.sendJsonMessage]);

    const onDriveMove = useCallback((event: IJoystickUpdateEvent) => {
        linearRef.current = (event.y ?? 0) * MAX_LINEAR_MPS;
        publishTwist();
    }, [publishTwist]);
    const onSteerMove = useCallback((event: IJoystickUpdateEvent) => {
        angularRef.current = -(event.x ?? 0) * MAX_ANGULAR_RAD_S;
        publishTwist();
    }, [publishTwist]);
    const onCombinedMove = useCallback((event: IJoystickUpdateEvent) => {
        linearRef.current = (event.y ?? 0) * MAX_LINEAR_MPS;
        angularRef.current = -(event.x ?? 0) * MAX_ANGULAR_RAD_S;
        publishTwist();
    }, [publishTwist]);
    const stopDrive = useCallback(() => {
        linearRef.current = 0;
        publishTwist();
    }, [publishTwist]);
    const stopSteer = useCallback(() => {
        angularRef.current = 0;
        publishTwist();
    }, [publishTwist]);
    const stopCombined = useCallback(() => {
        linearRef.current = 0;
        angularRef.current = 0;
        publishTwist();
    }, [publishTwist]);

    useEffect(() => {
        if (!armed) return;
        const pressed = new Set<string>();
        const updateFromKeys = () => {
            linearRef.current = ((pressed.has("w") ? 1 : 0) - (pressed.has("s") ? 1 : 0)) * MAX_LINEAR_MPS;
            angularRef.current = ((pressed.has("a") ? 1 : 0) - (pressed.has("d") ? 1 : 0)) * MAX_ANGULAR_RAD_S;
            publishTwist();
        };
        const stopForFocusLoss = () => {
            pressed.clear();
            linearRef.current = 0;
            angularRef.current = 0;
            publishTwist(true);
        };
        const onKeyDown = (event: KeyboardEvent) => {
            const key = event.key.toLowerCase();
            if (key === " ") {
                event.preventDefault();
                stopForFocusLoss();
                return;
            }
            if (!["w", "a", "s", "d"].includes(key)) return;
            event.preventDefault();
            pressed.add(key);
            updateFromKeys();
        };
        const onKeyUp = (event: KeyboardEvent) => {
            const key = event.key.toLowerCase();
            if (!["w", "a", "s", "d"].includes(key)) return;
            event.preventDefault();
            pressed.delete(key);
            updateFromKeys();
        };
        const onVisibilityChange = () => {
            if (document.visibilityState !== "visible") stopForFocusLoss();
        };
        window.addEventListener("keydown", onKeyDown);
        window.addEventListener("keyup", onKeyUp);
        window.addEventListener("blur", stopForFocusLoss);
        document.addEventListener("visibilitychange", onVisibilityChange);
        return () => {
            stopForFocusLoss();
            window.removeEventListener("keydown", onKeyDown);
            window.removeEventListener("keyup", onKeyUp);
            window.removeEventListener("blur", stopForFocusLoss);
            document.removeEventListener("visibilitychange", onVisibilityChange);
        };
    }, [armed, publishTwist]);

    const run = async (nextMode: DriveMode) => {
        const command = nextMode === "mow" ? 7 : nextMode === "transport" ? 9 : 3;
        await mowerAction("high_level_control", {Command: command})();
        setMode(nextMode);
        setArmed(true);
    };
    const stop = async () => {
        linearRef.current = 0;
        angularRef.current = 0;
        publishTwist(true);
        setArmed(false);
        await mowerAction("high_level_control", {Command: 8})();
    };
    const emergencyStop = () => modal.confirm({
        title: t("manualController.emergencyTitle"),
        content: t("manualController.emergencyBody"),
        okText: t("manualController.emergency"),
        okType: "danger",
        cancelText: t("manualController.cancel"),
        onOk: () => mowerAction("emergency", {Emergency: 1})(),
    });
    const rearm = async () => {
        await mowerAction("emergency", {Emergency: 0})();
        notification.success({message: t("manualController.rearmed")});
    };

    const mapData = useMemo(() => normalisedMap(map, odom), [map, odom]);
    const stateName = highLevelStatus.state_name ?? "—";
    const isEmergency = highLevelStatus.emergency ?? false;
    const joystickBase = `radial-gradient(circle at 50% 40%, ${limeAlpha(0.12)}, ${colors.bgBase} 72%)`;
    const joystickStick = `radial-gradient(circle at 38% 32%, ${colors.primaryLight}, ${colors.mint} 55%, ${colors.emeraldDeep})`;
    const actionStyle: React.CSSProperties = {
        minHeight: 52, flex: 1, fontWeight: 700, letterSpacing: "0.01em",
    };
    const dualStick = isMobile && isLandscape;

    return (
        <section aria-label={t("manualController.ariaLabel")} style={{width: "100%", minWidth: 0, maxWidth: 1180, margin: "0 auto"}}>
            <div className="manual-controller-heading" style={{display: "flex", alignItems: "center", justifyContent: "space-between", gap: 12, marginBottom: 12}}>
                <div>
                    <div style={{display: "flex", alignItems: "center", gap: 8}}>
                        <Gamepad2 size={22} color={colors.primary}/>
                        <h1 className="mn-display" style={{margin: 0, color: colors.text, fontSize: 28}}>{t("manualController.title")}</h1>
                    </div>
                    <p style={{margin: "4px 0 0", color: colors.textSecondary}}>{t("manualController.subtitle")}</p>
                </div>
                <Tag color={isEmergency ? "error" : "success"} style={{margin: 0, fontWeight: 700}}>{isEmergency ? t("manualController.emergencyActive") : stateName}</Tag>
            </div>

            <div style={{display: "grid", gridTemplateColumns: dualStick ? "minmax(0, 0.85fr) minmax(0, 1.2fr) minmax(0, 0.85fr)" : "minmax(0, 0.9fr) minmax(0, 1.1fr)", gap: 16, alignItems: "stretch"}} className={`manual-controller-grid ${dualStick ? "manual-controller-dual" : "manual-controller-combined"}`}>
                {dualStick ? <JoystickPanel label={t("manualController.driveStick")} hint={t("manualController.driveHint")} compact={compactLandscape}>
                    <Joystick size={compactLandscape ? 88 : 154} stickSize={compactLandscape ? 40 : 66} baseColor={joystickBase} stickColor={joystickStick} move={onDriveMove} stop={stopDrive} throttle={50}/>
                </JoystickPanel> : <JoystickPanel label={t("manualController.combinedStick")} hint={t("manualController.combinedHint")} combined>
                    <Joystick size={isMobile ? 184 : 136} stickSize={isMobile ? 78 : 58} baseColor={joystickBase} stickColor={joystickStick} move={onCombinedMove} stop={stopCombined} throttle={50}/>
                </JoystickPanel>}

                <div className={compactLandscape ? "manual-controller-map manual-controller-map-compact" : "manual-controller-map"} style={{minWidth: 0, borderRadius: 20, overflow: "hidden", background: colors.panel, border: `1px solid ${colors.border}`, boxShadow: colors.glassShadow}}>
                    <div style={{display: "flex", alignItems: "center", justifyContent: "space-between", padding: "12px 14px", borderBottom: `1px solid ${colors.border}`}}>
                        <span style={{fontWeight: 700, color: colors.text}}><MapPin size={16} style={{verticalAlign: "-3px", marginRight: 6}}/>{t("manualController.map")}</span>
                        <span style={{fontSize: 12, color: colors.textSecondary}}>{t("manualController.mapHint")}</span>
                    </div>
                    {mapData.polygons.length > 0 ? <LiveMapMini polygons={mapData.polygons} robot={mapData.robot} coverage={0} height={compactLandscape ? 68 : 250}/> : (
                        <div style={{height: compactLandscape ? 68 : 250, display: "grid", placeItems: "center", color: colors.textSecondary, background: `linear-gradient(135deg, ${colors.bgSubtle}, ${colors.panel})`}}>
                            <span><MapPin size={18} style={{verticalAlign: "-4px", marginRight: 6}}/>{t("manualController.awaitingMap")}</span>
                        </div>
                    )}
                    <div style={{padding: "10px 14px", display: "flex", justifyContent: "space-between", color: colors.textSecondary, fontSize: 12}}>
                        <span><CircleDot size={14} color={colors.primary} style={{verticalAlign: "-2px", marginRight: 5}}/>{t("manualController.livePosition")}</span>
                        <span>{t("manualController.speed", {value: Math.abs(odom.twist?.twist?.linear?.x ?? 0).toFixed(2)})}</span>
                    </div>
                </div>

                {dualStick && <JoystickPanel label={t("manualController.steerStick")} hint={t("manualController.steerHint")} compact={compactLandscape}>
                    <Joystick size={compactLandscape ? 88 : 154} stickSize={compactLandscape ? 40 : 66} baseColor={joystickBase} stickColor={joystickStick} move={onSteerMove} stop={stopSteer} throttle={50}/>
                </JoystickPanel>}
            </div>
            {!isMobile && <p style={{margin: "10px 0 0", color: colors.textSecondary, fontSize: 13}}>{t("manualController.keyboardHint")}</p>}

            <div style={{display: "grid", gridTemplateColumns: "repeat(5, minmax(0, 1fr))", gap: 10, marginTop: 16}} className="manual-controller-actions">
                <Button type={mode === "mow" ? "primary" : "default"} icon={<Scissors size={17}/>} onClick={() => run("mow")} style={actionStyle}>{t("manualController.mow")}</Button>
                <Button type={mode === "transport" ? "primary" : "default"} icon={<Radio size={17}/>} onClick={() => run("transport")} style={actionStyle}>{t("manualController.transport")}</Button>
                <Button type={mode === "record" ? "primary" : "default"} icon={<SquareDot size={17}/>} onClick={() => run("record")} style={actionStyle}>{t("manualController.record")}</Button>
                <Button icon={<Square size={17}/>} onClick={stop} style={actionStyle}>{t("manualController.stop")}</Button>
                {isEmergency ? <Button onClick={rearm} style={actionStyle}>{t("manualController.rearm")}</Button> : <Button danger type="primary" icon={<Square size={17}/>} onClick={emergencyStop} style={actionStyle}>{t("manualController.emergency")}</Button>}
            </div>
            <style>{`
                @media (max-width: 760px) {
                    .manual-controller-grid { grid-template-columns: 1fr !important; }
                    .manual-controller-combined > div:first-child { order: 1; }
                    .manual-controller-combined > div:last-child { order: 2; }
                    .manual-controller-actions { grid-template-columns: repeat(2, minmax(0, 1fr)) !important; }
                }
                @media (orientation: landscape) and (max-height: 600px) {
                    .manual-controller-heading { display: none !important; }
                    .manual-controller-dual { grid-template-columns: 0.85fr 1.2fr 0.85fr !important; }
                    .manual-controller-actions { grid-template-columns: repeat(5, minmax(0, 1fr)) !important; }
                    .manual-controller-map-compact > div:first-child { padding: 6px 8px !important; }
                    .manual-controller-map-compact > div:first-child span:last-child { display: none; }
                    .manual-controller-map-compact > div:last-child { padding: 5px 8px !important; }
                }
            `}</style>
        </section>
    );
};

const JoystickPanel = ({label, hint, children, combined = false, compact = false}: {label: string; hint: string; children: React.ReactNode; combined?: boolean; compact?: boolean}) => {
    const {colors} = useThemeMode();
    return <div style={{display: "grid", minWidth: 0, placeItems: "center", gap: compact ? 6 : 10, padding: compact ? 8 : combined ? 20 : 16, textAlign: "center", borderRadius: 20, background: colors.panel, border: `1px solid ${colors.border}`}}>
        <strong style={{color: colors.text}}>{label}</strong>
        <div style={{touchAction: "none"}}>{children}</div>
        <span style={{fontSize: 12, color: colors.textSecondary}}>{hint}</span>
    </div>;
};

export default ManualControllerPage;
