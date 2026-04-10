import {Col, Row, Statistic} from "antd";
import {booleanFormatter, booleanFormatterInverted, progressFormatter, stateRenderer} from "./utils.tsx";
import {useHighLevelStatus} from "../hooks/useHighLevelStatus.ts";
import {usePower} from "../hooks/usePower.ts";
import {useStatus} from "../hooks/useStatus.ts";
import {useEmergency} from "../hooks/useEmergency.ts";
import {useGPS} from "../hooks/useGPS.ts";
import {useSettings} from "../hooks/useSettings.ts";
import {useThemeMode} from "../theme/ThemeContext.tsx";
import {AbsolutePoseConstants} from "../types/ros.ts";

export function HighLevelStatusComponent() {
    const {colors} = useThemeMode();
    const {highLevelStatus} = useHighLevelStatus()
    const power = usePower()
    const status = useStatus()
    const emergency = useEmergency()
    const gps = useGPS()
    const {settings} = useSettings()

    // Derive charging state: prefer highLevelStatus, fall back to status topic
    const isCharging = highLevelStatus.is_charging ?? status.is_charging ?? false;

    // Derive emergency state: prefer highLevelStatus, fall back to emergency topic
    const isEmergency = highLevelStatus.emergency ?? emergency.active_emergency ?? false;

    // Derive battery percentage: prefer highLevelStatus, fall back to voltage-based estimate
    const batteryPercent = (() => {
        if (highLevelStatus.battery_percent != null && highLevelStatus.battery_percent > 0) {
            return highLevelStatus.battery_percent * 100;
        }
        // Estimate from voltage if highLevelStatus is unavailable
        if (power.v_battery) {
            const full = parseFloat(settings["battery_full_voltage"] ?? "28.5");
            const empty = parseFloat(settings["battery_empty_voltage"] ?? "23.0");
            const pct = ((power.v_battery - empty) / (full - empty)) * 100;
            return Math.max(0, Math.min(100, pct));
        }
        return 0;
    })();

    // Derive GPS quality: prefer highLevelStatus, fall back to GPS topic flags
    const gpsQuality = (() => {
        if (highLevelStatus.gps_quality_percent != null && highLevelStatus.gps_quality_percent > 0) {
            return highLevelStatus.gps_quality_percent * 100;
        }
        // Estimate from GPS flags (RTK=1 means "has GPS", FIXED=2 means RTK fix, FLOAT=4 means RTK float)
        if (gps.flags != null) {
            if (gps.flags & AbsolutePoseConstants.FLAG_GPS_RTK_FIXED) return 100;   // RTK fixed
            if (gps.flags & AbsolutePoseConstants.FLAG_GPS_RTK_FLOAT) return 50;    // RTK float
            if (gps.flags & AbsolutePoseConstants.FLAG_GPS_RTK) return 25;      // GPS fix, no RTK
        }
        return 0;
    })();

    // Derive state name: prefer highLevelStatus, fall back to basic inference
    const stateName = highLevelStatus.state_name ?? (
        isEmergency ? "EMERGENCY" :
        isCharging ? "CHARGING" :
        status.mower_status != null ? "IDLE" :
        undefined
    );

    const estimateRemainingChargingTime = () => {
        if (!power.v_battery || !power.charge_current || power.charge_current == 0) {
            return "∞"
        }
        const capacity = (settings["battery_capacity_mah"] ?? "3000.0");
        const full = (settings["battery_full_voltage"] ?? "28.5");
        const empty = (settings["battery_empty_voltage"] ?? "23.0");
        if (!capacity || !full || !empty) {
            return "∞"
        }
        const estimatedAmpsPerVolt = parseFloat(capacity) / (parseFloat(full) - parseFloat(empty))
        const estimatedRemainingAmps = (parseFloat(full) - (power.v_battery ?? 0)) * estimatedAmpsPerVolt;
        if (estimatedRemainingAmps < 10) {
            return "∞"
        }
        const remaining = estimatedRemainingAmps / ((power.charge_current ?? 0) * 1000)
        if (remaining < 0) {
            return "∞"
        }
        return Date.now() + remaining * (1000 * 60 * 60)
    };
    return <Row gutter={[16, 16]}>
        <Col lg={6} xs={12}><Statistic title="State" valueStyle={{color: colors.primary}}
                                       value={stateRenderer(stateName)}/></Col>
        <Col lg={6} xs={12}><Statistic title="GPS" precision={2}
                                       value={gpsQuality}
                                       suffix={"%"}/></Col>
        <Col lg={6} xs={12}><Statistic title="Battery" value={batteryPercent}
                                       formatter={progressFormatter}/></Col>
        <Col lg={6} xs={12}>{isCharging ?
            <Statistic.Countdown title="Charge ETA" format={"HH:mm"}
                                       value={estimateRemainingChargingTime()}/> :
            <Statistic title="Charge ETA" value="--:--"/>}
        </Col>
        <Col lg={6} xs={12}><Statistic title="Charging" value={isCharging ? "Yes" : "No"}
                                       formatter={booleanFormatter}/></Col>
        <Col lg={6} xs={12}><Statistic title="Emergency" value={isEmergency ? "Yes" : "No"}
                                       formatter={booleanFormatterInverted}/></Col>
    </Row>;
}
