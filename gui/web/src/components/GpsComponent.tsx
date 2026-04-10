import {Col, Row, Statistic} from "antd";
import {useGPS} from "../hooks/useGPS.ts";
import { booleanFormatter, booleanFormatterInverted } from "./utils.tsx";
import { AbsolutePoseConstants as Flags } from "../types/ros.ts";
import {useThemeMode} from "../theme/ThemeContext.tsx";

export function GpsComponent() {
    const {colors} = useThemeMode();
    const gps = useGPS();

    const flags = gps.flags ?? 0;
    // RTK is active when FIXED or FLOAT bit is set (not just the base RTK/GPS-fix bit)
    const hasRtk = !!((flags & Flags.FIXED) || (flags & Flags.FLOAT));
    let fixType = "\u2013";
    if ((flags & Flags.FIXED) != 0) {
        fixType = "RTK FIX";
    } else if ((flags & Flags.FLOAT) != 0) {
        fixType = "RTK FLOAT";
    } else if ((flags & Flags.RTK) != 0) {
        fixType = "GPS FIX";
    }

    return <>
        <Row gutter={[16, 16]}>
            <Col lg={8} xs={24}><Statistic precision={2} title="Position X (m)"
                                        value={gps.pose?.pose?.position?.x}/></Col>
            <Col lg={8} xs={24}><Statistic precision={2} title="Position Y (m)"
                                        value={gps.pose?.pose?.position?.y}/></Col>
            <Col lg={8} xs={24}><Statistic precision={2} title="Altitude" value={gps.pose?.pose?.position?.z}/></Col>
            <Col lg={8} xs={24}><Statistic precision={2} title="Orientation"
                                        value={gps.pose?.pose?.orientation?.z}/></Col>
            <Col lg={8} xs={24}><Statistic precision={3} title="Accuracy (m)" value={gps.position_accuracy}/></Col>
            </Row>
        <Row gutter={[16, 16]}>
            <Col lg={8} xs={24}><Statistic title="RTK" value={hasRtk ? "Yes" : "No"}
                                        formatter={booleanFormatter}/></Col>
            <Col lg={8} xs={24}><Statistic title="Fix type" value={fixType}
                                        valueStyle={{color: fixType.includes("FIX") ? colors.primary : fixType.includes("FLOAT") ? colors.warning : colors.danger}}/></Col>
            <Col lg={8} xs={24}><Statistic title="Dead reckoning" value={(flags & Flags.DEAD_RECKONING) != 0 ? "Yes" : "No"}
                                        formatter={booleanFormatterInverted}/></Col>
        </Row>
    </>;
}
