import {Col, Row, Statistic} from "antd";
import {useGPS, AbsolutePoseFlags as Flags, useThemeMode, booleanFormatter, booleanFormatterInverted} from "@mowglinext/common";

export function GpsComponent() {
    const {colors} = useThemeMode();
    const gps = useGPS();

    const flags = gps.Flags ?? 0;
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
                                        value={gps.Pose?.Pose?.Position?.X}/></Col>
            <Col lg={8} xs={24}><Statistic precision={2} title="Position Y (m)"
                                        value={gps.Pose?.Pose?.Position?.Y}/></Col>
            <Col lg={8} xs={24}><Statistic precision={2} title="Altitude" value={gps.Pose?.Pose?.Position?.Z}/></Col>
            <Col lg={8} xs={24}><Statistic precision={2} title="Orientation"
                                        value={gps.Pose?.Pose?.Orientation?.Z}/></Col>
            <Col lg={8} xs={24}><Statistic precision={3} title="Accuracy (m)" value={gps.PositionAccuracy}/></Col>
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
