import {Col, Row, Statistic} from "antd";
import {useWheelTicks} from "../hooks/useWheelTicks.ts";

export function WheelTicksComponent() {
    const wheelTicks = useWheelTicks();
    return <Row gutter={[16, 16]}>
        <Col lg={8} xs={24}><Statistic title="Rear Left" value={wheelTicks?.wheel_ticks_rl}/></Col>
        <Col lg={8} xs={24}><Statistic title="Rear Right" value={wheelTicks?.wheel_ticks_rr}/></Col>
        <Col lg={8} xs={24}><Statistic title="Rear Left Direction" value={wheelTicks?.wheel_direction_rl}/></Col>
        <Col lg={8} xs={24}><Statistic title="Rear Right Direction" value={wheelTicks?.wheel_direction_rr}/></Col>
    </Row>;
}