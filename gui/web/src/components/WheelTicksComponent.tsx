import {Col, Row, Statistic} from "antd";
import {useWheelTicks} from "../hooks/useWheelTicks.ts";

export function WheelTicksComponent() {
    const wheelTicks = useWheelTicks();
    return <Row gutter={[16, 16]}>
        <Col lg={8} xs={24}><Statistic title="Rear Left" value={wheelTicks?.WheelTicksRl}/></Col>
        <Col lg={8} xs={24}><Statistic title="Rear Right" value={wheelTicks?.WheelTicksRr}/></Col>
        <Col lg={8} xs={24}><Statistic title="Rear Left Direction" value={wheelTicks?.WheelDirectionRl}/></Col>
        <Col lg={8} xs={24}><Statistic title="Rear Right Direction" value={wheelTicks?.WheelDirectionRr}/></Col>
    </Row>;
}