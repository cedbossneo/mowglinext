import {Col, Row, Statistic} from "antd";
import {useImu} from "../hooks/useImu.ts";

export function ImuComponent() {
    const imu = useImu();
    return <Row gutter={[16, 16]}>
        <Col lg={8} xs={24}><Statistic precision={9} title="Angular Velocity X"
                                       value={imu.angular_velocity?.x}/></Col>
        <Col lg={8} xs={24}><Statistic precision={9} title="Angular Velocity Y"
                                       value={imu.angular_velocity?.y}/></Col>
        <Col lg={8} xs={24}><Statistic precision={9} title="Angular Velocity Z"
                                       value={imu.angular_velocity?.z}/></Col>
        <Col lg={8} xs={24}><Statistic precision={9} title="Linear Acceleration X"
                                       value={imu.linear_acceleration?.x}/></Col>
        <Col lg={8} xs={24}><Statistic precision={9} title="Linear Acceleration Y"
                                       value={imu.linear_acceleration?.y}/></Col>
        <Col lg={8} xs={24}><Statistic precision={9} title="Linear Acceleration Z"
                                       value={imu.linear_acceleration?.z}/></Col>
        <Col lg={8} xs={24}><Statistic precision={9} title="Orientation X" value={imu.orientation?.x}/></Col>
        <Col lg={8} xs={24}><Statistic precision={9} title="Orientation Y" value={imu.orientation?.y}/></Col>
        <Col lg={8} xs={24}><Statistic precision={9} title="Orientation Z" value={imu.orientation?.z}/></Col>
    </Row>;
}