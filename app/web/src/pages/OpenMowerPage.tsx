import {Card, Col, Collapse, Row} from "antd";
import {StatusComponent} from "../components/StatusComponent";
import {SystemInfoComponent} from "../components/SystemInfoComponent";
import {HighLevelStatusComponent} from "../components/HighLevelStatusComponent";
import {ImuComponent} from "../components/ImuComponent";
import {WheelTicksComponent} from "../components/WheelTicksComponent";
import {GpsComponent} from "../components/GpsComponent";
import {MiniMap} from "../components/MiniMap";
import {useIsMobile} from "../hooks/useIsMobile";
import {MowerActions} from "@mowglinext/common";

export const OpenMowerPage = () => {
    const isMobile = useIsMobile();

    if (isMobile) {
        return (
            <div style={{display: 'flex', flexDirection: 'column', gap: 12, paddingBottom: 8}}>
                {/* Hero: Mini live map */}
                <MiniMap height={180}/>

                {/* Action buttons */}
                <MowerActions bare/>

                {/* Compact status */}
                <StatusComponent compact/>

                {/* Detailed info in accordion */}
                <Collapse
                    defaultActiveKey={[]}
                    size="small"
                    items={[
                        {
                            key: "system",
                            label: "System Info",
                            children: <SystemInfoComponent/>,
                        },
                        {
                            key: "details",
                            label: "Detailed Status",
                            children: <StatusComponent/>,
                        },
                        {
                            key: "sensors",
                            label: "Sensors & Diagnostics",
                            children: <Row gutter={[16, 16]}>
                                <Col span={24}>
                                    <Card title="IMU" size="small"><ImuComponent/></Card>
                                </Col>
                                <Col span={24}>
                                    <Card title="GPS" size="small"><GpsComponent/></Card>
                                </Col>
                                <Col span={24}>
                                    <Card title="Wheel Ticks" size="small"><WheelTicksComponent/></Card>
                                </Col>
                            </Row>,
                        },
                    ]}
                />
            </div>
        );
    }

    return <Row gutter={[16, 16]}>
        <Col lg={12} xs={24}>
            <MowerActions/>
        </Col>
        <Col lg={12} xs={24}>
            <Card title="Overview" size="small">
                <HighLevelStatusComponent/>
            </Card>
        </Col>
        <Col span={24}>
            <StatusComponent/>
        </Col>
        <Col span={24}>
            <SystemInfoComponent/>
        </Col>
        <Col span={24}>
            <Collapse
                defaultActiveKey={[]}
                items={[
                    {
                        key: "sensors",
                        label: "Sensors & Diagnostics",
                        children: <Row gutter={[16, 16]}>
                            <Col span={24}>
                                <Card title="IMU" size="small">
                                    <ImuComponent/>
                                </Card>
                            </Col>
                            <Col lg={12} xs={24}>
                                <Card title="GPS" size="small">
                                    <GpsComponent/>
                                </Card>
                            </Col>
                            <Col lg={12} xs={24}>
                                <Card title="Wheel Ticks" size="small">
                                    <WheelTicksComponent/>
                                </Card>
                            </Col>
                        </Row>,
                    },
                ]}
            />
        </Col>
    </Row>
}

export default OpenMowerPage;
