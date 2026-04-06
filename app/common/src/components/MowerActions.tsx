import {useApi} from "../hooks/useApi";
import {Card, Col, Divider, Row} from "antd";
import {PlayCircleOutlined, HomeOutlined, WarningOutlined} from '@ant-design/icons';
import {AsyncButton} from "./AsyncButton";
import React from "react";
import styled from "styled-components";
import {AsyncDropDownButton} from "./AsyncDropDownButton";
import {useHighLevelStatus} from "../hooks/useHighLevelStatus";

const ActionsCard = styled(Card)`
  .ant-card-body > button {
    margin-right: 10px;
    margin-bottom: 10px;
  }
`;

export const useMowerAction = () => {
    const guiApi = useApi()
    return (command: string, args: Record<string, any> = {}) => async () => {
        try {
            const res = await guiApi.openmower.callCreate(command, args)
            if (res.error) {
                throw new Error(res.error.error)
            }
        } catch (e: any) {
            throw new Error(e.message)
        }
    };
};

export const MowerActions: React.FC<React.PropsWithChildren<{bare?: boolean}>> = (props) => {
    const {highLevelStatus} = useHighLevelStatus();
    const mowerAction = useMowerAction()
    const actionMenuItems: {
        key: string,
        label: string,
        actions: { command: string, args: any }[],
        danger?: boolean
    }[] = [
        {
            key: "mower_s1",
            label: "Area Recording",
            actions: [{
                command: "high_level_control",
                args: {
                    Command: 3,
                }
            }]
        },
        {
            key: "mower_s2",
            label: "Mow Next Area",
            actions: [{
                command: "high_level_control",
                args: {
                    Command: 4,
                }
            }]
        },
        {
            key: highLevelStatus.StateName == "IDLE" ? "continue" : "pause",
            label: highLevelStatus.StateName == "IDLE" ? "Continue" : "Pause",
            actions: highLevelStatus.StateName == "IDLE" ? [{
                command: "mower_logic", args: {
                    Config: {
                        Bools: [{
                            Name: "manual_pause_mowing",
                            Value: false
                        }]
                    }
                }
            }, {
                command: "high_level_control",
                args: {
                    Command: 1,
                }
            }] : [{
                command: "mower_logic", args: {
                    Config: {
                        Bools: [{
                            Name: "manual_pause_mowing",
                            Value: true
                        }]
                    }
                }
            }]
        },
        {
            key: "emergency_off",
            "label": "Emergency Off",
            "danger": true,
            actions: [{
                command: "emergency",
                args: {
                    Emergency: 0,
                }
            }]
        },
        {
            key: "mow_forward",
            "label": "Blade Forward",
            actions: [{
                command: "mow_enabled",
                args: {MowEnabled: 1, MowDirection: 0}
            }]
        },
        {
            key: "mow_backward",
            "label": "Blade Backward",
            actions: [{
                command: "mow_enabled",
                args: {MowEnabled: 1, MowDirection: 1}
            }]
        },
        {
            key: "mow_off",
            "label": "Blade Off",
            "danger": true,
            actions: [{
                command: "mow_enabled",
                args: {MowEnabled: 0, MowDirection: 0}
            }]
        },
    ];
    let children = props.children;
    if (children && Array.isArray(children)) {
        children = children.map(c => {
            return c ? <Col>{c}</Col> : null
        })
    } else if (children) {
        children = <Col>{children}</Col>
    }
    const content = (
        <Row gutter={[8, 8]} justify={"start"}>
            {children}
            {children ? <Col><Divider type={"vertical"}/></Col> : null}
            <Col>
                {highLevelStatus.StateName == "IDLE" ? <AsyncButton icon={<PlayCircleOutlined/>} type="primary" key="btnHLC1"
                                                                          onAsyncClick={mowerAction("high_level_control", {Command: 1})}
                >Start</AsyncButton> : null}
                {highLevelStatus.StateName !== "IDLE" ? <AsyncButton icon={<HomeOutlined/>} type="primary" key="btnHLC2"
                                                                           onAsyncClick={mowerAction("high_level_control", {Command: 2})}
                >Home</AsyncButton> : null}
            </Col>
            <Col>
                {!highLevelStatus.Emergency ?
                    <AsyncButton danger icon={<WarningOutlined/>} key="btnEmergencyOn" onAsyncClick={mowerAction("emergency", {Emergency: 1})}
                    >Emergency On</AsyncButton> : null}
                {highLevelStatus.Emergency ?
                    <AsyncButton danger icon={<WarningOutlined/>} key="btnEmergencyOff" onAsyncClick={mowerAction("emergency", {Emergency: 0})}
                    >Emergency Off</AsyncButton> : null}
            </Col>
            <Col>
                <AsyncDropDownButton style={{display: "inline"}}  key="drpActions"  menu={{
                    items: actionMenuItems,
                    onAsyncClick: async (e) => {
                        const item = actionMenuItems.find(item => item.key == e.key)
                        for (const action of (item?.actions ?? [])) {
                            await mowerAction(action.command, action.args)();
                        }
                    }
                }}>
                    More
                </AsyncDropDownButton>
            </Col>
        </Row>
    );

    if (props.bare) {
        return content;
    }

    return <ActionsCard title={"Actions"} size={"small"}>
        {content}
    </ActionsCard>;
};
