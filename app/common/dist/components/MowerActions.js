import { jsx as _jsx, jsxs as _jsxs } from "react/jsx-runtime";
import { useApi } from "../hooks/useApi";
import { Card, Col, Divider, Row } from "antd";
import { PlayCircleOutlined, HomeOutlined, WarningOutlined } from '@ant-design/icons';
import { AsyncButton } from "./AsyncButton";
import styled from "styled-components";
import { AsyncDropDownButton } from "./AsyncDropDownButton";
import { useHighLevelStatus } from "../hooks/useHighLevelStatus";
const ActionsCard = styled(Card) `
  .ant-card-body > button {
    margin-right: 10px;
    margin-bottom: 10px;
  }
`;
export const useMowerAction = () => {
    const guiApi = useApi();
    return (command, args = {}) => async () => {
        try {
            const res = await guiApi.openmower.callCreate(command, args);
            if (res.error) {
                throw new Error(res.error.error);
            }
        }
        catch (e) {
            throw new Error(e.message);
        }
    };
};
export const MowerActions = (props) => {
    const { highLevelStatus } = useHighLevelStatus();
    const mowerAction = useMowerAction();
    const actionMenuItems = [
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
                    args: { MowEnabled: 1, MowDirection: 0 }
                }]
        },
        {
            key: "mow_backward",
            "label": "Blade Backward",
            actions: [{
                    command: "mow_enabled",
                    args: { MowEnabled: 1, MowDirection: 1 }
                }]
        },
        {
            key: "mow_off",
            "label": "Blade Off",
            "danger": true,
            actions: [{
                    command: "mow_enabled",
                    args: { MowEnabled: 0, MowDirection: 0 }
                }]
        },
    ];
    let children = props.children;
    if (children && Array.isArray(children)) {
        children = children.map(c => {
            return c ? _jsx(Col, { children: c }) : null;
        });
    }
    else if (children) {
        children = _jsx(Col, { children: children });
    }
    const content = (_jsxs(Row, { gutter: [8, 8], justify: "start", children: [children, children ? _jsx(Col, { children: _jsx(Divider, { type: "vertical" }) }) : null, _jsxs(Col, { children: [highLevelStatus.StateName == "IDLE" ? _jsx(AsyncButton, { icon: _jsx(PlayCircleOutlined, {}), type: "primary", onAsyncClick: mowerAction("high_level_control", { Command: 1 }), children: "Start" }, "btnHLC1") : null, highLevelStatus.StateName !== "IDLE" ? _jsx(AsyncButton, { icon: _jsx(HomeOutlined, {}), type: "primary", onAsyncClick: mowerAction("high_level_control", { Command: 2 }), children: "Home" }, "btnHLC2") : null] }), _jsxs(Col, { children: [!highLevelStatus.Emergency ?
                        _jsx(AsyncButton, { danger: true, icon: _jsx(WarningOutlined, {}), onAsyncClick: mowerAction("emergency", { Emergency: 1 }), children: "Emergency On" }, "btnEmergencyOn") : null, highLevelStatus.Emergency ?
                        _jsx(AsyncButton, { danger: true, icon: _jsx(WarningOutlined, {}), onAsyncClick: mowerAction("emergency", { Emergency: 0 }), children: "Emergency Off" }, "btnEmergencyOff") : null] }), _jsx(Col, { children: _jsx(AsyncDropDownButton, { style: { display: "inline" }, menu: {
                        items: actionMenuItems,
                        onAsyncClick: async (e) => {
                            const item = actionMenuItems.find(item => item.key == e.key);
                            for (const action of (item?.actions ?? [])) {
                                await mowerAction(action.command, action.args)();
                            }
                        }
                    }, children: "More" }, "drpActions") })] }));
    if (props.bare) {
        return content;
    }
    return _jsx(ActionsCard, { title: "Actions", size: "small", children: content });
};
