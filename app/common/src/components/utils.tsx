import {CheckCircleTwoTone, CloseCircleTwoTone} from "@ant-design/icons";
import {Progress} from "antd";
import {COLORS} from "../theme/colors";

export const booleanFormatter = (value: any) => (value === "On" || value === "Yes") ?
    <CheckCircleTwoTone twoToneColor={COLORS.primary}/> : <CloseCircleTwoTone
        twoToneColor={COLORS.danger}/>;
export const booleanFormatterInverted = (value: any) => (value === "On" || value === "Yes") ?
    <CheckCircleTwoTone twoToneColor={COLORS.danger}/> : <CloseCircleTwoTone
        twoToneColor={COLORS.primary}/>;
export const stateRenderer = (value: string | undefined) => {
    switch (value) {
        case "IDLE":
            return "Idle"
        case "MOWING":
            return "Mowing"
        case "DOCKING":
            return "Docking"
        case "UNDOCKING":
            return "Undocking"
        case "AREA_RECORDING":
            return "Area Recording"
        case "CHARGING":
            return "Charging"
        case "EMERGENCY":
            return "Emergency"
        default:
            return value ?? "Offline"
    }
};
export const progressFormatter = (value: any) => {
    return <Progress steps={3} percent={value} size={25} showInfo={false} strokeColor={COLORS.primary}/>
};

export const progressFormatterSmall = (value: any) => {
    return <Progress steps={3} percent={value} size={11} showInfo={false} strokeColor={COLORS.primary}/>
};
