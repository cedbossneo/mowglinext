import { jsx as _jsx } from "react/jsx-runtime";
import { CheckCircleTwoTone, CloseCircleTwoTone } from "@ant-design/icons";
import { Progress } from "antd";
import { COLORS } from "../theme/colors";
export const booleanFormatter = (value) => (value === "On" || value === "Yes") ?
    _jsx(CheckCircleTwoTone, { twoToneColor: COLORS.primary }) : _jsx(CloseCircleTwoTone, { twoToneColor: COLORS.danger });
export const booleanFormatterInverted = (value) => (value === "On" || value === "Yes") ?
    _jsx(CheckCircleTwoTone, { twoToneColor: COLORS.danger }) : _jsx(CloseCircleTwoTone, { twoToneColor: COLORS.primary });
export const stateRenderer = (value) => {
    switch (value) {
        case "IDLE":
            return "Idle";
        case "MOWING":
            return "Mowing";
        case "DOCKING":
            return "Docking";
        case "UNDOCKING":
            return "Undocking";
        case "AREA_RECORDING":
            return "Area Recording";
        case "CHARGING":
            return "Charging";
        case "EMERGENCY":
            return "Emergency";
        default:
            return value ?? "Offline";
    }
};
export const progressFormatter = (value) => {
    return _jsx(Progress, { steps: 3, percent: value, size: 25, showInfo: false, strokeColor: COLORS.primary });
};
export const progressFormatterSmall = (value) => {
    return _jsx(Progress, { steps: 3, percent: value, size: 11, showInfo: false, strokeColor: COLORS.primary });
};
