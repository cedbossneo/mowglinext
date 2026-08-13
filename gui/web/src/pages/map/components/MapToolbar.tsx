import {App, Button, Dropdown, Space} from "antd";
import type {MenuProps} from "antd";
import type {MenuItemType} from "antd/es/menu/interface";
import {
    GlobalOutlined,
    EllipsisOutlined,
    EditOutlined,
    DatabaseOutlined,
    DownloadOutlined,
    ControlOutlined,
    PlayCircleOutlined,
    HomeOutlined,
    WarningOutlined,
    ScissorOutlined,
    AimOutlined,
    ForwardOutlined,
    CaretRightOutlined,
    PauseOutlined,
    ThunderboltOutlined,
    CheckOutlined,
    CloseOutlined,
    ImportOutlined,
} from "@ant-design/icons";
import type {MenuInfo} from "rc-menu/lib/interface";
import {useTranslation} from "react-i18next";
import AsyncButton from "../../../components/AsyncButton.tsx";
import AsyncDropDownButton from "../../../components/AsyncDropDownButton.tsx";
import type {Feature} from "geojson";

interface MowingAreaItem extends MenuItemType {
    feat: Feature;
}

interface MapToolbarProps {
    useSatellite: boolean;
    mowingAreas: MowingAreaItem[];
    stateName?: string;
    emergency?: boolean;
    onEditMap: () => void;
    onToggleSatellite: () => void;
    onOpenManualController: () => void;
    onBackupMap: () => void;
    onRestoreMap: () => void;
    onDownloadGeoJSON: () => void;
    onImportOpenMower: () => void;
    onMowArea: (key: string) => Promise<void>;
    pitched?: boolean;
    onTogglePitch?: () => void;
    onStart?: () => Promise<void>;
    onHome?: () => Promise<void>;
    onEmergencyOn?: () => Promise<void>;
    onEmergencyOff?: () => Promise<void>;
    onAreaRecording?: () => Promise<void>;
    onMowNextArea?: () => Promise<void>;
    onContinueOrPause?: () => Promise<void>;
    onBladeForward?: () => Promise<void>;
    onBladeBackward?: () => Promise<void>;
    onBladeOff?: () => Promise<void>;
    onRecordFinish?: () => Promise<void>;
    onRecordCancel?: () => Promise<void>;
}

export const MapToolbar = ({
    useSatellite, mowingAreas, stateName, emergency,
    onEditMap, onToggleSatellite,
    onOpenManualController,
    onBackupMap, onRestoreMap, onDownloadGeoJSON, onImportOpenMower,
    onMowArea, pitched, onTogglePitch,
    onStart, onHome, onEmergencyOn, onEmergencyOff,
    onAreaRecording, onMowNextArea, onContinueOrPause,
    onBladeForward, onBladeBackward, onBladeOff,
    onRecordFinish, onRecordCancel,
}: MapToolbarProps) => {
    const {notification} = App.useApp();
    const {t} = useTranslation();
    const isIdle = stateName === "IDLE" || stateName === "IDLE_DOCKED";
    const isRecording = stateName === "RECORDING";

    const safeCall = (fn?: () => Promise<void>) => {
        fn?.().catch((e: Error) => {
            console.error(e);
            notification.error({
                message: t("mapToolbar.actionFailed"),
                description: e.message,
            });
        });
    };

    const moreMenuItems: MenuProps["items"] = [
        {key: "satellite", icon: <GlobalOutlined />, label: useSatellite ? t("mapToolbar.darkMap") : t("mapToolbar.satellite")},
        ...(onTogglePitch
            ? [{key: "pitch", icon: <GlobalOutlined />, label: pitched ? t("mapToolbar.flattenMap") : t("mapToolbar.tilt3dView")} satisfies NonNullable<MenuProps["items"]>[number]]
            : []),
        {type: "divider"},
        {key: "areaRecording", icon: <AimOutlined />, label: t("mapToolbar.areaRecording")},
        {key: "mowNext", icon: <ForwardOutlined />, label: t("mapToolbar.mowNextArea")},
        {key: "continueOrPause", icon: isIdle ? <CaretRightOutlined /> : <PauseOutlined />, label: isIdle ? t("mapToolbar.continue") : t("mapToolbar.pause")},
        {type: "divider"},
        {key: "manual", icon: <ControlOutlined />, label: t("mapToolbar.manualMowing")},
        {type: "divider"},
        {key: "bladeForward", icon: <ThunderboltOutlined />, label: t("mapToolbar.bladeForward")},
        {key: "bladeBackward", icon: <ThunderboltOutlined />, label: t("mapToolbar.bladeBackward")},
        {key: "bladeOff", icon: <ThunderboltOutlined />, label: t("mapToolbar.bladeOff"), danger: true},
        {type: "divider"},
        {key: "backup", icon: <DatabaseOutlined />, label: t("mapToolbar.backupMap")},
        {key: "restore", icon: <DatabaseOutlined />, label: t("mapToolbar.restoreMap")},
        {key: "importOpenMower", icon: <ImportOutlined />, label: t("mapToolbar.importFromOpenMower")},
        {type: "divider"},
        {key: "download", icon: <DownloadOutlined />, label: t("mapToolbar.downloadGeojson")},
    ];

    const handleMoreClick: MenuProps["onClick"] = ({key}: MenuInfo) => {
        switch (key) {
            case "satellite": onToggleSatellite(); break;
            case "pitch": onTogglePitch?.(); break;
            case "manual": onOpenManualController(); break;
            case "areaRecording": safeCall(onAreaRecording); break;
            case "mowNext": safeCall(onMowNextArea); break;
            case "continueOrPause": safeCall(onContinueOrPause); break;
            case "bladeForward": safeCall(onBladeForward); break;
            case "bladeBackward": safeCall(onBladeBackward); break;
            case "bladeOff": safeCall(onBladeOff); break;
            case "backup": onBackupMap(); break;
            case "restore": onRestoreMap(); break;
            case "importOpenMower": onImportOpenMower(); break;
            case "download": onDownloadGeoJSON(); break;
        }
    };

    return (
        <Space size="small" wrap>
            <Button
                type="primary"
                icon={<EditOutlined />}
                onClick={onEditMap}
            >
                {t("mapToolbar.editMap")}
            </Button>

            {isRecording ? (
                <>
                    <AsyncButton
                        type="primary"
                        icon={<CheckOutlined />}
                        onAsyncClick={onRecordFinish!}
                    >
                        {t("mapToolbar.finishRecording")}
                    </AsyncButton>
                    <AsyncButton
                        danger
                        icon={<CloseOutlined />}
                        onAsyncClick={onRecordCancel!}
                    >
                        {t("mapToolbar.cancelRecording")}
                    </AsyncButton>
                </>
            ) : (
                <>
                    {isIdle && (
                        <AsyncButton
                            type="primary"
                            icon={<PlayCircleOutlined />}
                            onAsyncClick={onStart!}
                        >
                            {t("mapToolbar.start")}
                        </AsyncButton>
                    )}
                    {/* Home (return-to-dock) is always available outside recording
                        so the robot can be sent back even while idle off-dock. */}
                    <AsyncButton
                        type={isIdle ? "default" : "primary"}
                        icon={<HomeOutlined />}
                        onAsyncClick={onHome!}
                    >
                        {t("mapToolbar.home")}
                    </AsyncButton>
                </>
            )}

            {!emergency ? (
                <AsyncButton
                    danger
                    icon={<WarningOutlined />}
                    onAsyncClick={onEmergencyOn!}
                >
                    {t("mapToolbar.emergencyOn")}
                </AsyncButton>
            ) : (
                <AsyncButton
                    danger
                    icon={<WarningOutlined />}
                    onAsyncClick={onEmergencyOff!}
                >
                    {t("mapToolbar.emergencyOff")}
                </AsyncButton>
            )}

            <AsyncDropDownButton
                icon={<ScissorOutlined />}
                menu={{
                    items: mowingAreas,
                    onAsyncClick: (e: MenuInfo) => onMowArea(e.key),
                }}
            >
                {t("mapToolbar.mowArea")}
            </AsyncDropDownButton>

            <Button icon={<ControlOutlined />} onClick={onOpenManualController}>
                {t("mapToolbar.manualMow")}
            </Button>

            <Dropdown
                menu={{items: moreMenuItems, onClick: handleMoreClick}}
                trigger={["click"]}
            >
                <Button icon={<EllipsisOutlined />}>{t("mapToolbar.more")}</Button>
            </Dropdown>
        </Space>
    );
};
