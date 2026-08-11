import React from "react";
import {Card, Radio, Space, Typography} from "antd";
import {useTranslation} from "react-i18next";
import {useThemeMode} from "../../theme/ThemeContext.tsx";
import type {DisplayMode} from "../../theme/ThemeContext.tsx";

const {Paragraph, Text} = Typography;

const DISPLAY_MODE_OPTIONS: DisplayMode[] = ['visual', 'balanced', 'efficient'];

export const DisplayModeSection: React.FC = () => {
    const {t} = useTranslation();
    const {colors, displayMode, setDisplayMode} = useThemeMode();

    return (
        <Card size="small">
            <Space direction="vertical" size={14} style={{width: "100%"}}>
                <div>
                    <Text strong style={{fontSize: 14}}>{t("displayMode.title")}</Text>
                    <Paragraph type="secondary" style={{margin: "4px 0 0"}}>
                        {t("displayMode.description")}
                    </Paragraph>
                </div>

                <Radio.Group
                    value={displayMode}
                    onChange={(event) => setDisplayMode(event.target.value as DisplayMode)}
                    aria-label={t("displayMode.title")}
                    style={{display: "grid", gridTemplateColumns: "repeat(auto-fit, minmax(190px, 1fr))", gap: 10}}
                >
                    {DISPLAY_MODE_OPTIONS.map((mode) => {
                        const selected = displayMode === mode;
                        return (
                            <Radio
                                key={mode}
                                value={mode}
                                style={{
                                    alignItems: "flex-start",
                                    minHeight: 76,
                                    marginInlineEnd: 0,
                                    padding: "12px 14px",
                                    borderRadius: 12,
                                    border: `1px solid ${selected ? colors.accent : colors.border}`,
                                    background: selected ? colors.primaryBg : colors.bgElevated,
                                }}
                            >
                                <span style={{display: "inline-flex", flexDirection: "column", gap: 4, paddingLeft: 4}}>
                                    <Text strong style={{color: colors.text}}>{t(`displayMode.${mode}.label`)}</Text>
                                    <Text type="secondary" style={{fontSize: 12, lineHeight: 1.45}}>
                                        {t(`displayMode.${mode}.description`)}
                                    </Text>
                                </span>
                            </Radio>
                        );
                    })}
                </Radio.Group>

                <Text type="secondary" style={{fontSize: 12}}>
                    {t("displayMode.reducedMotion")}
                </Text>
            </Space>
        </Card>
    );
};
