import {createSchemaField} from "@formily/react";
import {Form, FormButtonGroup, FormItem, FormLayout, Input, NumberPicker, Select, Switch} from "@formily/antd-v5";
import {useApi, useThemeMode} from "@mowglinext/common";
import {App, Card, Col, Row} from "antd";
import React, {CSSProperties, useEffect, useMemo} from "react";
import type {Form as FormType} from "@formily/core";
import {createForm, onFieldValueChange} from "@formily/core";

import {SettingsConfig, SettingsDesc, SettingValueType, useSettings} from "../hooks/useSettings";
import {useIsMobile} from "../hooks/useIsMobile";

const SchemaField = createSchemaField({
    components: {
        Input,
        FormItem,
        Select,
        Switch,
        NumberPicker,
    },
})
export const SettingsComponent: React.FC<{
    actions?: (form: FormType<SettingsConfig>, save: (values: SettingsConfig) => Promise<void>, restartOM: () => Promise<void>, restartGUI: () => Promise<void>) => React.ReactElement[],
    contentStyle?: CSSProperties
}> = (props) => {
    const {colors} = useThemeMode();
    const form = useMemo(
        () =>
            createForm<SettingsConfig>({
                validateFirst: true,
                effects: (form) => {
                    onFieldValueChange('system.mqtt.enabled', (field) => {
                        form.setFieldState('*(system.mqtt.host,system.mqtt.prefix)', (state) => {
                            //For the initial linkage, if the field cannot be found, setFieldState will push the update into the update queue until the field appears before performing the operation
                            state.display = field.value ? "visible" : "hidden";
                        })
                    })
                    onFieldValueChange('system.homekit.enabled', (field) => {
                        form.setFieldState('*(system.homekit.pincode)', (state) => {
                            //For the initial linkage, if the field cannot be found, setFieldState will push the update into the update queue until the field appears before performing the operation
                            state.display = field.value ? "visible" : "hidden";
                        })
                    })
                    onFieldValueChange('system.map.enabled', (field) => {
                        form.setFieldState('*(system.map.tileServer,system.map.tileUri)', (state) => {
                            //For the initial linkage, if the field cannot be found, setFieldState will push the update into the update queue until the field appears before performing the operation
                            state.display = field.value ? "visible" : "hidden";
                        })
                    })
                },
            }),
        [],
    )

    const guiApi = useApi()
    const {notification} = App.useApp();
    const isMobile = useIsMobile();
    const {settings, setSettings, loading} = useSettings()
    useEffect(() => {
        if (settings && Object.keys(settings).length > 0) {
            form.setInitialValues(settings)
        }
    }, [settings]);
    useEffect(() => {
        form.setLoading(loading)
    }, [loading]);

    const restartOpenMower = async () => {
        try {
            const resContainersList = await guiApi.containers.containersList()
            if (resContainersList.error) {
                throw new Error(resContainersList.error.error)
            }
            const openMowerContainer = resContainersList.data.containers?.find((container) => container.labels?.app == "openmower" || container.names?.includes("/openmower"))
            if (openMowerContainer?.id) {
                const res = await guiApi.containers.containersCreate(openMowerContainer.id, "restart")
                if (res.error) {
                    throw new Error(res.error.error)
                }
                notification.success({
                    message: "OpenMower restarted",
                })
            } else {
                throw new Error("OpenMower container not found")
            }
        } catch (e: any) {
            notification.error({
                message: "Failed to restart OpenMower",
                description: e.message,
            })
        }
    }

    const restartGui = async () => {
        try {
            const resContainersList = await guiApi.containers.containersList()
            if (resContainersList.error) {
                throw new Error(resContainersList.error.error)
            }
            const openMowerContainer = resContainersList.data.containers?.find((container) => container.labels?.app == "gui" || container.names?.includes("/mowglinext"))
            if (openMowerContainer?.id) {
                const res = await guiApi.containers.containersCreate(openMowerContainer.id, "restart")
                if (res.error) {
                    throw new Error(res.error.error)
                }
                notification.success({
                    message: "OpenMower restarted",
                })
            } else {
                throw new Error("OpenMower container not found")
            }
        } catch (e: any) {
            notification.error({
                message: "Failed to restart OpenMower",
                description: e.message,
            })
        }
    }

    const sections = Object.keys(SettingsDesc).reduce((acc, key) => {
        const setting = SettingsDesc[key];
        if (!acc[setting.section]) {
            acc[setting.section] = []
        }
        acc[setting.section].push(key)
        return acc
    }, {} as Record<string, string[]>)

    return <Row>
        <Form form={form} style={{width: '100%'}}>
            <Col span={24} style={{...(!isMobile ? {height: '80vh', overflowY: 'auto' as const} : {paddingBottom: 80}), ...(!isMobile ? props.contentStyle : undefined)}}>
                <FormLayout layout="vertical">
                    {
                        Object.keys(sections).map(section => {
                            return (
                                <Card key={section} title={section} style={{marginBottom: 16}}>
                                    {
                                        sections[section].map(settingKey => {
                                            const setting = SettingsDesc[settingKey];
                                            switch (setting.type) {
                                                case SettingValueType.Lat:
                                                    return (
                                                        <SchemaField key={settingKey}><SchemaField.Number
                                                            name={settingKey}
                                                            title={setting.description}
                                                            default={setting.defaultValue}
                                                            x-component-props={{step: 0.000000001}}
                                                            x-component="NumberPicker"
                                                            x-decorator-props={{tooltip: setting.help}}
                                                            x-decorator="FormItem"/></SchemaField>)
                                                case SettingValueType.Lon:
                                                    return (
                                                        <SchemaField key={settingKey}><SchemaField.Number
                                                            name={settingKey}
                                                            title={setting.description}
                                                            default={setting.defaultValue}
                                                            x-component-props={{step: 0.000000001}}
                                                            x-component="NumberPicker"
                                                            x-decorator-props={{tooltip: setting.help}}
                                                            x-decorator="FormItem"/></SchemaField>)
                                                case SettingValueType.Boolean:
                                                    return (
                                                        <SchemaField key={settingKey}><SchemaField.Boolean
                                                            name={settingKey} title={setting.description}
                                                            default={setting.defaultValue} x-component="Switch"
                                                            x-decorator-props={{tooltip: setting.help}}
                                                            x-decorator="FormItem"/></SchemaField>)
                                                case SettingValueType.Float:
                                                    return (
                                                        <SchemaField key={settingKey}><SchemaField.Number
                                                            name={settingKey} title={setting.description}
                                                            default={setting.defaultValue}
                                                            x-component-props={{step: 0.01}}
                                                            x-decorator-props={{tooltip: setting.help}}
                                                            x-component="NumberPicker"
                                                            x-decorator="FormItem"/></SchemaField>)
                                                case SettingValueType.Int:
                                                    return (
                                                        <SchemaField key={settingKey}><SchemaField.Number
                                                            name={settingKey} title={setting.description}
                                                            default={setting.defaultValue}
                                                            x-component-props={{step: 1}}
                                                            x-decorator-props={{tooltip: setting.help}}
                                                            x-component="NumberPicker"
                                                            x-decorator="FormItem"/></SchemaField>)
                                                case SettingValueType.Select:
                                                    return (
                                                        <SchemaField key={settingKey}><SchemaField.String
                                                            name={settingKey}
                                                            title={setting.description}
                                                            default={setting.defaultValue}
                                                            enum={setting.options.map(opt => ({
                                                                label: opt.label,
                                                                value: opt.id
                                                            }))} x-component="Select"
                                                            x-decorator-props={{tooltip: setting.help}}
                                                            x-decorator="FormItem"/></SchemaField>)
                                                case SettingValueType.String:
                                                    return (
                                                        <SchemaField key={settingKey}><SchemaField.String
                                                            name={settingKey}
                                                            title={setting.description}
                                                            default={setting.defaultValue}
                                                            x-component="Input"
                                                            x-decorator-props={{tooltip: setting.help}}
                                                            x-decorator="FormItem"/></SchemaField>)

                                            }
                                        })
                                    }
                                </Card>)
                        })
                    }
                </FormLayout>
            </Col>
            <Col span={24} style={{
                position: "fixed",
                bottom: isMobile ? 'calc(56px + env(safe-area-inset-bottom, 0px))' : 20,
                left: isMobile ? 0 : undefined,
                right: isMobile ? 0 : undefined,
                padding: isMobile ? '8px 12px' : undefined,
                background: isMobile ? colors.bgCard : undefined,
                borderTop: isMobile ? `1px solid ${colors.border}` : undefined,
                zIndex: 50,
            }}>
                <FormButtonGroup.FormItem>
                    {props.actions && props.actions(form, setSettings, restartOpenMower, restartGui)}
                </FormButtonGroup.FormItem>
            </Col>
        </Form>
    </Row>
}
