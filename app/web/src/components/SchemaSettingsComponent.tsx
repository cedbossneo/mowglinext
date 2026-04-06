import React, { CSSProperties, useCallback, useEffect, useState } from "react";
import { Card, Col, Row, Spin, Switch, Input, InputNumber, Select, Form, Button, Space, Typography } from "antd";
import { SaveOutlined, ReloadOutlined, PlusOutlined, DeleteOutlined } from "@ant-design/icons";
import { JSONSchemaProperty, JSONSchemaCondition, useSettingsSchema } from "../hooks/useSettingsSchema";
import { useIsMobile } from "../hooks/useIsMobile";
import { useThemeMode } from "@mowglinext/common";
import { RobotComponentEditor } from "./RobotComponentEditor";

type FieldRenderProps = {
    name: string;
    prop: JSONSchemaProperty;
    value: any;
    onChange: (name: string, value: any) => void;
};

/** Turn a snake_case key like "mowing_speed" into "Mowing Speed" */
const humanizeKey = (key: string): string => {
    return key
        .split("_")
        .map((w) => w.charAt(0).toUpperCase() + w.slice(1).toLowerCase())
        .join(" ");
};

const SchemaField: React.FC<FieldRenderProps> = ({ name, prop, value, onChange }) => {
    const title = prop.title || humanizeKey(name);
    const description = prop.description;

    if (prop.enum && prop.enum.length > 0) {
        return (
            <Form.Item label={title} tooltip={description}>
                <Select
                    value={value ?? prop.default}
                    onChange={(v) => onChange(name, v)}
                    options={prop.enum.map((e: any) => ({ label: String(e), value: e }))}
                />
            </Form.Item>
        );
    }

    switch (prop.type) {
        case "boolean":
            return (
                <Form.Item label={title} tooltip={description}>
                    <Switch
                        checked={value ?? prop.default ?? false}
                        onChange={(v) => onChange(name, v)}
                    />
                </Form.Item>
            );
        case "number":
        case "integer":
            return (
                <Form.Item label={title} tooltip={description}>
                    <InputNumber
                        value={value ?? prop.default}
                        onChange={(v) => onChange(name, v)}
                        step={prop.type === "integer" ? 1 : 0.01}
                        style={{ width: "100%" }}
                    />
                </Form.Item>
            );
        default:
            return (
                <Form.Item label={title} tooltip={description}>
                    <Input
                        value={value ?? prop.default ?? ""}
                        onChange={(e) => onChange(name, e.target.value)}
                    />
                </Form.Item>
            );
    }
};

type KeyValueEditorProps = {
    sectionKey: string;
    title: string;
    description?: string;
    entries: Record<string, string>;
    onEntriesChange: (entries: Record<string, string>) => void;
};

const KeyValueEditor: React.FC<KeyValueEditorProps> = ({ title, description, entries, onEntriesChange }) => {
    const [newKey, setNewKey] = useState("");
    const [newValue, setNewValue] = useState("");

    const handleAdd = () => {
        const key = newKey.trim();
        if (!key) return;
        onEntriesChange({ ...entries, [key]: newValue });
        setNewKey("");
        setNewValue("");
    };

    const handleRemove = (key: string) => {
        const updated = { ...entries };
        delete updated[key];
        onEntriesChange(updated);
    };

    const handleValueChange = (key: string, val: string) => {
        onEntriesChange({ ...entries, [key]: val });
    };

    return (
        <Card title={title} style={{ marginBottom: 16 }}>
            {description && (
                <Typography.Paragraph type="secondary">{description}</Typography.Paragraph>
            )}
            <Form layout="vertical">
                {Object.entries(entries).map(([key, val]) => (
                    <Form.Item key={key} label={key}>
                        <Space.Compact style={{ width: "100%" }}>
                            <Input
                                value={val}
                                onChange={(e) => handleValueChange(key, e.target.value)}
                                style={{ flex: 1 }}
                            />
                            <Button
                                danger
                                icon={<DeleteOutlined />}
                                onClick={() => handleRemove(key)}
                            />
                        </Space.Compact>
                    </Form.Item>
                ))}
                <Form.Item label="Add new variable">
                    <Space.Compact style={{ width: "100%" }}>
                        <Input
                            placeholder="KEY"
                            value={newKey}
                            onChange={(e) => setNewKey(e.target.value.toUpperCase())}
                            style={{ width: "40%" }}
                            onPressEnter={handleAdd}
                        />
                        <Input
                            placeholder="VALUE"
                            value={newValue}
                            onChange={(e) => setNewValue(e.target.value)}
                            style={{ flex: 1 }}
                            onPressEnter={handleAdd}
                        />
                        <Button
                            type="primary"
                            icon={<PlusOutlined />}
                            onClick={handleAdd}
                            disabled={!newKey.trim()}
                        >
                            Add
                        </Button>
                    </Space.Compact>
                </Form.Item>
            </Form>
        </Card>
    );
};

const resolveConditionalProperties = (
    allOf: JSONSchemaCondition[] | undefined,
    values: Record<string, any>,
    baseProps?: Record<string, JSONSchemaProperty>
): Record<string, JSONSchemaProperty> => {
    if (!allOf) return {};
    const result: Record<string, JSONSchemaProperty> = {};

    for (const condition of allOf) {
        if (!condition.if?.properties) continue;

        let matches = true;
        for (const [key, check] of Object.entries(condition.if.properties)) {
            // Fall back to schema default when the value hasn't been set yet
            const currentVal = values[key] ?? baseProps?.[key]?.default;
            if (check.const !== undefined && currentVal !== check.const) {
                matches = false;
                break;
            }
            if (check.enum && !check.enum.includes(currentVal)) {
                matches = false;
                break;
            }
        }

        if (matches) {
            if (condition.then?.properties) {
                for (const [key, prop] of Object.entries(condition.then.properties)) {
                    result[key] = prop;
                }
                // Recurse into nested allOf in then
                if ((condition.then as any).allOf) {
                    const nested = resolveConditionalProperties((condition.then as any).allOf, values);
                    Object.assign(result, nested);
                }
            }
        } else {
            // Handle else branch
            const elseBranch = (condition as any).else;
            if (elseBranch?.properties) {
                for (const [key, prop] of Object.entries(elseBranch.properties)) {
                    result[key] = prop as JSONSchemaProperty;
                }
            }
        }
    }

    return result;
};

type SectionProps = {
    sectionKey: string;
    section: JSONSchemaProperty;
    values: Record<string, any>;
    onChange: (name: string, value: any) => void;
    onCustomEnvChange?: (entries: Record<string, string>) => void;
};

const SchemaSection: React.FC<SectionProps> = ({ sectionKey, section, values, onChange, onCustomEnvChange }) => {
    // Handle additionalProperties sections (key-value editor)
    if ((section as any).additionalProperties && !section.properties) {
        const entries: Record<string, string> = {};
        const customEnv = values["__custom_environment"] as Record<string, string> | undefined;
        if (customEnv) {
            Object.assign(entries, customEnv);
        }

        return (
            <KeyValueEditor
                sectionKey={sectionKey}
                title={section.title || sectionKey}
                description={section.description}
                entries={entries}
                onEntriesChange={(newEntries) => {
                    if (onCustomEnvChange) {
                        onCustomEnvChange(newEntries);
                    }
                }}
            />
        );
    }

    const baseProps = section.properties || {};
    const conditionalProps = resolveConditionalProperties(section.allOf, values, baseProps);
    const allProps = { ...baseProps, ...conditionalProps };

    return (
        <Card title={section.title || sectionKey} style={{ marginBottom: 16 }}>
            <Form layout="vertical">
                {Object.entries(allProps).map(([key, prop]) => {
                    if (prop.type === "object") {
                        return (
                            <SchemaSection
                                key={key}
                                sectionKey={key}
                                section={prop}
                                values={values}
                                onChange={onChange}
                                onCustomEnvChange={onCustomEnvChange}
                            />
                        );
                    }
                    return (
                        <SchemaField
                            key={key}
                            name={key}
                            prop={prop}
                            value={values[key]}
                            onChange={onChange}
                        />
                    );
                })}
            </Form>
        </Card>
    );
};

export const SchemaSettingsComponent: React.FC<{
    contentStyle?: CSSProperties;
    onRestartOM?: () => Promise<void>;
    onRestartGUI?: () => Promise<void>;
}> = (props) => {
    const isMobile = useIsMobile();
    const { colors } = useThemeMode();
    const { schema, values: savedValues, saveValues, loading } = useSettingsSchema();
    const [localValues, setLocalValues] = useState<Record<string, any>>({});

    useEffect(() => {
        if (savedValues) {
            setLocalValues(savedValues);
        }
    }, [savedValues]);

    const handleChange = useCallback((name: string, value: any) => {
        setLocalValues((prev) => ({ ...prev, [name]: value }));
    }, []);

    const handleSave = useCallback(async () => {
        await saveValues(localValues);
    }, [localValues, saveValues]);

    if (!schema) {
        return <Spin size="large" style={{ display: "block", margin: "100px auto" }} />;
    }

    const sections = schema.properties || {};

    return (
        <Row>
            <Col span={24} style={{ height: isMobile ? "auto" : "80vh", overflowY: isMobile ? undefined : "auto", paddingBottom: 80, ...props.contentStyle }}>
                {Object.entries(sections).map(([key, section]) => (
                    <React.Fragment key={key}>
                        {key === "sensor_settings" && (
                            <RobotComponentEditor values={localValues} onChange={handleChange} />
                        )}
                        <SchemaSection
                            sectionKey={key}
                            section={section as JSONSchemaProperty}
                            values={localValues}
                            onChange={handleChange}
                        />
                    </React.Fragment>
                ))}
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
                <Space wrap={isMobile} size={isMobile ? 8 : undefined}>
                    <Button
                        type="primary"
                        icon={<SaveOutlined />}
                        onClick={handleSave}
                        loading={loading}
                    >
                        Save Settings
                    </Button>
                    {props.onRestartOM && (
                        <Button icon={<ReloadOutlined />} onClick={props.onRestartOM}>
                            Restart OpenMower
                        </Button>
                    )}
                    {props.onRestartGUI && (
                        <Button icon={<ReloadOutlined />} onClick={props.onRestartGUI}>
                            Restart GUI
                        </Button>
                    )}
                </Space>
            </Col>
        </Row>
    );
};
