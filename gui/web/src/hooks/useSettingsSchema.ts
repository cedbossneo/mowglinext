import { useApi } from "./useApi.ts";
import { App } from "antd";
import { useCallback, useEffect, useState } from "react";

export type JSONSchema = {
    type?: string;
    title?: string;
    description?: string;
    properties?: Record<string, JSONSchemaProperty>;
    allOf?: JSONSchemaCondition[];
    required?: string[];
};

export type JSONSchemaProperty = {
    type?: string;
    title?: string;
    description?: string;
    default?: any;
    enum?: any[];
    minimum?: number;
    maximum?: number;
    "x-environment-variable"?: string;
    "x-yaml-node"?: string;
    "x-remap-values"?: Record<string, any>;
    "x-section"?: string;
    properties?: Record<string, JSONSchemaProperty>;
    allOf?: JSONSchemaCondition[];
    additionalProperties?: JSONSchemaProperty | boolean;
};

export type JSONSchemaCondition = {
    if?: {
        properties?: Record<string, { const?: any; enum?: any[] }>;
    };
    then?: {
        properties?: Record<string, JSONSchemaProperty>;
        allOf?: JSONSchemaCondition[];
    };
    else?: {
        properties?: Record<string, JSONSchemaProperty>;
    };
};

export const useSettingsSchema = () => {
    const guiApi = useApi();
    const { notification } = App.useApp();
    const [schema, setSchema] = useState<JSONSchema | null>(null);
    const [values, setValues] = useState<Record<string, any>>({});
    const [loading, setLoading] = useState(false);

    useEffect(() => {
        (async () => {
            try {
                setLoading(true);
                const [schemaRes, valuesRes] = await Promise.all([
                    guiApi.settings.schemaList(),
                    guiApi.settings.yamlList(),
                ]);
                if (schemaRes.error) {
                    throw new Error((schemaRes.error as any).error);
                }
                setSchema(schemaRes.data as JSONSchema);
                if (!valuesRes.error) {
                    setValues((valuesRes.data as Record<string, any>) || {});
                }
            } catch (e: any) {
                notification.error({
                    message: "Failed to load settings schema",
                    description: e.message,
                });
            } finally {
                setLoading(false);
            }
        })();
    }, []);

    const saveValues = useCallback(
        async (newValues: Record<string, any>) => {
            try {
                setLoading(true);
                const res = await guiApi.settings.yamlCreate(newValues);
                if (res.error) {
                    throw new Error((res.error as any).error);
                }
                setValues(newValues);
                notification.success({ message: "Settings saved" });
            } catch (e: any) {
                notification.error({
                    message: "Failed to save settings",
                    description: e.message,
                });
            } finally {
                setLoading(false);
            }
        },
        [guiApi, notification]
    );

    return { schema, values, saveValues, loading };
};
