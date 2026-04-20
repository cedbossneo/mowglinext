import { App } from "antd";
import { useApi } from "./useApi.ts";
import { useCallback, useEffect, useRef, useState } from "react";

export type BackendSettingsValues = {
    HARDWARE_BACKEND: "mowgli" | "mavros";
    MAVROS_AUTOPILOT: "ardupilot" | "px4";
};

const DEFAULT_VALUES: BackendSettingsValues = {
    HARDWARE_BACKEND: "mowgli",
    MAVROS_AUTOPILOT: "ardupilot",
};

const LOAD_ERROR_NOTIFICATION_KEY = "backend-settings-load-error";

export const useBackendSettings = () => {
    const guiApi = useApi();
    const { notification } = App.useApp();
    const [values, setValues] = useState<BackendSettingsValues>(DEFAULT_VALUES);
    const [loading, setLoading] = useState(false);
    const lastLoadErrorRef = useRef<string | null>(null);

    useEffect(() => {
        let cancelled = false;

        (async () => {
            try {
                setLoading(true);
                const res = await guiApi.settings.settingsList();
                if (res.error) {
                    throw new Error(res.error.error);
                }

                const settings = res.data.settings ?? {};
                const nextValues: BackendSettingsValues = {
                    HARDWARE_BACKEND:
                        settings.HARDWARE_BACKEND === "mavros" ? "mavros" : "mowgli",
                    MAVROS_AUTOPILOT:
                        settings.MAVROS_AUTOPILOT === "px4" ? "px4" : "ardupilot",
                };

                if (!cancelled) {
                    setValues(nextValues);
                    lastLoadErrorRef.current = null;
                }
            } catch (e: any) {
                const description = e.message ?? "Unknown error";
                if (!cancelled && lastLoadErrorRef.current !== description) {
                    notification.error({
                        key: LOAD_ERROR_NOTIFICATION_KEY,
                        message: "Failed to load backend settings",
                        description,
                    });
                    lastLoadErrorRef.current = description;
                }
            } finally {
                if (!cancelled) {
                    setLoading(false);
                }
            }
        })();

        return () => {
            cancelled = true;
        };
    }, []);

    const saveValues = useCallback(
        async (newValues: BackendSettingsValues, options?: { silent?: boolean }) => {
            try {
                setLoading(true);
                const payload = {
                    HARDWARE_BACKEND: newValues.HARDWARE_BACKEND,
                    MAVROS_AUTOPILOT: newValues.MAVROS_AUTOPILOT,
                };
                const res = await guiApi.settings.settingsCreate(payload);
                if (res.error) {
                    throw new Error(res.error.error);
                }
                setValues(newValues);
                if (!options?.silent) {
                    notification.success({
                        message: "Backend settings saved",
                        description: "Restart ROS2 to apply the new backend configuration.",
                    });
                }
            } catch (e: any) {
                notification.error({
                    message: "Failed to save backend settings",
                    description: e.message,
                });
                throw e;
            } finally {
                setLoading(false);
            }
        },
        [guiApi, notification]
    );

    return { values, setValues, saveValues, loading };
};
