import {useApi} from "@mowglinext/common";
import {App} from "antd";
import {useEffect, useState} from "react";

export const useEnv = () => {
    const guiApi = useApi()
    const {notification} = App.useApp();
    const [env, setEnv] = useState<Record<string, any>>({})
    useEffect(() => {
        (async () => {
            try {
                const envs = await guiApi.config.envsList()
                if (envs.error) {
                    throw new Error(envs.error.error ?? "")
                }
                setEnv(envs.data)
            } catch (e: any) {
                notification.error({
                    message: "Failed to load config",
                    description: e.message,
                })
            }
        })()
    }, [])
    return env
}
