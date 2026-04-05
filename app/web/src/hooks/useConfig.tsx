import {useApi} from "@mowglinext/common";
import {App} from "antd";
import {useEffect, useState} from "react";

export const useConfig = (keys: string[]) => {
    const guiApi = useApi()
    const {notification} = App.useApp();
    const [config, setConfig] = useState<Record<string, any>>({})
    const handleSetConfig = async (newConfig: Record<string, any>) => {
        try {
            const offsetConfig = await guiApi.config.keysSetCreate(newConfig)
            if (offsetConfig.error) {
                throw new Error(offsetConfig.error.error ?? "")
            }
            setConfig(oldConfig => ({
                ...oldConfig,
                ...offsetConfig.data
            }))
        } catch (e: any) {
            notification.error({
                message: "Failed to save config",
                description: e.message,
            })
        }
    }
    useEffect(() => {
        (async () => {
            try {
                const keysMap: Record<string, any> = {}
                keys.forEach((key) => {
                    keysMap[key] = ""
                })
                const offsetConfig = await guiApi.config.keysGetCreate(keysMap)
                if (offsetConfig.error) {
                    throw new Error(offsetConfig.error.error ?? "")
                }
                setConfig(offsetConfig.data)
            } catch (e: any) {
                notification.error({
                    message: "Failed to load config",
                    description: e.message,
                })
            }
        })()
    }, [])
    return {config, setConfig: handleSetConfig}
}
