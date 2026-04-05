import {App, Select, Space} from "antd";
import {useEffect, useState} from "react";
import Terminal, {ColorMode, TerminalOutput} from "react-terminal-ui";
import {AsyncButton, useWS, useApi, useThemeMode} from "@mowglinext/common";
import {StyledTerminal} from "../components/StyledTerminal";
import ansiHTML from "../utils/ansi";
import {useIsMobile} from "../hooks/useIsMobile";

type ContainerList = { value: string, label: string, status: "started" | "stopped", labels: Record<string, string> };
export const LogsPage = () => {
    const {colors} = useThemeMode();
    const guiApi = useApi();
    const {notification} = App.useApp();
    const isMobile = useIsMobile();
    const [containers, setContainers] = useState<ContainerList[]>([]);
    const [containerId, setContainerId] = useState<string | undefined>(undefined);
    const [data, setData] = useState<string[]>([])
    const stream = useWS<string>(() => {
        notification.error({
            message: "Logs stream closed",
        });
    }, () => {
        console.log({
            message: "Logs stream connected",
        })
    }, (e, first) => {
        setData((data) => {
            if (first) {
                return [e];
            }
            return [...data, e]
        })
    });

    async function listContainers() {
        try {
            const containers = await guiApi.containers.containersList();
            if (containers.error) {
                throw new Error(containers.error.error)
            }
            let options = containers.data.containers?.flatMap<ContainerList>((container) => {
                if (!container.names || !container.id) {
                    return [];
                }
                return [{
                    label: container.labels?.app ? `${container.labels.app} ( ${container.names[0].replace("/", "")} )` : container.names[0].replace("/", ""),
                    value: container.id,
                    status: container.state == "running" ? "started" : "stopped",
                    labels: container.labels ?? {}
                }]
            });
            setContainers(options ?? []);
            if (!!options?.length && !containerId) {
                setContainerId(options[0].value)
            }
        } catch (e: any) {
            notification.error({
                message: "Failed to list containers",
                description: e.message
            })
        }
    }

    useEffect(() => {
        (async () => {
            await listContainers();
        })();
    }, [])

    useEffect(() => {
        if (containerId) {
            stream.start(`/api/containers/${containerId}/logs`);
            return () => {
                stream?.stop();
            }
        }
    }, [containerId])
    const commandContainer = (command: "start" | "stop" | "restart") => async () => {
        const messages = {
            "start": "Container started",
            "stop": "Container stopped",
            "restart": "Container restarted"
        };
        try {
            if (containerId) {
                const res = await guiApi.containers.containersCreate(containerId, command);
                if (res.error) {
                    throw new Error(res.error.error)
                }
                if (command === "start" || command === "restart") {
                    stream.start(`/api/containers/${containerId}/logs`);
                } else {
                    stream?.stop();
                }
                await listContainers();
                notification.success({
                    message: messages[command],
                })
            }
        } catch (e: any) {
            notification.error({
                message: `Failed to ${command} container`,
                description: e.message
            })
        }
    };

    const selectedContainer = containers.find((container) => container.value === containerId);

    return (
        <div style={{
            display: 'flex',
            flexDirection: 'column',
            gap: 12,
            height: '100%',
        }}>
            {/* Controls bar */}
            <div style={{
                display: 'flex',
                flexDirection: isMobile ? 'column' : 'row',
                gap: 8,
                alignItems: isMobile ? 'stretch' : 'center',
                background: colors.bgCard,
                borderRadius: 12,
                padding: 12,
                flexShrink: 0,
            }}>
                <Select<string>
                    options={containers}
                    value={containerId}
                    style={{flex: 1, minWidth: isMobile ? undefined : 200}}
                    onSelect={(value) => setContainerId(value)}
                    placeholder="Select container"
                />
                <Space size={8} style={{flexShrink: 0}}>
                    {selectedContainer && selectedContainer.status === "started" && (
                        <>
                            <AsyncButton onAsyncClick={commandContainer("restart")} size={isMobile ? "middle" : "small"}>
                                Restart
                            </AsyncButton>
                            <AsyncButton
                                disabled={selectedContainer.labels.app == "gui"}
                                onAsyncClick={commandContainer("stop")}
                                size={isMobile ? "middle" : "small"}
                            >
                                Stop
                            </AsyncButton>
                        </>
                    )}
                    {selectedContainer && selectedContainer.status === "stopped" && (
                        <AsyncButton onAsyncClick={commandContainer("start")} size={isMobile ? "middle" : "small"}>
                            Start
                        </AsyncButton>
                    )}
                </Space>
            </div>

            {/* Terminal */}
            <div style={{flex: 1, minHeight: 0, overflow: 'hidden', borderRadius: 12}}>
                <StyledTerminal style={{height: '100%'}}>
                    <Terminal colorMode={ColorMode.Dark}>
                        {data.map((line, index) => {
                            return <TerminalOutput key={index}>
                                <div dangerouslySetInnerHTML={{__html: ansiHTML(line)}}></div>
                            </TerminalOutput>
                        })}
                    </Terminal>
                </StyledTerminal>
            </div>
        </div>
    );
}

export default LogsPage;
