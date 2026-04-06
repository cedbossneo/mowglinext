import { useState, useCallback, type ReactNode } from "react";
import { ConnectionProvider, type ConnectionState } from "@mowglinext/common";

function getDefaultConnection(): ConnectionState {
    const isDev = import.meta.env?.DEV;
    const protocol = window.location.protocol;
    const wsProtocol = protocol === "https:" ? "wss:" : "ws:";
    const host = isDev ? "localhost:4006" : window.location.host;
    return {
        baseHttpUrl: "",  // relative URLs for web (same origin)
        baseWsUrl: `${wsProtocol}//${host}`,
        authToken: null,
        isConnected: true,
        robotId: null,
    };
}

export function WebConnectionProvider({ children }: { children: ReactNode }) {
    const [connection, setConnectionState] = useState<ConnectionState>(getDefaultConnection);

    const setConnection = useCallback((partial: Partial<ConnectionState>) => {
        setConnectionState(prev => ({ ...prev, ...partial }));
    }, []);

    return (
        <ConnectionProvider value={{ ...connection, setConnection }}>
            {children}
        </ConnectionProvider>
    );
}
