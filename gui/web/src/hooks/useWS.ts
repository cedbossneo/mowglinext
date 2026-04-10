import reactUseWebSocketModule from "react-use-websocket";
import {useRef, useState} from "react";

// Vite 8 CJS interop may wrap the default export differently at runtime
const useWebSocket = (reactUseWebSocketModule as unknown as { default: typeof reactUseWebSocketModule }).default ?? reactUseWebSocketModule;

export const useWS = <T>(onError: (e: Error) => void, onInfo: (msg: string) => void, onData: (data: T, first?: boolean) => void) => {
    const [uri, setUri] = useState<string | null>(null);
    const [first, setFirst] = useState(false)

    // Keep refs to always call the latest callbacks, avoiding stale closures
    const onDataRef = useRef(onData);
    onDataRef.current = onData;
    const onErrorRef = useRef(onError);
    onErrorRef.current = onError;
    const onInfoRef = useRef(onInfo);
    onInfoRef.current = onInfo;

    const ws = useWebSocket(uri, {
        share: true,
        shouldReconnect: () => true,
        reconnectAttempts: Infinity,
        reconnectInterval: (attempt: number) => Math.min(1000 * Math.pow(2, attempt), 30000),
        onOpen: () => {
            console.log(`Opened stream ${uri}`)
            onInfoRef.current("Stream connected")
        },
        onError: () => {
            console.log(`Error on stream ${uri}`)
            onErrorRef.current(new Error(`Stream error`))
        },
        onClose: () => {
            console.log(`Stream closed ${uri}`)
            onErrorRef.current(new Error(`Stream closed`))
        },
        onMessage: (e: MessageEvent) => {
            if (first) {
                setFirst(false)
            }
            onDataRef.current(atob(e.data) as T, first);
        }
    });
    const start = (uri: string) => {
        setFirst(true)
        const protocol = window.location.protocol === 'https:' ? 'wss' : 'ws';
        if (import.meta.env.DEV) {
            setUri(`${protocol}://localhost:4006${uri}`)
        } else {
            setUri(`${protocol}://${window.location.host}${uri}`)
        }
    };
    const stop = () => {
        console.log(`Closing stream ${ws.getWebSocket()?.url}`)
        setUri(null)
        setFirst(false)
    }
    return {start, stop, sendJsonMessage: ws.sendJsonMessage}
}
