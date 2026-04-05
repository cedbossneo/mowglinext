export declare const useWS: <T>(onError: (e: Error) => void, onInfo: (msg: string) => void, onData: (data: T, first?: boolean) => void) => {
    start: (path: string) => void;
    stop: () => void;
    sendJsonMessage: import("react-use-websocket/dist/lib/types").SendJsonMessage;
};
//# sourceMappingURL=useWS.d.ts.map