export interface ConnectionState {
    baseHttpUrl: string;
    baseWsUrl: string;
    authToken: string | null;
    isConnected: boolean;
    robotId: string | null;
}
export interface ConnectionContextValue extends ConnectionState {
    setConnection: (state: Partial<ConnectionState>) => void;
}
export declare const ConnectionProvider: import("react").Provider<ConnectionContextValue>;
export declare const useConnection: () => ConnectionContextValue;
//# sourceMappingURL=ConnectionContext.d.ts.map