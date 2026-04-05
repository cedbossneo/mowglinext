import { createContext, useContext } from "react";

export interface ConnectionState {
  baseHttpUrl: string;      // "http://192.168.1.50" or "https://proxy.mowgli.garden"
  baseWsUrl: string;        // ws:// or wss:// derived
  authToken: string | null; // null for local, JWT for cloud
  isConnected: boolean;
  robotId: string | null;   // null for local, UUID for cloud
}

export interface ConnectionContextValue extends ConnectionState {
  setConnection: (state: Partial<ConnectionState>) => void;
}

const ConnectionContext = createContext<ConnectionContextValue>({
  baseHttpUrl: "",
  baseWsUrl: "",
  authToken: null,
  isConnected: false,
  robotId: null,
  setConnection: () => {},
});

export const ConnectionProvider = ConnectionContext.Provider;
export const useConnection = () => useContext(ConnectionContext);
