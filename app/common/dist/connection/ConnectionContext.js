import { createContext, useContext } from "react";
const ConnectionContext = createContext({
    baseHttpUrl: "",
    baseWsUrl: "",
    authToken: null,
    isConnected: false,
    robotId: null,
    setConnection: () => { },
});
export const ConnectionProvider = ConnectionContext.Provider;
export const useConnection = () => useContext(ConnectionContext);
