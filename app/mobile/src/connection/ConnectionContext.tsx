import { useState, useCallback, useEffect, type ReactNode } from "react";
import { ConnectionProvider, type ConnectionState } from "@mowglinext/common";
import { getSavedConnection } from "../native/preferences";

export function MobileConnectionProvider({ children }: { children: ReactNode }) {
  const [connection, setConnectionState] = useState<ConnectionState>({
    baseHttpUrl: "",
    baseWsUrl: "",
    authToken: null,
    isConnected: false,
    robotId: null,
  });

  useEffect(() => {
    getSavedConnection().then((saved) => {
      if (saved) {
        setConnectionState(saved);
      }
    });
  }, []);

  const setConnection = useCallback((partial: Partial<ConnectionState>) => {
    setConnectionState((prev) => ({ ...prev, ...partial }));
  }, []);

  return (
    <ConnectionProvider value={{ ...connection, setConnection }}>
      {children}
    </ConnectionProvider>
  );
}
