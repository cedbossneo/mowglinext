import { useEffect, useState } from "react";
import { useWS } from "./useWS";
export const useStatus = () => {
    const [status, setStatus] = useState({});
    const statusStream = useWS(() => {
        console.log({
            message: "Status Stream closed",
        });
    }, () => {
        console.log({
            message: "Status Stream connected",
        });
    }, (e) => {
        setStatus(JSON.parse(e));
    });
    useEffect(() => {
        statusStream.start("/api/openmower/subscribe/status");
        return () => {
            statusStream.stop();
        };
    }, []);
    return status;
};
