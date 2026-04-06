import { useEffect, useState } from "react";
import { useWS } from "./useWS";
export const useEmergency = () => {
    const [emergency, setEmergency] = useState({});
    const emergencyStream = useWS(() => {
        console.log({
            message: "Emergency Stream closed",
        });
    }, () => {
        console.log({
            message: "Emergency Stream connected",
        });
    }, (e) => {
        setEmergency(JSON.parse(e));
    });
    useEffect(() => {
        emergencyStream.start("/api/openmower/subscribe/emergency");
        return () => {
            emergencyStream.stop();
        };
    }, []);
    return emergency;
};
