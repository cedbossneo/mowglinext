import { useEffect, useState } from "react";
import { useWS } from "./useWS";
export const useGPS = () => {
    const [gps, setGps] = useState({});
    const gpsStream = useWS(() => {
        console.log({
            message: "GPS Stream closed",
        });
    }, () => {
        console.log({
            message: "GPS Stream connected",
        });
    }, (e) => {
        setGps(JSON.parse(e));
    });
    useEffect(() => {
        gpsStream.start("/api/openmower/subscribe/gps");
        return () => {
            gpsStream.stop();
        };
    }, []);
    return gps;
};
