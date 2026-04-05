import { useEffect, useState } from "react";
import { useWS } from "./useWS";
export const usePower = () => {
    const [power, setPower] = useState({});
    const powerStream = useWS(() => {
        console.log({
            message: "Power Stream closed",
        });
    }, () => {
        console.log({
            message: "Power Stream connected",
        });
    }, (e) => {
        setPower(JSON.parse(e));
    });
    useEffect(() => {
        powerStream.start("/api/openmower/subscribe/power");
        return () => {
            powerStream.stop();
        };
    }, []);
    return power;
};
