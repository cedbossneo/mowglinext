import {useEffect, useState} from "react";
import {Power} from "../types/ros";
import {useWS} from "./useWS";

export const usePower = () => {
    const [power, setPower] = useState<Power>({})
    const powerStream = useWS<string>(() => {
            console.log({
                message: "Power Stream closed",
            })
        }, () => {
            console.log({
                message: "Power Stream connected",
            })
        },
        (e) => {
            setPower(JSON.parse(e))
        })
    useEffect(() => {
        powerStream.start("/api/openmower/subscribe/power",)
        return () => {
            powerStream.stop()
        }
    }, []);
    return power;
};
