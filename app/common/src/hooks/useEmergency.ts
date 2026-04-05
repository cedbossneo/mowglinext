import {useEffect, useState} from "react";
import {Emergency} from "../types/ros";
import {useWS} from "./useWS";

export const useEmergency = () => {
    const [emergency, setEmergency] = useState<Emergency>({})
    const emergencyStream = useWS<string>(() => {
            console.log({
                message: "Emergency Stream closed",
            })
        }, () => {
            console.log({
                message: "Emergency Stream connected",
            })
        },
        (e) => {
            setEmergency(JSON.parse(e))
        })
    useEffect(() => {
        emergencyStream.start("/api/openmower/subscribe/emergency",)
        return () => {
            emergencyStream.stop()
        }
    }, []);
    return emergency;
};
