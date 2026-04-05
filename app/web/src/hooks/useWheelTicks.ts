import {useEffect, useState} from "react";
import {WheelTick, useWS} from "@mowglinext/common";

export const useWheelTicks = () => {
    const [wheelTicks, setWheelTicks] = useState<WheelTick>({})
    const ticksStream = useWS<string>(() => {
            console.log({
                message: "Wheel Ticks Stream closed",
            })
        }, () => {
            console.log({
                message: "Wheel Ticks Stream connected",
            })
        },
        (e) => {
            setWheelTicks(JSON.parse(e))
        })
    useEffect(() => {
        ticksStream.start("/api/openmower/subscribe/ticks",)
        return () => {
            ticksStream.stop()
        }
    }, []);
    return wheelTicks;
};
