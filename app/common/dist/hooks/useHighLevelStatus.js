import { useEffect, useState } from "react";
import { useWS } from "./useWS";
export const useHighLevelStatus = () => {
    const [highLevelStatus, setHighLevelStatus] = useState({});
    const highLevelStatusStream = useWS(() => {
        console.log({
            message: "High Level Status Stream closed",
        });
    }, () => {
        console.log({
            message: "High Level Status Stream connected",
        });
    }, (e) => {
        setHighLevelStatus(JSON.parse(e));
    });
    useEffect(() => {
        highLevelStatusStream.start("/api/openmower/subscribe/highLevelStatus");
        return () => {
            highLevelStatusStream.stop();
        };
    }, []);
    return { highLevelStatus, stop: highLevelStatusStream.stop, start: highLevelStatusStream.start };
};
