import { useEffect, useState } from "react";
import { useWS } from "./useWS";
export const usePose = () => {
    const [pose, setPose] = useState({});
    const poseStream = useWS(() => {
        console.log({
            message: "POSE Stream closed",
        });
    }, () => {
        console.log({
            message: "POSE Stream connected",
        });
    }, (e) => {
        setPose(JSON.parse(e));
    });
    useEffect(() => {
        poseStream.start("/api/openmower/subscribe/pose");
        return () => {
            poseStream.stop();
        };
    }, []);
    return pose;
};
