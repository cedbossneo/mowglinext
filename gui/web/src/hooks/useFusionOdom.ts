import {useEffect, useState} from "react";
import {useWS} from "./useWS.ts";

export interface FusionOdom {
    header?: { stamp?: { sec: number; nanosec: number }; frame_id?: string };
    child_frame_id?: string;
    pose?: {
        pose?: {
            position?: { x: number; y: number; z: number };
            orientation?: { x: number; y: number; z: number; w: number };
        };
        covariance?: number[];
    };
    twist?: {
        twist?: {
            linear?: { x: number; y: number; z: number };
            angular?: { x: number; y: number; z: number };
        };
    };
}

/**
 * Subscribes to the global filtered odometry published by
 * ekf_map_node (robot_localization dual-EKF). Before the 2026-04-24
 * migration this came from FusionCore on /fusion/odom; the topic
 * alias kept its name ("fusionRaw") on the backend to avoid churning
 * every consumer. The function name is kept as useFusionOdom for
 * the same reason — rename to useMapOdometry in a follow-up if we
 * want to retire the FusionCore branding entirely.
 */
export const useFusionOdom = () => {
    const [odom, setOdom] = useState<FusionOdom>({})
    const stream = useWS<string>(() => {
            console.log({ message: "MapOdometry Stream closed" })
        }, () => {
            console.log({ message: "MapOdometry Stream connected" })
        },
        (e) => {
            setOdom(JSON.parse(e))
        })
    useEffect(() => {
        stream.start("/api/mowglinext/subscribe/fusionRaw")
        return () => { stream.stop() }
    }, []);
    return odom;
};
