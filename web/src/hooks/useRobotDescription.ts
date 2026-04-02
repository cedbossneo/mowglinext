import { useEffect, useRef, useState } from "react";
import { useWS } from "./useWS.ts";

export type RobotGeometry = {
    baseLength: number;
    baseWidth: number;
    baseHeight: number;
    wheelRadius: number;
    wheelWidth: number;
    wheelTrack: number;
    wheelXOffset: number;
    casterRadius: number;
    casterXOffset: number;
    casterTrack: number;
    bladeRadius: number;
};

const DEFAULTS: RobotGeometry = {
    baseLength: 0.54,
    baseWidth: 0.40,
    baseHeight: 0.19,
    wheelRadius: 0.04475,
    wheelWidth: 0.04,
    wheelTrack: 0.40,
    wheelXOffset: -0.18,
    casterRadius: 0.03,
    casterXOffset: 0.22,
    casterTrack: 0.36,
    bladeRadius: 0.09,
};

/**
 * Extract a named xacro:property value from URDF XML.
 * Matches: <xacro:property name="foo" value="0.123"/>
 */
const extractProperty = (xml: string, name: string): number | null => {
    // Match both xacro:property and plain property elements
    const re = new RegExp(
        `<(?:xacro:)?property\\s+name=["']${name}["']\\s+value=["']([^"']+)["']`,
        "i"
    );
    const m = xml.match(re);
    if (!m) return null;
    // Value may be a xacro expression like "${...}", skip those
    const val = m[1];
    if (val.includes("$")) return null;
    const n = parseFloat(val);
    return isNaN(n) ? null : n;
};

/**
 * Parse the URDF/xacro XML string into a RobotGeometry.
 * Falls back to DEFAULTS for any value not found.
 */
const parseUrdf = (xml: string): RobotGeometry => {
    const get = (name: string, fallback: number): number =>
        extractProperty(xml, name) ?? fallback;

    return {
        baseLength: get("base_length", DEFAULTS.baseLength),
        baseWidth: get("base_width", DEFAULTS.baseWidth),
        baseHeight: get("base_height", DEFAULTS.baseHeight),
        wheelRadius: get("wheel_radius", DEFAULTS.wheelRadius),
        wheelWidth: get("wheel_width", DEFAULTS.wheelWidth),
        wheelTrack: get("wheel_track", DEFAULTS.wheelTrack),
        wheelXOffset: get("wheel_x_offset", DEFAULTS.wheelXOffset),
        casterRadius: get("caster_radius", DEFAULTS.casterRadius),
        casterXOffset: get("caster_x_offset", DEFAULTS.casterXOffset),
        casterTrack: get("caster_track", DEFAULTS.casterTrack),
        bladeRadius: get("blade_radius", DEFAULTS.bladeRadius),
    };
};

/**
 * Subscribe to /robot_description and parse robot geometry from the URDF.
 * Returns DEFAULTS immediately and updates when the URDF is received.
 */
export const useRobotDescription = (): RobotGeometry => {
    const [geometry, setGeometry] = useState<RobotGeometry>(DEFAULTS);
    const receivedRef = useRef(false);

    const stream = useWS<string>(
        () => { /* error */ },
        () => { /* info */ },
        (raw) => {
            if (receivedRef.current) return; // only need the first message
            try {
                const msg = JSON.parse(raw);
                // rosbridge delivers std_msgs/String as {"data": "..."}
                // After snakeToPascal it may be {"Data": "..."} or {"data": "..."}
                const urdfXml: string = msg.Data ?? msg.data ?? "";
                if (urdfXml.length > 0) {
                    receivedRef.current = true;
                    setGeometry(parseUrdf(urdfXml));
                }
            } catch {
                // ignore parse errors
            }
        }
    );

    useEffect(() => {
        stream.start("/api/openmower/subscribe/robotDescription");
        return () => {
            stream.stop();
        };
    }, []);

    return geometry;
};

export { DEFAULTS as DEFAULT_GEOMETRY };
