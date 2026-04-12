import {useCallback, useRef, useState} from "react";
import type {TwistStamped} from "../../../types/ros.ts";
import type {IJoystickUpdateEvent} from "react-joystick-component/build/lib/Joystick";

const JOY_SEND_INTERVAL_MS = 100;

interface UseManualModeOptions {
    mowerAction: (action: string, params: Record<string, unknown>) => () => Promise<void>;
    joyStream: { sendJsonMessage: (msg: unknown) => void; start: (uri: string) => void };
}

export function useManualMode({mowerAction, joyStream}: UseManualModeOptions) {
    const [manualMode, setManualMode] = useState<number | undefined>();
    const lastTwistRef = useRef<TwistStamped | null>(null);
    const joyIntervalRef = useRef<ReturnType<typeof setInterval> | undefined>(undefined);

    const startJoyInterval = useCallback(() => {
        clearInterval(joyIntervalRef.current);
        joyIntervalRef.current = setInterval(() => {
            if (lastTwistRef.current) {
                joyStream.sendJsonMessage(lastTwistRef.current);
            }
        }, JOY_SEND_INTERVAL_MS);
    }, [joyStream]);

    const stopJoyInterval = useCallback(() => {
        clearInterval(joyIntervalRef.current);
        joyIntervalRef.current = undefined;
    }, []);

    const handleManualMode = async () => {
        // Start the joy WebSocket immediately so it's ready when the user moves the joystick.
        // Don't wait for the MANUAL_MOWING state to propagate back via highLevelStatus.
        joyStream.start("/api/mowglinext/publish/joy");
        await mowerAction("high_level_control", {Command: 7})();
        // Enable mowing blade immediately, then keep it alive every 10s
        await mowerAction("mow_enabled", {MowEnabled: 1, MowDirection: 0})();
        setManualMode(setInterval(() => {
            (async () => {
                await mowerAction("mow_enabled", {MowEnabled: 1, MowDirection: 0})();
            })();
        }, 10000));
    };

    const handleStopManualMode = async () => {
        await mowerAction("high_level_control", {Command: 2})();
        clearInterval(manualMode);
        setManualMode(undefined);
        stopJoyInterval();
        lastTwistRef.current = null;
        await mowerAction("mow_enabled", {MowEnabled: 0, MowDirection: 0})();
    };

    const handleJoyMove = useCallback((event: IJoystickUpdateEvent) => {
        const msg: TwistStamped = {
            header: {stamp: {sec: 0, nanosec: 0}, frame_id: ""},
            twist: {linear: {x: event.y ?? 0, y: 0, z: 0}, angular: {z: (event.x ?? 0) * -1, x: 0, y: 0}},
        };
        lastTwistRef.current = msg;
        joyStream.sendJsonMessage(msg);
        // Start the repeat interval if not already running
        if (!joyIntervalRef.current) {
            startJoyInterval();
        }
    }, [joyStream, startJoyInterval]);

    const handleJoyStop = useCallback(() => {
        const msg: TwistStamped = {
            header: {stamp: {sec: 0, nanosec: 0}, frame_id: ""},
            twist: {linear: {x: 0, y: 0, z: 0}, angular: {z: 0, x: 0, y: 0}},
        };
        lastTwistRef.current = null;
        stopJoyInterval();
        joyStream.sendJsonMessage(msg);
    }, [joyStream, stopJoyInterval]);

    return {manualMode, handleManualMode, handleStopManualMode, handleJoyMove, handleJoyStop};
}
