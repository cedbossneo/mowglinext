import {useEffect, useState} from "react";
import type {NotificationInstance} from "antd/es/notification/interface";

// Display-only Mapbox bearing (compass rotation in degrees, clockwise from
// north). Persists in GUI config under `gui.map.display.bearing`. The bearing
// is the visual rotation of the basemap + overlays — it does NOT change the
// robot's frame, and the lat/lon emitted by Mapbox click events is already
// rotation-aware, so no inverse-rotation math is required for click handlers.

let bearingTimeout: ReturnType<typeof setTimeout> | null = null;

interface UseMapBearingOptions {
    config: Record<string, string>;
    setConfig: (cfg: Record<string, string>) => Promise<void>;
    notification: NotificationInstance;
}

export function useMapBearing({config, setConfig, notification}: UseMapBearingOptions) {
    const [bearing, setBearing] = useState(0);

    useEffect(() => {
        const b = parseFloat(config["gui.map.display.bearing"] ?? "0");
        if (!isNaN(b)) setBearing(b);
    }, [config]);

    const handleBearing = (value: number) => {
        // Normalise into [-180, 180) so the persisted value stays compact.
        const normalised = ((value + 180) % 360 + 360) % 360 - 180;
        if (bearingTimeout != null) clearTimeout(bearingTimeout);
        bearingTimeout = setTimeout(() => {
            (async () => {
                try {
                    await setConfig({"gui.map.display.bearing": normalised.toString()});
                } catch (e: any) {
                    notification.error({message: "Failed to save bearing", description: e.message});
                }
            })();
        }, 1000);
        setBearing(normalised);
    };

    return {bearing, handleBearing};
}
