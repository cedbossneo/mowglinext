import {useEffect, useMemo, useRef, useState} from "react";
import {GnssStatus} from "../types/ros.ts";
import {useWS} from "./useWS.ts";
import {useDiagnostics} from "./useDiagnostics.ts";
import {
    deriveGnssStatusFromDiagnostics,
    hasTypedGnssStatusSample,
    mergeGnssStatusDiagnosticProjection,
} from "../utils/gpsStatus.ts";

export const GNSS_STATUS_AUTHORITY_TIMEOUT_MS = 5_000;

export const useGnssStatus = () => {
    const [gnssStatus, setGnssStatus] = useState<GnssStatus>({});
    const parseWarningLoggedRef = useRef(false);
    const authorityTimerRef = useRef<ReturnType<typeof setTimeout> | null>(null);
    const {diagnostics} = useDiagnostics();

    const clearAuthorityTimer = () => {
        if (authorityTimerRef.current !== null) {
            clearTimeout(authorityTimerRef.current);
            authorityTimerRef.current = null;
        }
    };
    const invalidate = () => {
        clearAuthorityTimer();
        setGnssStatus({});
    };
    const refreshAuthorityTimer = () => {
        clearAuthorityTimer();
        authorityTimerRef.current = setTimeout(() => {
            authorityTimerRef.current = null;
            setGnssStatus({});
        }, GNSS_STATUS_AUTHORITY_TIMEOUT_MS);
    };

    const gnssStatusStream = useWS<string>(() => invalidate(), () => {}, (payload) => {
        try {
            const typedStatus = payload as unknown as GnssStatus;
            if (!hasTypedGnssStatusSample(typedStatus)) {
                invalidate();
                return;
            }
            setGnssStatus(typedStatus);
            refreshAuthorityTimer();
            parseWarningLoggedRef.current = false;
        } catch (error) {
            if (!parseWarningLoggedRef.current) {
                console.warn("Ignoring malformed /gps/status frame", error);
                parseWarningLoggedRef.current = true;
            }
        }
    });
    useEffect(() => {
        gnssStatusStream.start("/api/mowglinext/subscribe/gnssStatus");
        return () => {
            gnssStatusStream.stop();
            clearAuthorityTimer();
        };
        // useWS stores the latest callbacks in refs; the stream itself must
        // follow component lifetime rather than reconnect on every state render.
        // eslint-disable-next-line react-hooks/exhaustive-deps
    }, []);

    const diagnosticFallback = useMemo(
        () => deriveGnssStatusFromDiagnostics(diagnostics),
        [diagnostics],
    );

    return hasTypedGnssStatusSample(gnssStatus)
        ? mergeGnssStatusDiagnosticProjection(gnssStatus, diagnosticFallback)
        : {};
};
