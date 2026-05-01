import { useCallback, useEffect, useRef, useState } from "react";

export type PairingState = "unstarted" | "pending" | "confirmed" | "failed" | "paired";

export interface PairingStatus {
    state: PairingState;
    robotID: string;
    ownerName?: string;
    confirmCode?: string;
}

export interface QRData {
    payload: string;
    robotID: string;
    setupToken: string;
}

export interface UsePairingStatusResult {
    status: PairingStatus | null;
    qr: QRData | null;
    error: string | null;
    refresh: () => void;
    confirm: () => Promise<void>;
    reset: () => Promise<void>;
}

const BASE = import.meta.env.DEV ? "http://localhost:4006" : "";

export function usePairingStatus(intervalMs = 1500): UsePairingStatusResult {
    const [status, setStatus] = useState<PairingStatus | null>(null);
    const [qr, setQr] = useState<QRData | null>(null);
    const [error, setError] = useState<string | null>(null);
    const setupTokenRef = useRef<string | null>(null);
    const intervalRef = useRef<ReturnType<typeof setInterval> | null>(null);
    const mountedRef = useRef(true);

    const fetchQr = useCallback(async () => {
        try {
            const res = await fetch(`${BASE}/api/pair/qr`);
            if (!res.ok) throw new Error(`QR fetch failed: ${res.status}`);
            const data = (await res.json()) as QRData;
            if (mountedRef.current) {
                setQr(data);
                setupTokenRef.current = data.setupToken;
                setError(null);
            }
        } catch (e: unknown) {
            if (mountedRef.current) {
                setError(e instanceof Error ? e.message : "Failed to fetch QR code");
            }
        }
    }, []);

    const fetchStatus = useCallback(async () => {
        try {
            const res = await fetch(`${BASE}/api/pair/status`);
            if (!res.ok) throw new Error(`Status fetch failed: ${res.status}`);
            const data = (await res.json()) as PairingStatus;
            if (mountedRef.current) {
                setStatus(data);
                // Stop polling once a terminal state is reached
                if (data.state === "paired" || data.state === "failed") {
                    if (intervalRef.current !== null) {
                        clearInterval(intervalRef.current);
                        intervalRef.current = null;
                    }
                }
            }
        } catch (e: unknown) {
            if (mountedRef.current) {
                setError(e instanceof Error ? e.message : "Failed to fetch pairing status");
            }
        }
    }, []);

    const startPolling = useCallback(() => {
        if (intervalRef.current !== null) {
            clearInterval(intervalRef.current);
        }
        intervalRef.current = setInterval(fetchStatus, intervalMs);
    }, [fetchStatus, intervalMs]);

    const refresh = useCallback(() => {
        void fetchQr();
        void fetchStatus();
        startPolling();
    }, [fetchQr, fetchStatus, startPolling]);

    useEffect(() => {
        mountedRef.current = true;
        void fetchQr();
        void fetchStatus();
        startPolling();
        return () => {
            mountedRef.current = false;
            if (intervalRef.current !== null) {
                clearInterval(intervalRef.current);
            }
        };
    }, [fetchQr, fetchStatus, startPolling]);

    const confirm = useCallback(async () => {
        const token = setupTokenRef.current;
        if (!token) {
            setError("No setup token available — reload the QR code first.");
            return;
        }
        try {
            const res = await fetch(`${BASE}/api/pair/confirm`, {
                method: "POST",
                headers: { "Content-Type": "application/json" },
                body: JSON.stringify({ setupToken: token }),
            });
            if (!res.ok) throw new Error(`Confirm failed: ${res.status}`);
            // Polling will pick up the paired state automatically
        } catch (e: unknown) {
            setError(e instanceof Error ? e.message : "Confirm request failed");
        }
    }, []);

    // TODO: Confirm with the backend agent that /api/pair/reset is implemented
    // before relying on this in production. The endpoint should clear the
    // pending pairing session and return state to "unstarted".
    const reset = useCallback(async () => {
        try {
            await fetch(`${BASE}/api/pair/reset`, { method: "POST" });
            if (mountedRef.current) {
                setStatus(null);
                setError(null);
            }
            void fetchStatus();
            startPolling();
        } catch (e: unknown) {
            if (mountedRef.current) {
                setError(e instanceof Error ? e.message : "Reset request failed");
            }
        }
    }, [fetchStatus, startPolling]);

    return { status, qr, error, refresh, confirm, reset };
}
