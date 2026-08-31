import {act, renderHook} from "@testing-library/react";
import {afterEach, beforeEach, describe, expect, it, vi} from "vitest";
import {GnssStatusConstants} from "../types/ros.ts";

const wsMock = vi.hoisted(() => ({
    onError: undefined as ((error: Error) => void) | undefined,
    onInfo: undefined as ((message: string) => void) | undefined,
    onData: undefined as ((data: unknown, first?: boolean) => void) | undefined,
    start: vi.fn(),
    stop: vi.fn(),
}));

const diagnosticsMock = vi.hoisted(() => ({
    diagnostics: {
        status: [{
            level: 0,
            name: "GPS",
            message: "RTK Fixed",
            hardware_id: "old",
            values: [{key: "fix_status", value: "2"}],
            receivedAt: 1,
        }],
    },
}));

vi.mock("./useWS.ts", () => ({
    useWS: (
        onError: (error: Error) => void,
        onInfo: (message: string) => void,
        onData: (data: unknown, first?: boolean) => void,
    ) => {
        wsMock.onError = onError;
        wsMock.onInfo = onInfo;
        wsMock.onData = onData;
        return {start: wsMock.start, stop: wsMock.stop, sendJsonMessage: vi.fn()};
    },
}));

vi.mock("./useDiagnostics.ts", () => ({
    useDiagnostics: () => ({
        diagnostics: diagnosticsMock.diagnostics,
        start: vi.fn(),
        stop: vi.fn(),
    }),
}));

import {GNSS_STATUS_AUTHORITY_TIMEOUT_MS, useGnssStatus} from "./useGnssStatus.ts";

const healthyStatus = {
    header: {stamp: {sec: 100, nanosec: 0}, frame_id: "gps"},
    backend: "universal",
    receiver_vendor: "Unicore",
    receiver_model: "UM982",
    fix_type: GnssStatusConstants.FIX_TYPE_RTK_FIXED,
    fix_valid: true,
    rtk_mode: GnssStatusConstants.RTK_MODE_FIXED,
    capability_flags:
        GnssStatusConstants.CAP_CORRECTION_TRANSPORT |
        GnssStatusConstants.CAP_CORRECTION_FLOW |
        GnssStatusConstants.CAP_CORRECTION_SEMANTIC,
    value_flags:
        GnssStatusConstants.CAP_CORRECTION_TRANSPORT |
        GnssStatusConstants.CAP_CORRECTION_FLOW |
        GnssStatusConstants.CAP_CORRECTION_SEMANTIC,
    correction_transport_status: GnssStatusConstants.CORRECTION_TRANSPORT_STATUS_STREAMING,
    correction_response_accepted: true,
    correction_flow_status: GnssStatusConstants.CORRECTION_FLOW_STATUS_ACTIVE,
    correction_semantic_status: GnssStatusConstants.CORRECTION_SEMANTIC_STATUS_HEALTHY,
    correction_source: "caster:2101/MOUNT",
};

describe("useGnssStatus lifecycle authority", () => {
    beforeEach(() => {
        vi.useFakeTimers();
        wsMock.start.mockClear();
        wsMock.stop.mockClear();
        wsMock.onError = undefined;
        wsMock.onInfo = undefined;
        wsMock.onData = undefined;
    });

    afterEach(() => {
        vi.useRealTimers();
    });

    it("starts fail-closed and does not resurrect diagnostics-only state", () => {
        const {result} = renderHook(() => useGnssStatus());
        expect(result.current).toEqual({});
        expect(wsMock.start).toHaveBeenCalledWith("/api/mowglinext/subscribe/gnssStatus");
    });

    it("clears on socket loss and reconnect info alone cannot restore", () => {
        const {result} = renderHook(() => useGnssStatus());
        act(() => wsMock.onData?.(healthyStatus, true));
        expect(result.current.fix_type).toBe(GnssStatusConstants.FIX_TYPE_RTK_FIXED);

        act(() => wsMock.onError?.(new Error("Stream closed")));
        expect(result.current).toEqual({});
        act(() => wsMock.onInfo?.("Stream connected"));
        expect(result.current).toEqual({});
    });

    it("treats the backend tombstone as authoritative CLEAR", () => {
        const {result} = renderHook(() => useGnssStatus());
        act(() => wsMock.onData?.(healthyStatus, true));
        act(() => wsMock.onData?.({}, false));
        expect(result.current).toEqual({});
    });

    it("expires delivery with a monotonic timer and fresh data restores it", () => {
        const {result} = renderHook(() => useGnssStatus());
        act(() => wsMock.onData?.(healthyStatus, true));
        act(() => {
            void vi.advanceTimersByTime(GNSS_STATUS_AUTHORITY_TIMEOUT_MS - 1);
        });
        expect(result.current.fix_valid).toBe(true);
        act(() => {
            void vi.advanceTimersByTime(1);
        });
        expect(result.current).toEqual({});

        act(() => wsMock.onData?.({...healthyStatus, position_observation_sequence: 2}, false));
        expect(result.current.fix_valid).toBe(true);
    });

    it("moves the local silence deadline when another typed sample arrives", () => {
        const {result} = renderHook(() => useGnssStatus());
        act(() => wsMock.onData?.(healthyStatus, true));
        act(() => {
            void vi.advanceTimersByTime(GNSS_STATUS_AUTHORITY_TIMEOUT_MS - 1_000);
        });
        act(() => wsMock.onData?.({...healthyStatus, position_observation_sequence: 2}, false));
        act(() => {
            void vi.advanceTimersByTime(1_000);
        });
        expect(result.current.fix_valid).toBe(true);
        act(() => {
            void vi.advanceTimersByTime(GNSS_STATUS_AUTHORITY_TIMEOUT_MS - 1_000);
        });
        expect(result.current).toEqual({});
    });

    it("stops the stream and timer on unmount", () => {
        const {unmount} = renderHook(() => useGnssStatus());
        act(() => wsMock.onData?.(healthyStatus, true));
        unmount();
        expect(wsMock.stop).toHaveBeenCalled();
        expect(vi.getTimerCount()).toBe(0);
    });
});
