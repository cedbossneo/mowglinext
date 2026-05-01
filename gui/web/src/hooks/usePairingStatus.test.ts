import { act, renderHook, waitFor } from "@testing-library/react";
import { afterEach, describe, expect, it, vi } from "vitest";
import { usePairingStatus } from "./usePairingStatus";

// ── Fixtures ──────────────────────────────────────────────────────────────────

const QR_RESPONSE = {
    payload: "mowgli://pair?key=TESTKEY&token=TOK123&ip=192.168.1.100&host=mowgli.local",
    robotID: "MOWGLI-TEST01",
    setupToken: "TOK123",
};

const STATUS_UNSTARTED = { state: "unstarted", robotID: "MOWGLI-TEST01" };
const STATUS_PENDING = {
    state: "pending",
    robotID: "MOWGLI-TEST01",
    ownerName: "Alice",
    confirmCode: "4821",
};
const STATUS_PAIRED = { state: "paired", robotID: "MOWGLI-TEST01", ownerName: "Alice" };
const STATUS_FAILED = { state: "failed", robotID: "MOWGLI-TEST01" };

function ok(body: unknown): Response {
    return { ok: true, status: 200, json: () => Promise.resolve(body) } as Response;
}

function fail(status = 503): Response {
    return { ok: false, status, json: () => Promise.resolve({}) } as Response;
}

function makeStaticFetch(urlMap: Record<string, unknown>): typeof fetch {
    return vi.fn((url: string) => {
        const key = Object.keys(urlMap).find((k) => url.includes(k));
        return Promise.resolve(key !== undefined ? ok(urlMap[key]) : fail(404));
    }) as unknown as typeof fetch;
}

afterEach(() => {
    vi.restoreAllMocks();
    vi.useRealTimers();
});

// ── Tests ────────────────────────────────────────────────────────────────────

describe("usePairingStatus", () => {
    it("fetches QR and initial status on mount", async () => {
        globalThis.fetch = makeStaticFetch({
            "/api/pair/qr": QR_RESPONSE,
            "/api/pair/status": STATUS_UNSTARTED,
        });

        const { result } = renderHook(() => usePairingStatus(60000));

        await waitFor(() => {
            expect(result.current.qr).not.toBeNull();
            expect(result.current.status).not.toBeNull();
        });

        expect(result.current.qr?.robotID).toBe("MOWGLI-TEST01");
        expect(result.current.qr?.setupToken).toBe("TOK123");
        expect(result.current.status?.state).toBe("unstarted");
        expect(result.current.error).toBeNull();
    });

    it("polls status at the given interval (fake timers)", async () => {
        vi.useFakeTimers({ shouldAdvanceTime: false });

        let statusCallCount = 0;
        globalThis.fetch = vi.fn((url: string) => {
            if (url.includes("/api/pair/status")) statusCallCount++;
            return Promise.resolve(ok(url.includes("/api/pair/status") ? STATUS_UNSTARTED : QR_RESPONSE));
        }) as unknown as typeof fetch;

        renderHook(() => usePairingStatus(1500));

        // Flush the initial async fetches triggered by useEffect
        await act(async () => { await Promise.resolve(); await Promise.resolve(); });
        const initialCount = statusCallCount;

        // Advance 3 poll intervals; each tick fires fetchStatus
        await act(async () => {
            vi.advanceTimersByTime(1500);
            await Promise.resolve(); await Promise.resolve();
            vi.advanceTimersByTime(1500);
            await Promise.resolve(); await Promise.resolve();
            vi.advanceTimersByTime(1500);
            await Promise.resolve(); await Promise.resolve();
        });

        expect(statusCallCount).toBeGreaterThan(initialCount + 2);
    });

    it("stops polling when state becomes 'paired'", async () => {
        vi.useFakeTimers({ shouldAdvanceTime: false });

        let statusCallCount = 0;
        globalThis.fetch = vi.fn((url: string) => {
            if (url.includes("/api/pair/status")) statusCallCount++;
            return Promise.resolve(ok(url.includes("/api/pair/status") ? STATUS_PAIRED : QR_RESPONSE));
        }) as unknown as typeof fetch;

        renderHook(() => usePairingStatus(1500));

        // Flush initial fetch — should set state to paired and clear the interval
        await act(async () => { await Promise.resolve(); await Promise.resolve(); });
        const countAfterPaired = statusCallCount;

        // Advance 4 intervals — no more calls expected
        await act(async () => {
            vi.advanceTimersByTime(6000);
            await Promise.resolve(); await Promise.resolve();
        });

        expect(statusCallCount).toBe(countAfterPaired);
    });

    it("stops polling when state becomes 'failed'", async () => {
        vi.useFakeTimers({ shouldAdvanceTime: false });

        let statusCallCount = 0;
        globalThis.fetch = vi.fn((url: string) => {
            if (url.includes("/api/pair/status")) statusCallCount++;
            return Promise.resolve(ok(url.includes("/api/pair/status") ? STATUS_FAILED : QR_RESPONSE));
        }) as unknown as typeof fetch;

        renderHook(() => usePairingStatus(1500));

        await act(async () => { await Promise.resolve(); await Promise.resolve(); });
        const countAfterFailed = statusCallCount;

        await act(async () => {
            vi.advanceTimersByTime(6000);
            await Promise.resolve(); await Promise.resolve();
        });

        expect(statusCallCount).toBe(countAfterFailed);
    });

    it("sets error when QR fetch fails", async () => {
        globalThis.fetch = vi.fn((url: string) => {
            if (url.includes("/api/pair/qr")) return Promise.reject(new Error("Network error"));
            return Promise.resolve(ok(STATUS_UNSTARTED));
        }) as unknown as typeof fetch;

        const { result } = renderHook(() => usePairingStatus(60000));

        await waitFor(() => expect(result.current.error).not.toBeNull());
        expect(result.current.error).toContain("Network error");
    });

    it("continues polling after a status fetch error (retries on error)", async () => {
        vi.useFakeTimers({ shouldAdvanceTime: false });

        let statusCallCount = 0;
        globalThis.fetch = vi.fn((url: string) => {
            if (url.includes("/api/pair/qr")) return Promise.resolve(ok(QR_RESPONSE));
            statusCallCount++;
            // First two calls fail, then succeed with pending
            return Promise.resolve(statusCallCount <= 2 ? fail(503) : ok(STATUS_PENDING));
        }) as unknown as typeof fetch;

        const { result } = renderHook(() => usePairingStatus(1500));

        // Flush initial fetch (call 1 — fails)
        await act(async () => { await Promise.resolve(); await Promise.resolve(); });
        // Advance two more intervals (calls 2 and 3 — call 3 succeeds)
        await act(async () => {
            vi.advanceTimersByTime(1500);
            await Promise.resolve(); await Promise.resolve();
            vi.advanceTimersByTime(1500);
            await Promise.resolve(); await Promise.resolve();
        });

        expect(result.current.status?.state).toBe("pending");
        expect(result.current.status?.ownerName).toBe("Alice");
        expect(result.current.status?.confirmCode).toBe("4821");
    });

    it("confirm() POSTs to /api/pair/confirm with the correct setupToken body", async () => {
        // Keep the mock typed as vi.Mock so .mock.calls is accessible for inspection
        const fetchMock = vi.fn((url: string) => {
            if (url.includes("/api/pair/qr")) return Promise.resolve(ok(QR_RESPONSE));
            if (url.includes("/api/pair/confirm")) return Promise.resolve(ok({}));
            return Promise.resolve(ok(STATUS_UNSTARTED));
        });
        globalThis.fetch = fetchMock as unknown as typeof fetch;

        const { result } = renderHook(() => usePairingStatus(60000));
        await waitFor(() => expect(result.current.qr).not.toBeNull());

        await act(async () => { await result.current.confirm(); });

        const calls = fetchMock.mock.calls as Array<[string, RequestInit?]>;
        const confirmCall = calls.find(([url]) => url.includes("/api/pair/confirm"));
        expect(confirmCall).toBeDefined();
        const body = JSON.parse(confirmCall![1]!.body as string) as Record<string, unknown>;
        expect(body).toEqual({ setupToken: "TOK123" });
    });

    it("confirm() sets an error when no setup token is available", async () => {
        // All fetches fail so setupToken is never stored in the ref
        globalThis.fetch = vi.fn(() => Promise.resolve(fail(503))) as unknown as typeof fetch;

        const { result } = renderHook(() => usePairingStatus(60000));
        await waitFor(() => expect(result.current.error).not.toBeNull());

        await act(async () => { await result.current.confirm(); });
        expect(result.current.error).toMatch(/setup token/i);
    });

    it("reset() POSTs to /api/pair/reset and restarts polling", async () => {
        // After reset(), the hook clears status then immediately re-fetches.
        // We verify the reset endpoint was called; status will be re-populated
        // by the re-fetch triggered inside reset().
        const fetchMock = vi.fn((url: string) => {
            if (url.includes("/api/pair/qr")) return Promise.resolve(ok(QR_RESPONSE));
            if (url.includes("/api/pair/reset")) return Promise.resolve(ok({}));
            return Promise.resolve(ok(STATUS_UNSTARTED));
        });
        globalThis.fetch = fetchMock as unknown as typeof fetch;

        const { result } = renderHook(() => usePairingStatus(60000));
        await waitFor(() => expect(result.current.status).not.toBeNull());

        await act(async () => { await result.current.reset(); });

        const calls = fetchMock.mock.calls as Array<[string, RequestInit?]>;
        const resetCall = calls.find(([url]) => url.includes("/api/pair/reset"));
        expect(resetCall).toBeDefined();
        // After reset + re-fetch, status should be re-populated (not stuck as null)
        await waitFor(() => expect(result.current.status?.state).toBe("unstarted"));
    });
});
