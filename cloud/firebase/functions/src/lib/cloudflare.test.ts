import { describe, it, expect, vi, beforeEach } from "vitest";

// ---------------------------------------------------------------------------
// Mock node-fetch before any cloudflare import
// ---------------------------------------------------------------------------
const mockFetch = vi.fn();

vi.mock("node-fetch", () => ({
  default: (...args: unknown[]) => mockFetch(...args),
}));

// ---------------------------------------------------------------------------
// Mock firebase-functions/v2/https so HttpsError works in isolation
// ---------------------------------------------------------------------------
vi.mock("firebase-functions/v2/https", () => ({
  HttpsError: class HttpsError extends Error {
    code: string;
    constructor(code: string, message: string) {
      super(message);
      this.code = code;
    }
  },
}));

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

function mockResponse(
  status: number,
  body: unknown,
  ok = status >= 200 && status < 300
): Promise<{ ok: boolean; status: number; json: () => Promise<unknown> }> {
  return Promise.resolve({
    ok,
    status,
    json: () => Promise.resolve(body),
  });
}

function cfSuccess<T>(result: T): unknown {
  return { success: true, errors: [], result };
}

// Set required env vars before the module is imported.
process.env["CLOUDFLARE_ACCOUNT_ID"] = "test-account-id";
process.env["CLOUDFLARE_API_TOKEN"] = "test-api-token";
process.env["CLOUDFLARE_ZONE_ID"] = "test-zone-id";
process.env["TUNNEL_DOMAIN"] = "tunnel.mowgli.garden";

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

describe("cloudflare", () => {
  beforeEach(() => {
    vi.clearAllMocks();
  });

  // ── createTunnel ────────────────────────────────────────────────────────

  describe("createTunnel", () => {
    it("creates a tunnel and returns structured credentials", async () => {
      const { createTunnel } = await import("./cloudflare.js");

      mockFetch.mockReturnValueOnce(
        mockResponse(200, cfSuccess({ id: "tunnel-uuid-001" }))
      );

      const result = await createTunnel("robot-xyz");

      expect(mockFetch).toHaveBeenCalledOnce();
      const [url, opts] = mockFetch.mock.calls[0] as [
        string,
        { method: string; body: string },
      ];
      expect(url).toContain("/accounts/test-account-id/cfd_tunnel");
      expect(opts.method).toBe("POST");

      const body = JSON.parse(opts.body) as Record<string, unknown>;
      expect(body["name"]).toBe("mowgli-robot-xyz");
      expect(body["config_src"]).toBe("cloudflare");

      expect(result.tunnelId).toBe("tunnel-uuid-001");
      expect(result.hostname).toBe("r-robot-xyz.tunnel.mowgli.garden");
      expect(result.credentialsJson.AccountTag).toBe("test-account-id");
      expect(result.credentialsJson.TunnelID).toBe("tunnel-uuid-001");
      // TunnelSecret must be a non-empty base64 string (32 random bytes)
      expect(result.credentialsJson.TunnelSecret).toMatch(/^[A-Za-z0-9+/]+=*$/);
    });

    it("maps 401 to permission-denied HttpsError", async () => {
      const { createTunnel } = await import("./cloudflare.js");
      mockFetch.mockReturnValueOnce(
        mockResponse(401, { success: false, errors: [] }, false)
      );
      await expect(createTunnel("robot-xyz")).rejects.toMatchObject({
        code: "permission-denied",
      });
    });

    it("maps 404 to not-found HttpsError", async () => {
      const { createTunnel } = await import("./cloudflare.js");
      mockFetch.mockReturnValueOnce(
        mockResponse(404, { success: false, errors: [] }, false)
      );
      await expect(createTunnel("robot-xyz")).rejects.toMatchObject({
        code: "not-found",
      });
    });

    it("maps 429 to resource-exhausted HttpsError", async () => {
      const { createTunnel } = await import("./cloudflare.js");
      mockFetch.mockReturnValueOnce(
        mockResponse(429, { success: false, errors: [] }, false)
      );
      await expect(createTunnel("robot-xyz")).rejects.toMatchObject({
        code: "resource-exhausted",
      });
    });

    it("retries once on transient 5xx then succeeds", async () => {
      const { createTunnel } = await import("./cloudflare.js");
      mockFetch
        .mockReturnValueOnce(
          mockResponse(503, { success: false, errors: [] }, false)
        )
        .mockReturnValueOnce(
          mockResponse(200, cfSuccess({ id: "tunnel-uuid-retry" }))
        );

      const result = await createTunnel("robot-xyz");

      expect(mockFetch).toHaveBeenCalledTimes(2);
      expect(result.tunnelId).toBe("tunnel-uuid-retry");
    });

    it("throws internal HttpsError after exhausting retries on persistent 5xx", async () => {
      const { createTunnel } = await import("./cloudflare.js");
      mockFetch
        .mockReturnValueOnce(
          mockResponse(503, { success: false, errors: [] }, false)
        )
        .mockReturnValueOnce(
          mockResponse(503, { success: false, errors: [] }, false)
        );

      await expect(createTunnel("robot-xyz")).rejects.toMatchObject({
        code: "internal",
      });
      expect(mockFetch).toHaveBeenCalledTimes(2);
    });
  });

  // ── deleteTunnel ────────────────────────────────────────────────────────

  describe("deleteTunnel", () => {
    it("sends DELETE with force=true to the correct endpoint", async () => {
      const { deleteTunnel } = await import("./cloudflare.js");
      mockFetch.mockReturnValueOnce(mockResponse(200, cfSuccess({})));

      await deleteTunnel("tunnel-uuid-001");

      const [url, opts] = mockFetch.mock.calls[0] as [
        string,
        { method: string },
      ];
      expect(url).toContain("/cfd_tunnel/tunnel-uuid-001");
      expect(url).toContain("force=true");
      expect(opts.method).toBe("DELETE");
    });
  });

  // ── createDNSRecord ─────────────────────────────────────────────────────

  describe("createDNSRecord", () => {
    it("creates a proxied CNAME record pointing to cfargotunnel.com", async () => {
      const { createDNSRecord } = await import("./cloudflare.js");
      mockFetch.mockReturnValueOnce(
        mockResponse(200, cfSuccess({ id: "dns-record-id-1" }))
      );

      await createDNSRecord(
        "r-robot-xyz.tunnel.mowgli.garden",
        "tunnel-uuid-001"
      );

      const [url, opts] = mockFetch.mock.calls[0] as [
        string,
        { method: string; body: string },
      ];
      expect(url).toContain("/zones/test-zone-id/dns_records");
      expect(opts.method).toBe("POST");

      const body = JSON.parse(opts.body) as Record<string, unknown>;
      expect(body["type"]).toBe("CNAME");
      expect(body["name"]).toBe("r-robot-xyz.tunnel.mowgli.garden");
      expect(body["content"]).toBe("tunnel-uuid-001.cfargotunnel.com");
      expect(body["proxied"]).toBe(true);
    });
  });

  // ── deleteDNSRecord ─────────────────────────────────────────────────────

  describe("deleteDNSRecord", () => {
    it("lists matching records then deletes by record ID", async () => {
      const { deleteDNSRecord } = await import("./cloudflare.js");

      mockFetch
        .mockReturnValueOnce(
          mockResponse(
            200,
            cfSuccess([
              {
                id: "dns-record-id-1",
                name: "r-robot-xyz.tunnel.mowgli.garden",
              },
            ])
          )
        )
        .mockReturnValueOnce(mockResponse(200, cfSuccess({})));

      await deleteDNSRecord("r-robot-xyz.tunnel.mowgli.garden");

      expect(mockFetch).toHaveBeenCalledTimes(2);
      const [deleteUrl, deleteOpts] = mockFetch.mock.calls[1] as [
        string,
        { method: string },
      ];
      expect(deleteUrl).toContain("/dns_records/dns-record-id-1");
      expect(deleteOpts.method).toBe("DELETE");
    });

    it("silently succeeds when no matching DNS record exists", async () => {
      const { deleteDNSRecord } = await import("./cloudflare.js");

      mockFetch.mockReturnValueOnce(mockResponse(200, cfSuccess([])));

      await expect(
        deleteDNSRecord("r-robot-xyz.tunnel.mowgli.garden")
      ).resolves.toBeUndefined();

      // Only the list request, no delete request
      expect(mockFetch).toHaveBeenCalledTimes(1);
    });
  });
});
