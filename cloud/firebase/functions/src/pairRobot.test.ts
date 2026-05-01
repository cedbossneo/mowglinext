import { describe, it, expect, vi, beforeEach } from "vitest";

// ---------------------------------------------------------------------------
// Mock firebase-admin/firestore
// ---------------------------------------------------------------------------
const mockBatchSet = vi.fn();
const mockBatchCommit = vi.fn().mockResolvedValue(undefined);
const mockBatch = { set: mockBatchSet, commit: mockBatchCommit };
const mockGet = vi.fn();

const mockSubCollectionDoc = vi.fn().mockReturnValue({ get: mockGet });
const mockSubCollection = vi.fn().mockReturnValue({ doc: mockSubCollectionDoc });

const mockRobotDocRef = {
  get: mockGet,
  collection: mockSubCollection,
};

const mockCollectionDoc = vi.fn().mockReturnValue(mockRobotDocRef);
const mockCollection = vi.fn().mockReturnValue({ doc: mockCollectionDoc });

vi.mock("./lib/firebase.js", () => ({
  db: {
    collection: mockCollection,
    batch: vi.fn().mockReturnValue(mockBatch),
  },
  auth: {},
  app: {},
}));

vi.mock("firebase-admin/firestore", () => ({
  FieldValue: {
    serverTimestamp: vi.fn().mockReturnValue("__SERVER_TIMESTAMP__"),
    arrayUnion: vi.fn((v: string) => ({ _type: "arrayUnion", v })),
    arrayRemove: vi.fn((v: string) => ({ _type: "arrayRemove", v })),
  },
}));

// ---------------------------------------------------------------------------
// Mock cloudflare
// ---------------------------------------------------------------------------
const mockCreateTunnel = vi.fn();
const mockCreateDNSRecord = vi.fn();
const mockDeleteTunnel = vi.fn();
const mockDeleteDNSRecord = vi.fn();

vi.mock("./lib/cloudflare.js", () => ({
  createTunnel: (...args: unknown[]) => mockCreateTunnel(...args),
  createDNSRecord: (...args: unknown[]) => mockCreateDNSRecord(...args),
  deleteTunnel: (...args: unknown[]) => mockDeleteTunnel(...args),
  deleteDNSRecord: (...args: unknown[]) => mockDeleteDNSRecord(...args),
}));

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

// A valid 32-byte base64-encoded public key (all zeros — synthetic test value).
const VALID_PUB_KEY = Buffer.alloc(32).toString("base64");

function makeRequest(
  uid: string | null,
  data: Record<string, unknown>
): { auth: { uid: string } | null; data: unknown } {
  return { auth: uid ? { uid } : null, data };
}

const VALID_INPUT = {
  robotId: "test-robot-1",
  robotPubKey: VALID_PUB_KEY,
  setupToken: "setup-token-abc123",
};

const TUNNEL_RESULT = {
  tunnelId: "tunnel-uuid-001",
  credentialsJson: {
    AccountTag: "acct-tag",
    TunnelSecret: "c2VjcmV0",
    TunnelID: "tunnel-uuid-001",
  },
  hostname: "r-test-robot-1.tunnel.mowgli.garden",
};

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

describe("pairRobot", () => {
  beforeEach(() => {
    vi.clearAllMocks();
    mockCreateTunnel.mockResolvedValue(TUNNEL_RESULT);
    mockCreateDNSRecord.mockResolvedValue(undefined);
    mockDeleteTunnel.mockResolvedValue(undefined);
    mockDeleteDNSRecord.mockResolvedValue(undefined);
    mockBatchSet.mockReturnValue(undefined);
    mockBatchCommit.mockResolvedValue(undefined);
    // Re-wire collection mock after clearAllMocks
    mockCollectionDoc.mockReturnValue(mockRobotDocRef);
    mockCollection.mockReturnValue({ doc: mockCollectionDoc });
  });

  it("rejects unauthenticated callers", async () => {
    const { pairRobot } = await import("./pairRobot.js");
    mockGet.mockResolvedValue({ exists: false, data: () => undefined });

    await expect(pairRobot.run(makeRequest(null, VALID_INPUT) as never)).rejects.toMatchObject({
      code: "unauthenticated",
    });
  });

  it("happy path: creates tunnel, DNS record, and Firestore docs", async () => {
    const { pairRobot } = await import("./pairRobot.js");
    mockGet.mockResolvedValue({ exists: false, data: () => undefined });

    const result = await pairRobot.run(makeRequest("user-uid-123", VALID_INPUT) as never);

    expect(mockCreateTunnel).toHaveBeenCalledWith("test-robot-1");
    expect(mockCreateDNSRecord).toHaveBeenCalledWith(
      "r-test-robot-1.tunnel.mowgli.garden",
      "tunnel-uuid-001"
    );
    expect(mockBatchCommit).toHaveBeenCalledOnce();
    expect(result).toEqual({
      tunnelHostname: "r-test-robot-1.tunnel.mowgli.garden",
      credentialsJson: TUNNEL_RESULT.credentialsJson,
    });
  });

  it("rejects already-paired robot with a different setupToken", async () => {
    const { pairRobot } = await import("./pairRobot.js");
    mockGet.mockResolvedValue({
      exists: true,
      data: () => ({ setupToken: "different-token", ownerUid: "other-user" }),
    });

    await expect(pairRobot.run(makeRequest("user-uid-123", VALID_INPUT) as never)).rejects.toMatchObject({
      code: "failed-precondition",
    });
    expect(mockCreateTunnel).not.toHaveBeenCalled();
  });

  it("cleans up tunnel when DNS creation fails (no DNS to delete)", async () => {
    const { pairRobot } = await import("./pairRobot.js");
    mockGet.mockResolvedValue({ exists: false, data: () => undefined });
    mockCreateDNSRecord.mockRejectedValue(new Error("DNS error"));

    await expect(pairRobot.run(makeRequest("user-uid-123", VALID_INPUT) as never)).rejects.toMatchObject({
      code: "internal",
    });

    expect(mockDeleteTunnel).toHaveBeenCalledWith("tunnel-uuid-001");
    // DNS record was never confirmed created, so deleteDNSRecord must not be called
    expect(mockDeleteDNSRecord).not.toHaveBeenCalled();
  });

  it("cleans up tunnel and DNS when Firestore write fails", async () => {
    const { pairRobot } = await import("./pairRobot.js");
    mockGet.mockResolvedValue({ exists: false, data: () => undefined });
    mockBatchCommit.mockRejectedValue(new Error("Firestore write failed"));

    await expect(pairRobot.run(makeRequest("user-uid-123", VALID_INPUT) as never)).rejects.toMatchObject({
      code: "internal",
    });

    expect(mockDeleteDNSRecord).toHaveBeenCalledWith(
      "r-test-robot-1.tunnel.mowgli.garden"
    );
    expect(mockDeleteTunnel).toHaveBeenCalledWith("tunnel-uuid-001");
  });
});
