import { describe, it, expect, vi, beforeEach } from "vitest";

// ---------------------------------------------------------------------------
// Mock firebase-admin/firestore
// ---------------------------------------------------------------------------
const mockBatchUpdate = vi.fn();
const mockBatchSet = vi.fn();
const mockBatchCommit = vi.fn().mockResolvedValue(undefined);
const mockBatch = {
  update: mockBatchUpdate,
  set: mockBatchSet,
  commit: mockBatchCommit,
};
const mockRobotGet = vi.fn();

const mockUserRobotDocRef = { get: vi.fn() };
const mockUserRobotDoc = vi.fn().mockReturnValue(mockUserRobotDocRef);
const mockUserRobotsCollection = vi.fn().mockReturnValue({ doc: mockUserRobotDoc });
const mockUserDocRef = { collection: mockUserRobotsCollection };
const mockUserDoc = vi.fn().mockReturnValue(mockUserDocRef);

const mockRobotDocRef = { get: mockRobotGet };
const mockRobotsDoc = vi.fn().mockReturnValue(mockRobotDocRef);

vi.mock("./lib/firebase.js", () => ({
  db: {
    collection: (name: string) => {
      if (name === "robots") return { doc: mockRobotsDoc };
      if (name === "users") return { doc: mockUserDoc };
      return { doc: vi.fn() };
    },
    batch: vi.fn().mockReturnValue(mockBatch),
  },
  auth: {
    getUserByEmail: vi.fn(),
  },
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
// Helpers
// ---------------------------------------------------------------------------

function makeRequest(
  uid: string | null,
  data: Record<string, unknown>
): { auth: { uid: string } | null; data: unknown } {
  return { auth: uid ? { uid } : null, data };
}

const VALID_INPUT = {
  robotId: "robot-abc",
  inviteeEmail: "invitee@example.com",
};

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

describe("inviteUser", () => {
  beforeEach(() => {
    vi.clearAllMocks();
    mockBatchUpdate.mockReturnValue(undefined);
    mockBatchSet.mockReturnValue(undefined);
    mockBatchCommit.mockResolvedValue(undefined);
  });

  it("owner can invite a user who exists in Auth", async () => {
    const { inviteUser } = await import("./inviteUser.js");
    const { auth } = await import("./lib/firebase.js");

    mockRobotGet.mockResolvedValue({
      exists: true,
      data: () => ({
        ownerUid: "owner-uid",
        name: "My Mower",
        allowedUids: ["owner-uid"],
      }),
    });
    vi.mocked(auth.getUserByEmail).mockResolvedValue({ uid: "invitee-uid" } as never);

    const result = await inviteUser.run(makeRequest("owner-uid", VALID_INPUT) as never);

    expect(mockBatchUpdate).toHaveBeenCalled();
    expect(mockBatchSet).toHaveBeenCalled();
    expect(mockBatchCommit).toHaveBeenCalledOnce();
    expect(result).toEqual({ success: true });
  });

  it("non-owner cannot invite users", async () => {
    const { inviteUser } = await import("./inviteUser.js");

    mockRobotGet.mockResolvedValue({
      exists: true,
      data: () => ({
        ownerUid: "owner-uid",
        name: "My Mower",
        allowedUids: ["owner-uid", "other-uid"],
      }),
    });

    await expect(
      inviteUser.run(makeRequest("other-uid", VALID_INPUT) as never)
    ).rejects.toMatchObject({ code: "permission-denied" });

    expect(mockBatchCommit).not.toHaveBeenCalled();
  });

  it("duplicate invite is idempotent — no write if already in allowedUids", async () => {
    const { inviteUser } = await import("./inviteUser.js");
    const { auth } = await import("./lib/firebase.js");

    mockRobotGet.mockResolvedValue({
      exists: true,
      data: () => ({
        ownerUid: "owner-uid",
        name: "My Mower",
        allowedUids: ["owner-uid", "invitee-uid"],
      }),
    });
    vi.mocked(auth.getUserByEmail).mockResolvedValue({ uid: "invitee-uid" } as never);

    const result = await inviteUser.run(makeRequest("owner-uid", VALID_INPUT) as never);

    expect(result).toEqual({ success: true });
    expect(mockBatchCommit).not.toHaveBeenCalled();
  });

  it("rejects invite when email is not registered in Auth", async () => {
    const { inviteUser } = await import("./inviteUser.js");
    const { auth } = await import("./lib/firebase.js");

    mockRobotGet.mockResolvedValue({
      exists: true,
      data: () => ({
        ownerUid: "owner-uid",
        name: "My Mower",
        allowedUids: ["owner-uid"],
      }),
    });
    const notFoundError = Object.assign(new Error("not found"), {
      code: "auth/user-not-found",
    });
    vi.mocked(auth.getUserByEmail).mockRejectedValue(notFoundError);

    await expect(
      inviteUser.run(makeRequest("owner-uid", VALID_INPUT) as never)
    ).rejects.toMatchObject({ code: "not-found" });

    expect(mockBatchCommit).not.toHaveBeenCalled();
  });
});
