import { onCall, HttpsError } from "firebase-functions/v2/https";
import { FieldValue } from "firebase-admin/firestore";
import { db, auth } from "./lib/firebase.js";
import { requireAuth } from "./lib/auth.js";
import { InviteUserInputSchema } from "./lib/zod-schemas.js";

export const inviteUser = onCall(
  { region: "us-central1" },
  async (request): Promise<{ success: true }> => {
    // 1. Auth check.
    const callerUid = requireAuth(request);

    // 2. Validate input.
    const parseResult = InviteUserInputSchema.safeParse(request.data);
    if (!parseResult.success) {
      throw new HttpsError(
        "invalid-argument",
        `Invalid input: ${parseResult.error.issues.map((i) => i.message).join(", ")}`
      );
    }
    const { robotId, inviteeEmail } = parseResult.data;

    // 3. Load robot doc and verify caller is the owner.
    const robotRef = db.collection("robots").doc(robotId);
    const robotSnap = await robotRef.get();

    if (!robotSnap.exists) {
      throw new HttpsError("not-found", `Robot ${robotId} not found.`);
    }

    const robotData = robotSnap.data()!;
    if (robotData["ownerUid"] !== callerUid) {
      throw new HttpsError(
        "permission-denied",
        "Only the robot owner can invite users."
      );
    }

    // 4. Look up the invitee by email via Firebase Auth.
    let inviteeUid: string;
    try {
      const inviteeRecord = await auth.getUserByEmail(inviteeEmail);
      inviteeUid = inviteeRecord.uid;
    } catch (err: unknown) {
      // firebase-admin throws with code 'auth/user-not-found'
      const code = (err as { code?: string }).code;
      if (code === "auth/user-not-found") {
        throw new HttpsError(
          "not-found",
          `No user found with email ${inviteeEmail}. They must create an account first.`
        );
      }
      console.error("inviteUser: Auth lookup failed", err);
      throw new HttpsError("internal", "Failed to look up user by email.");
    }

    // 5. Idempotent: if already in allowedUids, return success without writing.
    const allowedUids: string[] = robotData["allowedUids"] ?? [];
    if (allowedUids.includes(inviteeUid)) {
      return { success: true };
    }

    // 6. Write atomically.
    const now = FieldValue.serverTimestamp();
    const batch = db.batch();

    batch.update(robotRef, {
      allowedUids: FieldValue.arrayUnion(inviteeUid),
    });

    const userRobotRef = db
      .collection("users")
      .doc(inviteeUid)
      .collection("robots")
      .doc(robotId);

    batch.set(
      userRobotRef,
      {
        name: robotData["name"] ?? robotId,
        role: "member",
        addedAt: now,
      },
      { merge: true } // idempotent if doc already exists
    );

    await batch.commit();

    return { success: true };
  }
);
