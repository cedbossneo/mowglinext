import { onCall, HttpsError } from "firebase-functions/v2/https";
import { FieldValue } from "firebase-admin/firestore";
import { db } from "./lib/firebase.js";
import { requireAuth } from "./lib/auth.js";
import { RevokeUserInputSchema } from "./lib/zod-schemas.js";

export const revokeUser = onCall(
  { region: "us-central1" },
  async (request): Promise<{ success: true }> => {
    // 1. Auth check.
    const callerUid = requireAuth(request);

    // 2. Validate input.
    const parseResult = RevokeUserInputSchema.safeParse(request.data);
    if (!parseResult.success) {
      throw new HttpsError(
        "invalid-argument",
        `Invalid input: ${parseResult.error.issues.map((i) => i.message).join(", ")}`
      );
    }
    const { robotId, uid: targetUid } = parseResult.data;

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
        "Only the robot owner can revoke user access."
      );
    }

    // 4. Prevent revoking the owner themselves.
    if (targetUid === robotData["ownerUid"]) {
      throw new HttpsError(
        "failed-precondition",
        "Cannot revoke the robot owner's access. Transfer ownership or delete the robot instead."
      );
    }

    // 5. Idempotent: if user is not in allowedUids, there is nothing to do.
    const allowedUids: string[] = robotData["allowedUids"] ?? [];
    if (!allowedUids.includes(targetUid)) {
      return { success: true };
    }

    // 6. Remove atomically.
    const batch = db.batch();

    batch.update(robotRef, {
      allowedUids: FieldValue.arrayRemove(targetUid),
    });

    const userRobotRef = db
      .collection("users")
      .doc(targetUid)
      .collection("robots")
      .doc(robotId);

    batch.delete(userRobotRef);

    await batch.commit();

    return { success: true };
  }
);
