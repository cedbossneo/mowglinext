import { onCall, HttpsError } from "firebase-functions/v2/https";
import { FieldValue } from "firebase-admin/firestore";
import { createHash } from "crypto";
import { db } from "./lib/firebase.js";
import { requireAuth } from "./lib/auth.js";
import { RegisterFcmTokenInputSchema } from "./lib/zod-schemas.js";

/**
 * Derives the Firestore document ID for an FCM token.
 * SHA-256 hex of the raw token string ensures:
 *  - multiple UIDs registering the same physical device replace cleanly
 *  - the token itself is not stored as a document key (tokens can be long)
 */
function tokenHash(fcmToken: string): string {
  return createHash("sha256").update(fcmToken, "utf8").digest("hex");
}

export const registerFcmToken = onCall(
  { region: "us-central1" },
  async (request): Promise<{ success: true }> => {
    // 1. Auth check.
    const uid = requireAuth(request);

    // 2. Validate input.
    const parseResult = RegisterFcmTokenInputSchema.safeParse(request.data);
    if (!parseResult.success) {
      throw new HttpsError(
        "invalid-argument",
        `Invalid input: ${parseResult.error.issues.map((i) => i.message).join(", ")}`
      );
    }
    const { robotId, fcmToken, platform } = parseResult.data;

    // 3. Verify caller is allowed to receive notifications for this robot.
    const robotSnap = await db.collection("robots").doc(robotId).get();

    if (!robotSnap.exists) {
      throw new HttpsError("not-found", `Robot ${robotId} not found.`);
    }

    const allowedUids: string[] = robotSnap.data()?.["allowedUids"] ?? [];
    if (!allowedUids.includes(uid)) {
      throw new HttpsError(
        "permission-denied",
        "You do not have access to this robot."
      );
    }

    // 4. Upsert the device token document. Idempotent by design — re-registering
    //    the same token just refreshes lastSeen and clears a stale deletedAt.
    const docId = tokenHash(fcmToken);
    const deviceRef = db
      .collection("robots")
      .doc(robotId)
      .collection("devices")
      .doc(docId);

    await deviceRef.set(
      {
        uid,
        fcmToken,
        platform,
        lastSeen: FieldValue.serverTimestamp(),
        deletedAt: null,
      },
      { merge: true }
    );

    return { success: true };
  }
);
