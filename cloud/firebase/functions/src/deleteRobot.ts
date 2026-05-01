import { onCall, HttpsError } from "firebase-functions/v2/https";
import { db } from "./lib/firebase.js";
import { requireAuth } from "./lib/auth.js";
import { deleteTunnel, deleteDNSRecord } from "./lib/cloudflare.js";
import { DeleteRobotInputSchema } from "./lib/zod-schemas.js";

export const deleteRobot = onCall(
  { region: "us-central1" },
  async (request): Promise<{ success: true }> => {
    // 1. Auth check.
    const uid = requireAuth(request);

    // 2. Validate input.
    const parseResult = DeleteRobotInputSchema.safeParse(request.data);
    if (!parseResult.success) {
      throw new HttpsError(
        "invalid-argument",
        `Invalid input: ${parseResult.error.issues.map((i) => i.message).join(", ")}`
      );
    }
    const { robotId } = parseResult.data;

    // 3. Load robot doc and verify caller is the owner.
    const robotRef = db.collection("robots").doc(robotId);
    const robotSnap = await robotRef.get();

    if (!robotSnap.exists) {
      throw new HttpsError("not-found", `Robot ${robotId} not found.`);
    }

    const robotData = robotSnap.data()!;
    if (robotData["ownerUid"] !== uid) {
      throw new HttpsError(
        "permission-denied",
        "Only the robot owner can delete the robot."
      );
    }

    const tunnelId: string = robotData["tunnelId"] ?? "";
    const tunnelHostname: string = robotData["tunnelHostname"] ?? "";
    const allowedUids: string[] = robotData["allowedUids"] ?? [];

    // 4. Delete Cloudflare resources first. Log but don't abort on failure —
    //    we still want to clean up Firestore so the robot is removed from the
    //    user's view. Stale tunnels can be reaped manually if needed.
    if (tunnelHostname) {
      try {
        await deleteDNSRecord(tunnelHostname);
      } catch (err) {
        console.error("deleteRobot: failed to delete DNS record", {
          tunnelHostname,
          err,
        });
      }
    }

    if (tunnelId) {
      try {
        await deleteTunnel(tunnelId);
      } catch (err) {
        console.error("deleteRobot: failed to delete Cloudflare tunnel", {
          tunnelId,
          err,
        });
      }
    }

    // 5. Delete Firestore docs in a batch.
    //    Batches are limited to 500 operations; allowedUids is expected to be
    //    small (tens of users), well within the limit.
    const batch = db.batch();

    // Remove users/{memberUid}/robots/{robotId} for all current members + owner.
    for (const memberUid of allowedUids) {
      const userRobotRef = db
        .collection("users")
        .doc(memberUid)
        .collection("robots")
        .doc(robotId);
      batch.delete(userRobotRef);
    }

    // Delete the root robot document last.
    batch.delete(robotRef);

    await batch.commit();

    return { success: true };
  }
);
