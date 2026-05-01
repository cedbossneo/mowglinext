import { onCall, HttpsError } from "firebase-functions/v2/https";
import { FieldValue } from "firebase-admin/firestore";
import { db } from "./lib/firebase.js";
import { requireAuth } from "./lib/auth.js";
import {
  createTunnel,
  createDNSRecord,
  deleteTunnel,
  deleteDNSRecord,
} from "./lib/cloudflare.js";
import { PairRobotInputSchema } from "./lib/zod-schemas.js";
import type { TunnelCredentials } from "./lib/cloudflare.js";

export interface PairRobotResponse {
  tunnelHostname: string;
  credentialsJson: TunnelCredentials;
}

export const pairRobot = onCall(
  { region: "us-central1" },
  async (request): Promise<PairRobotResponse> => {
    // 1. Auth check first.
    const uid = requireAuth(request);

    // 2. Validate input.
    const parseResult = PairRobotInputSchema.safeParse(request.data);
    if (!parseResult.success) {
      throw new HttpsError(
        "invalid-argument",
        `Invalid input: ${parseResult.error.issues.map((i) => i.message).join(", ")}`
      );
    }
    const { robotId, robotName, robotPubKey, setupToken } = parseResult.data;

    // 3. Check robot is not already paired by a different owner.
    const robotRef = db.collection("robots").doc(robotId);
    const robotSnap = await robotRef.get();

    if (robotSnap.exists) {
      const data = robotSnap.data()!;
      // Allow re-pairing only if the same setupToken is presented (idempotent
      // re-pair by the same provisioning flow, e.g. after a network failure).
      if (data["setupToken"] !== setupToken) {
        throw new HttpsError(
          "failed-precondition",
          `Robot ${robotId} is already paired. Provide the original setupToken to re-pair.`
        );
      }
    }

    // 4. Provision Cloudflare tunnel + DNS. Track what we created for cleanup.
    let tunnelId: string | null = null;
    let hostname: string | null = null;
    let credentialsJson: TunnelCredentials | null = null;
    let dnsCreated = false;

    try {
      const tunnelResult = await createTunnel(robotId);
      tunnelId = tunnelResult.tunnelId;
      hostname = tunnelResult.hostname;
      credentialsJson = tunnelResult.credentialsJson;

      await createDNSRecord(hostname, tunnelId);
      dnsCreated = true;

      // 5. Write Firestore atomically via a batch.
      const now = FieldValue.serverTimestamp();
      const batch = db.batch();

      batch.set(robotRef, {
        ownerUid: uid,
        name: robotName ?? robotId,
        robotPubKey,
        tunnelHostname: hostname,
        tunnelId,
        allowedUids: [uid],
        pairedAt: now,
        lastSeen: now,
        status: "paired",
        // Store the setupToken so idempotent re-pair can be validated.
        setupToken,
      });

      const userRobotRef = db
        .collection("users")
        .doc(uid)
        .collection("robots")
        .doc(robotId);

      batch.set(userRobotRef, {
        name: robotName ?? robotId,
        role: "owner",
        addedAt: now,
      });

      await batch.commit();
    } catch (err) {
      // 6. Cleanup Cloudflare resources if anything after tunnel creation failed.
      if (tunnelId !== null) {
        try {
          if (dnsCreated && hostname !== null) {
            await deleteDNSRecord(hostname);
          }
          await deleteTunnel(tunnelId);
        } catch (cleanupErr) {
          console.error(
            "pairRobot: cleanup failed after error — manual Cloudflare cleanup may be required",
            { tunnelId, hostname, cleanupErr }
          );
        }
      }

      // Re-throw HttpsErrors as-is; wrap everything else.
      if (err instanceof HttpsError) throw err;
      console.error("pairRobot: unexpected error", err);
      throw new HttpsError("internal", "Failed to pair robot. Please retry.");
    }

    return {
      tunnelHostname: hostname!,
      credentialsJson: credentialsJson!,
    };
  }
);
