import { z } from "zod";

// ---------------------------------------------------------------------------
// Shared validators
// ---------------------------------------------------------------------------

const robotIdSchema = z
  .string()
  .min(1)
  .max(64)
  .regex(/^[a-zA-Z0-9_-]+$/, "robotId must be alphanumeric with _ or -");

// ---------------------------------------------------------------------------
// pairRobot
// ---------------------------------------------------------------------------

export const PairRobotInputSchema = z.object({
  /** Stable hardware identifier (e.g. serial number or UUID from firmware). */
  robotId: robotIdSchema,
  /** Optional human-readable display name. Defaults to robotId if omitted. */
  robotName: z.string().min(1).max(128).optional(),
  /**
   * Base64-encoded Curve25519 public key (32 bytes raw) from the robot's
   * Noise IK keypair. Used by the mobile app to verify the robot's identity
   * during the encrypted tunnel handshake.
   */
  robotPubKey: z
    .string()
    .regex(/^[A-Za-z0-9+/]+=*$/, "robotPubKey must be valid base64")
    .refine(
      (v) => Buffer.from(v, "base64").length === 32,
      "robotPubKey must decode to exactly 32 bytes (Curve25519)"
    ),
  /**
   * One-time setup token displayed in the robot's local UI or generated
   * during provisioning. Prevents unauthorized pairing attempts.
   */
  setupToken: z.string().min(8).max(256),
});

export type PairRobotInput = z.infer<typeof PairRobotInputSchema>;

// ---------------------------------------------------------------------------
// inviteUser
// ---------------------------------------------------------------------------

export const InviteUserInputSchema = z.object({
  robotId: robotIdSchema,
  inviteeEmail: z.string().email("inviteeEmail must be a valid email address"),
});

export type InviteUserInput = z.infer<typeof InviteUserInputSchema>;

// ---------------------------------------------------------------------------
// revokeUser
// ---------------------------------------------------------------------------

export const RevokeUserInputSchema = z.object({
  robotId: robotIdSchema,
  /** Firebase Auth uid of the user whose access is being revoked. */
  uid: z.string().min(1).max(128),
});

export type RevokeUserInput = z.infer<typeof RevokeUserInputSchema>;

// ---------------------------------------------------------------------------
// registerFcmToken
// ---------------------------------------------------------------------------

export const RegisterFcmTokenInputSchema = z.object({
  robotId: robotIdSchema,
  fcmToken: z.string().min(1).max(4096),
  platform: z.enum(["ios", "android", "web"]),
});

export type RegisterFcmTokenInput = z.infer<typeof RegisterFcmTokenInputSchema>;

// ---------------------------------------------------------------------------
// deleteRobot
// ---------------------------------------------------------------------------

export const DeleteRobotInputSchema = z.object({
  robotId: robotIdSchema,
});

export type DeleteRobotInput = z.infer<typeof DeleteRobotInputSchema>;
