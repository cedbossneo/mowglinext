import { HttpsError } from "firebase-functions/v2/https";
import type { CallableRequest } from "firebase-functions/v2/https";

/**
 * Extracts the caller's uid from the Functions v2 CallableRequest and throws
 * HttpsError('unauthenticated') if the caller has no Firebase Auth session.
 *
 * Call this as the very first line of every onCall handler.
 */
export function requireAuth(request: CallableRequest): string {
  if (!request.auth?.uid) {
    throw new HttpsError(
      "unauthenticated",
      "You must be signed in to call this function."
    );
  }
  return request.auth.uid;
}
