import fetch from "node-fetch";
import { HttpsError } from "firebase-functions/v2/https";

// ---------------------------------------------------------------------------
// Types
// ---------------------------------------------------------------------------

/** Credentials blob that cloudflared embeds in its config file. */
export interface TunnelCredentials {
  AccountTag: string;
  /** Base64-encoded 32-byte tunnel secret */
  TunnelSecret: string;
  TunnelID: string;
}

export interface CreateTunnelResult {
  tunnelId: string;
  credentialsJson: TunnelCredentials;
  hostname: string;
}

// Shape of a Cloudflare API error response.
interface CloudflareError {
  code: number;
  message: string;
}

interface CloudflareResponse<T> {
  success: boolean;
  errors: CloudflareError[];
  result: T | null;
}

interface TunnelResult {
  id: string;
  credentials_file?: TunnelCredentials;
}

interface DnsRecordResult {
  id: string;
}

// ---------------------------------------------------------------------------
// Config
// ---------------------------------------------------------------------------

function getConfig(): {
  accountId: string;
  apiToken: string;
  zoneId: string;
  tunnelDomain: string;
} {
  const accountId = process.env["CLOUDFLARE_ACCOUNT_ID"];
  const apiToken = process.env["CLOUDFLARE_API_TOKEN"];
  const zoneId = process.env["CLOUDFLARE_ZONE_ID"];
  const tunnelDomain =
    process.env["TUNNEL_DOMAIN"] ?? "tunnel.mowgli.garden";

  if (!accountId || !apiToken || !zoneId) {
    throw new HttpsError(
      "internal",
      "Cloudflare configuration is incomplete. Set CLOUDFLARE_ACCOUNT_ID, CLOUDFLARE_API_TOKEN, and CLOUDFLARE_ZONE_ID."
    );
  }

  return { accountId, apiToken, zoneId, tunnelDomain };
}

// ---------------------------------------------------------------------------
// HTTP helper with retry-once on transient 5xx
// ---------------------------------------------------------------------------

async function cfFetch<T>(
  url: string,
  options: Parameters<typeof fetch>[1],
  retries = 1
): Promise<T> {
  const response = await fetch(url, options);

  // Retry once on transient server errors (502, 503, 504)
  if (retries > 0 && response.status >= 500) {
    return cfFetch<T>(url, options, retries - 1);
  }

  if (response.status === 401) {
    throw new HttpsError(
      "permission-denied",
      "Cloudflare API token is invalid or missing required scopes."
    );
  }

  if (response.status === 404) {
    throw new HttpsError(
      "not-found",
      `Cloudflare resource not found (${url}).`
    );
  }

  if (response.status === 429) {
    throw new HttpsError(
      "resource-exhausted",
      "Cloudflare API rate limit exceeded. Please retry later."
    );
  }

  if (!response.ok) {
    // Try to extract the Cloudflare error message for diagnostics.
    let detail = `HTTP ${response.status}`;
    try {
      const body = (await response.json()) as CloudflareResponse<unknown>;
      if (body.errors?.length) {
        detail = body.errors.map((e) => `[${e.code}] ${e.message}`).join("; ");
      }
    } catch {
      // JSON parse failed; keep the status-based message.
    }
    throw new HttpsError("internal", `Cloudflare API error: ${detail}`);
  }

  const body = (await response.json()) as CloudflareResponse<T>;
  if (!body.success || body.result === null) {
    const detail =
      body.errors?.map((e) => `[${e.code}] ${e.message}`).join("; ") ??
      "unknown";
    throw new HttpsError(
      "internal",
      `Cloudflare API returned failure: ${detail}`
    );
  }

  return body.result;
}

function authHeaders(apiToken: string): Record<string, string> {
  return {
    Authorization: `Bearer ${apiToken}`,
    "Content-Type": "application/json",
  };
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

/**
 * Creates a named Cloudflare Tunnel for the given robotId.
 * The tunnel name is "mowgli-<robotId>" to avoid collisions with other
 * accounts using the same Cloudflare account.
 *
 * Returns the tunnel ID, credentials JSON (to be forwarded to cloudflared on
 * the robot), and the hostname assigned to this tunnel.
 */
export async function createTunnel(
  robotId: string
): Promise<CreateTunnelResult> {
  const { accountId, apiToken, tunnelDomain } = getConfig();

  const tunnelName = `mowgli-${robotId}`;

  // Generate a 32-byte random secret, base64-encoded, as required by the
  // Cloudflare Tunnel API when creating a named tunnel with config_src=cloudflare.
  const secretBytes = new Uint8Array(32);
  crypto.getRandomValues(secretBytes);
  const tunnelSecret = Buffer.from(secretBytes).toString("base64");

  const result = await cfFetch<TunnelResult>(
    `https://api.cloudflare.com/client/v4/accounts/${accountId}/cfd_tunnel`,
    {
      method: "POST",
      headers: authHeaders(apiToken),
      body: JSON.stringify({
        name: tunnelName,
        tunnel_secret: tunnelSecret,
        config_src: "cloudflare",
      }),
    }
  );

  const tunnelId = result.id;
  const hostname = `r-${robotId}.${tunnelDomain}`;

  const credentialsJson: TunnelCredentials = {
    AccountTag: accountId,
    TunnelSecret: tunnelSecret,
    TunnelID: tunnelId,
  };

  return { tunnelId, credentialsJson, hostname };
}

/**
 * Deletes a Cloudflare Tunnel by its UUID.
 * Passes force=true to clean up active connections before deletion.
 */
export async function deleteTunnel(tunnelId: string): Promise<void> {
  const { accountId, apiToken } = getConfig();

  await cfFetch<unknown>(
    `https://api.cloudflare.com/client/v4/accounts/${accountId}/cfd_tunnel/${tunnelId}?force=true`,
    {
      method: "DELETE",
      headers: authHeaders(apiToken),
    }
  );
}

/**
 * Creates a proxied CNAME DNS record pointing hostname → tunnelId.cfargotunnel.com
 * in the configured zone.
 */
export async function createDNSRecord(
  hostname: string,
  tunnelId: string
): Promise<void> {
  const { apiToken, zoneId } = getConfig();

  await cfFetch<DnsRecordResult>(
    `https://api.cloudflare.com/client/v4/zones/${zoneId}/dns_records`,
    {
      method: "POST",
      headers: authHeaders(apiToken),
      body: JSON.stringify({
        type: "CNAME",
        name: hostname,
        content: `${tunnelId}.cfargotunnel.com`,
        proxied: true,
        ttl: 1, // 1 = auto (proxied records ignore TTL)
        comment: "MowgliNext robot tunnel — managed by Firebase Functions",
      }),
    }
  );
}

/**
 * Deletes the CNAME DNS record for the given hostname.
 * Lists records first to obtain the record ID, then deletes it.
 * Silently succeeds if no matching record is found (idempotent cleanup).
 */
export async function deleteDNSRecord(hostname: string): Promise<void> {
  const { apiToken, zoneId } = getConfig();

  interface ListResult {
    id: string;
    name: string;
  }

  // List CNAME records matching the hostname to get the record ID.
  const listResponse = await fetch(
    `https://api.cloudflare.com/client/v4/zones/${zoneId}/dns_records?type=CNAME&name=${encodeURIComponent(hostname)}&per_page=5`,
    { headers: authHeaders(apiToken) }
  );

  if (!listResponse.ok) {
    // Non-fatal during cleanup: log and return so other cleanup still proceeds.
    console.error(
      `deleteDNSRecord: failed to list records for ${hostname}: HTTP ${listResponse.status}`
    );
    return;
  }

  const listBody =
    (await listResponse.json()) as CloudflareResponse<ListResult[]>;
  if (!listBody.success || !listBody.result?.length) {
    // No record found — already deleted or never created.
    return;
  }

  const recordId = listBody.result[0]!.id;

  await cfFetch<unknown>(
    `https://api.cloudflare.com/client/v4/zones/${zoneId}/dns_records/${recordId}`,
    {
      method: "DELETE",
      headers: authHeaders(apiToken),
    }
  );
}
