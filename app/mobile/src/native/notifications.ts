import { PushNotifications } from "@capacitor/push-notifications";

export async function registerPushNotifications(
  robotId: string,
  baseHttpUrl: string,
  authToken: string | null
): Promise<string | null> {
  try {
    const permission = await PushNotifications.requestPermissions();
    if (permission.receive !== "granted") return null;

    await PushNotifications.register();

    return new Promise((resolve) => {
      PushNotifications.addListener("registration", async (token) => {
        // Register token with proxy
        try {
          const headers: Record<string, string> = {
            "Content-Type": "application/json",
          };
          if (authToken) {
            headers["Authorization"] = `Bearer ${authToken}`;
          }
          await fetch(`${baseHttpUrl}/api/push/register`, {
            method: "POST",
            headers,
            body: JSON.stringify({
              robot_id: robotId,
              platform: getPlatform(),
              token: token.value,
            }),
          });
        } catch {
          // ignore registration failure
        }
        resolve(token.value);
      });

      PushNotifications.addListener("registrationError", () => {
        resolve(null);
      });
    });
  } catch {
    return null;
  }
}

function getPlatform(): "ios" | "android" {
  const ua = navigator.userAgent.toLowerCase();
  return ua.includes("iphone") || ua.includes("ipad") ? "ios" : "android";
}

export async function addPushListeners(
  onNotification: (title: string, body: string) => void
) {
  await PushNotifications.addListener(
    "pushNotificationReceived",
    (notification) => {
      onNotification(
        notification.title ?? "MowgliNext",
        notification.body ?? ""
      );
    }
  );
}
