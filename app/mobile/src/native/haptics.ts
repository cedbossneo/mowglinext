import { Haptics, ImpactStyle } from "@capacitor/haptics";

export async function triggerHaptic(style: "light" | "medium" | "heavy" = "medium") {
  try {
    const map = {
      light: ImpactStyle.Light,
      medium: ImpactStyle.Medium,
      heavy: ImpactStyle.Heavy,
    };
    await Haptics.impact({ style: map[style] });
  } catch {
    // Haptics not available (web fallback)
  }
}
