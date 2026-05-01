import { Haptics, ImpactStyle, NotificationType } from '@capacitor/haptics';

export async function impactLight(): Promise<void> {
  try {
    await Haptics.impact({ style: ImpactStyle.Light });
  } catch {
    // Haptics not available on web/simulator — ignore silently
  }
}

export async function impactMedium(): Promise<void> {
  try {
    await Haptics.impact({ style: ImpactStyle.Medium });
  } catch {
    // ignore
  }
}

export async function impactHeavy(): Promise<void> {
  try {
    await Haptics.impact({ style: ImpactStyle.Heavy });
  } catch {
    // ignore
  }
}

export async function notifySuccess(): Promise<void> {
  try {
    await Haptics.notification({ type: NotificationType.Success });
  } catch {
    // ignore
  }
}

export async function notifyError(): Promise<void> {
  try {
    await Haptics.notification({ type: NotificationType.Error });
  } catch {
    // ignore
  }
}

export async function notifyWarning(): Promise<void> {
  try {
    await Haptics.notification({ type: NotificationType.Warning });
  } catch {
    // ignore
  }
}
