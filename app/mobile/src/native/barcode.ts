import {
  BarcodeScanner,
  BarcodeFormat,
} from '@capacitor-mlkit/barcode-scanning';

/**
 * Request camera permission if needed, then scan one QR code.
 * Returns the raw string value, or null if the user cancelled.
 * Throws if camera permission is denied.
 */
export async function scanBarcode(): Promise<string | null> {
  const { camera } = await BarcodeScanner.requestPermissions();
  if (camera !== 'granted' && camera !== 'limited') {
    throw new Error(
      'Camera permission denied. Enable it in Settings to scan QR codes.',
    );
  }

  // Install the Google Barcode Scanner module on Android if not already present.
  // This is a no-op on iOS and web.
  try {
    const { available } =
      await BarcodeScanner.isGoogleBarcodeScannerModuleAvailable();
    if (!available) {
      await BarcodeScanner.installGoogleBarcodeScannerModule();
    }
  } catch {
    // Not on Android or module already available — continue
  }

  const { barcodes } = await BarcodeScanner.scan({
    formats: [BarcodeFormat.QrCode],
  });

  if (!barcodes || barcodes.length === 0) return null;
  return barcodes[0]?.rawValue ?? null;
}

/**
 * Returns true if barcode scanning is supported on this platform/device.
 */
export async function isScanSupported(): Promise<boolean> {
  try {
    const { supported } = await BarcodeScanner.isSupported();
    return supported;
  } catch {
    return false;
  }
}
