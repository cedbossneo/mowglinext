import type {Status} from '../types/ros';

export type FirmwareInventoryState = 'waiting' | 'disconnected' | 'stale' | 'compatible' | 'incompatible';

// Receipt of a cached message is not evidence of a live mainboard. Callers track
// advances of Status.stamp, avoiding any dependency on browser/robot clock sync.
export function firmwareInventoryState(status: Status, connected: boolean, lastAdvance: number | null, now: number): FirmwareInventoryState {
    if (!connected) return 'disconnected';
    if (lastAdvance === null) return 'waiting';
    if (now - lastAdvance > 6000) return 'stale';
    if (!status.firmware_version || status.firmware_version === 'unknown' || !status.firmware_protocol_version || status.firmware_compatible === undefined) return 'waiting';
    return status.firmware_compatible ? 'compatible' : 'incompatible';
}

export function browserBuildDiffers(browser: {revision?: string; built_at?: string}, server: {revision?: string; built_at?: string}): boolean {
    if (!browser.revision || !server.revision) return false;
    return browser.revision !== server.revision || Boolean(browser.built_at && server.built_at && browser.built_at !== server.built_at);
}

export function imageVersion(image: string): string {
    if (image.includes('@')) return '';
    const tail = image.split('/').pop() ?? '';
    return tail.includes(':') ? tail.slice(tail.lastIndexOf(':') + 1) : '';
}
