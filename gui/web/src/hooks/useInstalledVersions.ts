import {useCallback, useEffect, useRef, useState} from 'react';
import type {ApiVersionsResponse} from '../api/Api';
import {httpBase} from '../utils/apiHost';

export const browserBuild = {
    revision: import.meta.env.VITE_BUILD_REVISION as string | undefined,
    version: import.meta.env.VITE_BUILD_VERSION as string | undefined,
    built_at: import.meta.env.VITE_BUILD_TIME as string | undefined,
};

export function useInstalledVersions() {
    const [data, setData] = useState<ApiVersionsResponse | null>(null);
    const [loading, setLoading] = useState(true);
    const [error, setError] = useState(false);
    const controller = useRef<AbortController | null>(null);
    const refresh = useCallback(async () => {
        controller.current?.abort();
        const request = new AbortController();
        controller.current = request;
        setLoading(true);
        try {
            const response = await fetch(`${httpBase()}/api/system/versions`, {signal: request.signal, cache: 'no-store'});
            if (!response.ok) throw new Error('version inventory unavailable');
            const next: ApiVersionsResponse = await response.json();
            if (!Array.isArray(next.components) || typeof next.docker_available !== 'boolean') throw new Error('invalid inventory');
            if (!request.signal.aborted) { setData(next); setError(false); }
        } catch {
            if (!request.signal.aborted) setError(true);
        } finally {
            if (!request.signal.aborted) setLoading(false);
        }
    }, []);
    useEffect(() => {
        void refresh();
        const poll = window.setInterval(() => { if (!document.hidden) void refresh(); }, 30000);
        const focus = () => { void refresh(); };
        window.addEventListener('focus', focus);
        return () => { controller.current?.abort(); window.clearInterval(poll); window.removeEventListener('focus', focus); };
    }, [refresh]);
    return {data, loading, error, refresh};
}
