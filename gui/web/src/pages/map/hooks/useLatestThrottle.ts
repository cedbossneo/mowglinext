import { useEffect, useRef } from "react";

export interface LatestThrottle<T> {
  push: (value: T) => void;
  cancel: () => void;
}

/**
 * Emit immediately, then coalesce values that arrive inside the interval into
 * one trailing update. The trailing update always receives the latest value.
 */
export function createLatestThrottle<T>(
  callback: (value: T) => void,
  intervalMs: number,
): LatestThrottle<T> {
  let lastEmitAt = Number.NEGATIVE_INFINITY;
  let latest: T | undefined;
  let hasLatest = false;
  let timer: ReturnType<typeof setTimeout> | null = null;

  const emit = () => {
    timer = null;
    if (!hasLatest) return;
    const value = latest as T;
    latest = undefined;
    hasLatest = false;
    lastEmitAt = Date.now();
    callback(value);
  };

  const cancel = () => {
    if (timer !== null) clearTimeout(timer);
    timer = null;
    latest = undefined;
    hasLatest = false;
    lastEmitAt = Number.NEGATIVE_INFINITY;
  };

  return {
    push: (value: T) => {
      latest = value;
      hasLatest = true;
      const remainingMs = intervalMs - (Date.now() - lastEmitAt);
      if (remainingMs <= 0) {
        if (timer !== null) clearTimeout(timer);
        emit();
      } else if (timer === null) {
        timer = setTimeout(emit, remainingMs);
      }
    },
    cancel,
  };
}

/** Keep the throttle stable across renders while invoking the latest callback. */
export function useLatestThrottle<T>(
  callback: (value: T) => void,
  intervalMs: number,
): LatestThrottle<T> {
  const callbackRef = useRef(callback);
  callbackRef.current = callback;

  const throttleRef = useRef<LatestThrottle<T> | null>(null);
  if (throttleRef.current === null) {
    throttleRef.current = createLatestThrottle<T>(
      (value) => callbackRef.current(value),
      intervalMs,
    );
  }

  useEffect(() => {
    const throttle = throttleRef.current;
    return () => throttle?.cancel();
  }, []);

  return throttleRef.current;
}
