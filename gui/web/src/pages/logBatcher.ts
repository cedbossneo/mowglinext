export interface LogBatcher<T> {
  push: (value: T) => void;
  reset: () => void;
}

export function appendCappedBatch<T>(
  previous: T[],
  batch: T[],
  maxItems: number,
): T[] {
  if (maxItems <= 0) return [];
  if (batch.length >= maxItems) return batch.slice(batch.length - maxItems);

  const overflow = previous.length + batch.length - maxItems;
  if (overflow > 0) return previous.slice(overflow).concat(batch);
  return previous.concat(batch);
}

/** Collect incoming values and deliver at most one batch per interval. */
export function createLogBatcher<T>(
  onBatch: (batch: T[]) => void,
  intervalMs: number,
): LogBatcher<T> {
  let pending: T[] = [];
  let timer: ReturnType<typeof setTimeout> | null = null;

  const flush = () => {
    timer = null;
    if (pending.length === 0) return;
    const batch = pending;
    pending = [];
    onBatch(batch);
  };

  const reset = () => {
    if (timer !== null) clearTimeout(timer);
    timer = null;
    pending = [];
  };

  return {
    push: (value: T) => {
      pending.push(value);
      if (timer === null) timer = setTimeout(flush, intervalMs);
    },
    reset,
  };
}
