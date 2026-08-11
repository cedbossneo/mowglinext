import { afterEach, beforeEach, describe, expect, it, vi } from "vitest";
import { createLatestThrottle } from "./useLatestThrottle.ts";

describe("createLatestThrottle", () => {
  beforeEach(() => {
    vi.useFakeTimers();
    vi.setSystemTime(0);
  });

  afterEach(() => {
    vi.useRealTimers();
  });

  it("emits the leading value and the latest trailing value", () => {
    const emit = vi.fn();
    const throttle = createLatestThrottle<number>(emit, 100);

    for (let value = 0; value < 100; value += 1) {
      throttle.push(value);
    }

    expect(emit).toHaveBeenCalledTimes(1);
    expect(emit).toHaveBeenLastCalledWith(0);

    vi.advanceTimersByTime(100);
    expect(emit).toHaveBeenCalledTimes(2);
    expect(emit).toHaveBeenLastCalledWith(99);
  });

  it("caps a sustained 100 Hz stream at about 10 Hz", () => {
    const emit = vi.fn();
    const throttle = createLatestThrottle<number>(emit, 100);

    for (let value = 0; value < 100; value += 1) {
      throttle.push(value);
      vi.advanceTimersByTime(10);
    }

    expect(emit.mock.calls.length).toBeGreaterThanOrEqual(10);
    expect(emit.mock.calls.length).toBeLessThanOrEqual(11);
    expect(emit).toHaveBeenLastCalledWith(99);
  });

  it("caps a sustained 100 Hz stream at about 5 Hz", () => {
    const emit = vi.fn();
    const throttle = createLatestThrottle<number>(emit, 200);

    for (let value = 0; value < 100; value += 1) {
      throttle.push(value);
      vi.advanceTimersByTime(10);
    }

    expect(emit.mock.calls.length).toBeGreaterThanOrEqual(5);
    expect(emit.mock.calls.length).toBeLessThanOrEqual(6);
    expect(emit).toHaveBeenLastCalledWith(99);
  });

  it("uses the latest budget when the display mode changes", () => {
    const emit = vi.fn();
    let intervalMs = 100;
    const throttle = createLatestThrottle<number>(emit, () => intervalMs);

    throttle.push(1);
    intervalMs = 50;
    vi.advanceTimersByTime(50);
    throttle.push(2);

    expect(emit).toHaveBeenCalledTimes(2);
    expect(emit).toHaveBeenLastCalledWith(2);
  });

  it("drops a pending trailing value when cancelled", () => {
    const emit = vi.fn();
    const throttle = createLatestThrottle<number>(emit, 100);

    throttle.push(1);
    throttle.push(2);
    throttle.cancel();
    vi.advanceTimersByTime(100);

    expect(emit).toHaveBeenCalledTimes(1);
    expect(emit).toHaveBeenLastCalledWith(1);
  });
});
