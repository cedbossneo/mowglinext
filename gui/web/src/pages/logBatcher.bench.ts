import { bench, describe } from "vitest";
import { appendCappedBatch } from "./logBatcher.ts";

const MAX_LINES = 5_000;
const existing = Array.from({ length: MAX_LINES }, (_, index) => index);
const incoming = Array.from({ length: 100 }, (_, index) => MAX_LINES + index);
let benchmarkResult = existing;

describe("log buffer append", () => {
  bench("100 line-by-line updates", () => {
    let result = existing;
    for (const line of incoming)
      result = appendCappedBatch(result, [line], MAX_LINES);
    benchmarkResult = result;
  });

  bench("one 100-line batch", () => {
    benchmarkResult = appendCappedBatch(existing, incoming, MAX_LINES);
  });
});

void benchmarkResult;
