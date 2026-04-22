import {useState, useEffect} from "react";

/** Keep a rolling window of the last N values of a changing number. */
export function useTrail(value: number, length = 32): number[] {
  const [trail, setTrail] = useState<number[]>(() => Array(length).fill(value));
  useEffect(() => {
    setTrail(prev => {
      const next = prev.slice(1);
      next.push(value);
      return next;
    });
  }, [value]);
  return trail;
}
