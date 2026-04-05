import {describe, it, expect, vi, beforeEach, afterEach} from 'vitest';
import {renderHook, act} from '@testing-library/react';
import {useMapOffset} from './useMapOffset';

describe('useMapOffset', () => {
    let setConfig: ReturnType<typeof vi.fn>;
    let notification: {error: ReturnType<typeof vi.fn>};

    beforeEach(() => {
        setConfig = vi.fn().mockResolvedValue(undefined);
        notification = {error: vi.fn()};
        vi.useFakeTimers();
    });

    afterEach(() => {
        vi.useRealTimers();
    });

    it('initializes offsets from config', () => {
        const config = {'gui.map.offset.x': '1.5', 'gui.map.offset.y': '-2.3'};
        const {result} = renderHook(() =>
            useMapOffset({config, setConfig, notification} as any)
        );
        expect(result.current.offsetX).toBe(1.5);
        expect(result.current.offsetY).toBe(-2.3);
    });

    it('defaults to 0 when config is empty', () => {
        const {result} = renderHook(() =>
            useMapOffset({config: {}, setConfig, notification} as any)
        );
        expect(result.current.offsetX).toBe(0);
        expect(result.current.offsetY).toBe(0);
    });

    it('handleOffsetX updates immediately and debounces save', async () => {
        const config = {'gui.map.offset.x': '0', 'gui.map.offset.y': '0'};
        const {result} = renderHook(() =>
            useMapOffset({config, setConfig, notification} as any)
        );

        act(() => {
            result.current.handleOffsetX(5.5);
        });
        expect(result.current.offsetX).toBe(5.5);
        expect(setConfig).not.toHaveBeenCalled();

        // After debounce timeout
        await act(async () => {
            vi.advanceTimersByTime(1100);
        });
        expect(setConfig).toHaveBeenCalledWith({'gui.map.offset.x': '5.5'});
    });

    it('handleOffsetY updates immediately and debounces save', async () => {
        const config = {'gui.map.offset.x': '0', 'gui.map.offset.y': '0'};
        const {result} = renderHook(() =>
            useMapOffset({config, setConfig, notification} as any)
        );

        act(() => {
            result.current.handleOffsetY(-3.2);
        });
        expect(result.current.offsetY).toBe(-3.2);

        await act(async () => {
            vi.advanceTimersByTime(1100);
        });
        expect(setConfig).toHaveBeenCalledWith({'gui.map.offset.y': '-3.2'});
    });

    it('ignores NaN config values', () => {
        const config = {'gui.map.offset.x': 'invalid', 'gui.map.offset.y': 'abc'};
        const {result} = renderHook(() =>
            useMapOffset({config, setConfig, notification} as any)
        );
        expect(result.current.offsetX).toBe(0);
        expect(result.current.offsetY).toBe(0);
    });

    it('debounces multiple rapid changes', async () => {
        const config = {'gui.map.offset.x': '0', 'gui.map.offset.y': '0'};
        const {result} = renderHook(() =>
            useMapOffset({config, setConfig, notification} as any)
        );

        act(() => {
            result.current.handleOffsetX(1);
            result.current.handleOffsetX(2);
            result.current.handleOffsetX(3);
        });

        await act(async () => {
            vi.advanceTimersByTime(1100);
        });

        // Only the last value should be saved
        expect(setConfig).toHaveBeenCalledTimes(1);
        expect(setConfig).toHaveBeenCalledWith({'gui.map.offset.x': '3'});
    });
});
