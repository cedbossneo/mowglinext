import {describe, it, expect, vi, beforeEach, afterEach} from 'vitest';
import {renderHook, act} from '@testing-library/react';
import {useManualMode} from './useManualMode.ts';

describe('useManualMode', () => {
    let mowerAction: (action: string, params: Record<string, unknown>) => () => Promise<void>;
    let sendJsonMessage: (msg: unknown) => void;
    let startStream: (uri: string) => void;

    beforeEach(() => {
        mowerAction = vi.fn(() => vi.fn().mockResolvedValue(undefined));
        sendJsonMessage = vi.fn();
        startStream = vi.fn();
        vi.useFakeTimers();
    });

    afterEach(() => {
        vi.useRealTimers();
    });

    function renderManualMode() {
        return renderHook(() =>
            useManualMode({
                mowerAction,
                joyStream: {sendJsonMessage, start: startStream},
            })
        );
    }

    it('starts with manual mode off', () => {
        const {result} = renderManualMode();
        expect(result.current.manualMode).toBeUndefined();
    });

    it('handleManualMode activates manual mode', async () => {
        const {result} = renderManualMode();
        await act(async () => {
            await result.current.handleManualMode();
        });
        expect(startStream).toHaveBeenCalledWith('/api/mowglinext/publish/joy');
        expect(mowerAction).toHaveBeenCalledWith('high_level_control', {Command: 7});
        expect(mowerAction).toHaveBeenCalledWith('mow_enabled', {MowEnabled: 1, MowDirection: 0});
        expect(result.current.manualMode).toBeDefined();
    });

    it('handleStopManualMode deactivates manual mode', async () => {
        const {result} = renderManualMode();
        await act(async () => {
            await result.current.handleManualMode();
        });
        expect(result.current.manualMode).toBeDefined();

        await act(async () => {
            await result.current.handleStopManualMode();
        });
        expect(mowerAction).toHaveBeenCalledWith('high_level_control', {Command: 2});
        expect(mowerAction).toHaveBeenCalledWith('mow_enabled', {MowEnabled: 0, MowDirection: 0});
        expect(result.current.manualMode).toBeUndefined();
    });

    it('handleJoyMove sends twist message', () => {
        const {result} = renderManualMode();
        act(() => {
            result.current.handleJoyMove({x: 0.5, y: 0.8} as any);
        });
        expect(sendJsonMessage).toHaveBeenCalledWith({
            Linear: {X: 0.8, Y: 0, Z: 0},
            Angular: {Z: -0.5, X: 0, Y: 0},
        });
    });

    it('handleJoyStop sends zero velocity', () => {
        const {result} = renderManualMode();
        act(() => {
            result.current.handleJoyStop();
        });
        expect(sendJsonMessage).toHaveBeenCalledWith({
            Linear: {X: 0, Y: 0, Z: 0},
            Angular: {Z: 0, X: 0, Y: 0},
        });
    });

    it('handleJoyMove handles null x/y', () => {
        const {result} = renderManualMode();
        act(() => {
            result.current.handleJoyMove({x: null, y: null} as any);
        });
        expect(sendJsonMessage).toHaveBeenCalledWith({
            Linear: {X: 0, Y: 0, Z: 0},
            Angular: {Z: -0, X: 0, Y: 0},
        });
    });
});
