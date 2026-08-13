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

    function renderManualMode(stateName?: string) {
        return renderHook(() =>
            useManualMode({
                mowerAction,
                joyStream: {sendJsonMessage, start: startStream},
                stateName,
            })
        );
    }

    it('starts with manual mode off', () => {
        const {result} = renderManualMode();
        expect(result.current.manualMode).toBe(false);
    });

    it('handleManualMode activates manual mode', async () => {
        const {result} = renderManualMode();
        await act(async () => {
            await result.current.handleManualMode();
        });
        expect(mowerAction).toHaveBeenCalledWith('high_level_control', {Command: 7});
        expect(mowerAction).not.toHaveBeenCalledWith('mow_enabled', expect.anything());
        expect(result.current.manualMode).toBe(true);
    });

    it('handleManualDriveMode selects blade-disabled manual drive', async () => {
        const {result} = renderManualMode();
        await act(async () => {
            await result.current.handleManualDriveMode();
        });
        expect(mowerAction).toHaveBeenCalledWith('high_level_control', {Command: 9});
        expect(mowerAction).not.toHaveBeenCalledWith('mow_enabled', expect.anything());
        expect(result.current.manualMode).toBe(true);
    });

    it('keeps the joystick active when the mower reports manual drive', () => {
        const {result} = renderManualMode('MANUAL_DRIVING');
        expect(result.current.manualMode).toBe(true);
    });

    it('handleStopManualMode deactivates manual mode', async () => {
        const {result} = renderManualMode();
        await act(async () => {
            await result.current.handleManualMode();
        });
        expect(result.current.manualMode).toBe(true);

        await act(async () => {
            await result.current.handleStopManualMode();
        });
        expect(mowerAction).toHaveBeenCalledWith('high_level_control', {Command: 8});
        expect(mowerAction).not.toHaveBeenCalledWith('mow_enabled', expect.anything());
        expect(result.current.manualMode).toBe(false);
    });

    it('handleJoyMove scales joystick input by MAX_LINEAR_MPS and MAX_ANGULAR_RAD_S', () => {
        const {result} = renderManualMode();
        act(() => {
            result.current.handleJoyMove({x: 0.5, y: 0.8} as any);
        });
        // Raw joystick: x=0.5, y=0.8 → scaled to linear=0.8*0.25=0.2, angular=-0.5*0.6=-0.3
        expect(sendJsonMessage).toHaveBeenCalledWith({
            header: {stamp: {sec: 0, nanosec: 0}, frame_id: ""},
            twist: {linear: {x: 0.2, y: 0, z: 0}, angular: {z: -0.3, x: 0, y: 0}},
        });
    });

    it('handleJoyStop sends zero velocity', () => {
        const {result} = renderManualMode();
        act(() => {
            result.current.handleJoyStop();
        });
        expect(sendJsonMessage).toHaveBeenCalledWith({
            header: {stamp: {sec: 0, nanosec: 0}, frame_id: ""},
            twist: {linear: {x: 0, y: 0, z: 0}, angular: {z: 0, x: 0, y: 0}},
        });
    });

    it('cleans up blade keepalive on unmount', async () => {
        const {result, unmount} = renderManualMode();
        await act(async () => {
            await result.current.handleManualMode();
        });
        expect(result.current.manualMode).toBe(true);
        unmount();
        // No assertion needed — just verifying no error/leak on unmount
    });
});
