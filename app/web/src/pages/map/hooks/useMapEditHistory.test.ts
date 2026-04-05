import {describe, it, expect, vi, beforeEach} from 'vitest';
import {renderHook, act} from '@testing-library/react';
import {useMapEditHistory} from './useMapEditHistory';
import {MowingFeature} from "@mowglinext/common";

// Mock antd App.useApp to provide modal.confirm
const mockConfirm = vi.fn(({onOk}: {onOk: () => void}) => onOk());
vi.mock('antd', async () => {
    const actual = await vi.importActual('antd');
    return {
        ...actual,
        App: {
            ...((actual as Record<string, unknown>).App as Record<string, unknown>),
            useApp: () => ({
                notification: {},
                message: {},
                modal: {confirm: mockConfirm},
            }),
        },
    };
});

describe('useMapEditHistory', () => {
    let features: Record<string, MowingFeature>;
    let setFeatures: (features: Record<string, MowingFeature> | ((prev: Record<string, MowingFeature>) => Record<string, MowingFeature>)) => void;
    let editMap: boolean;
    let setEditMap: (v: boolean) => void;

    beforeEach(() => {
        features = {};
        setFeatures = vi.fn((updater) => {
            if (typeof updater === 'function') {
                features = updater(features);
            } else {
                features = updater;
            }
        });
        editMap = false;
        setEditMap = vi.fn((val) => {
            editMap = val;
        });
    });

    function renderHistory() {
        return renderHook(() =>
            useMapEditHistory({features, setFeatures, editMap, setEditMap})
        );
    }

    it('initializes with no unsaved changes', () => {
        const {result} = renderHistory();
        expect(result.current.hasUnsavedChanges).toBe(false);
        expect(result.current.historyIndex).toBe(-1);
    });

    it('handleEditMap toggles edit mode on', () => {
        const {result} = renderHistory();
        act(() => {
            result.current.handleEditMap();
        });
        expect(setEditMap).toHaveBeenCalledWith(true);
    });

    it('setHasUnsavedChanges updates state', () => {
        const {result} = renderHistory();
        act(() => {
            result.current.setHasUnsavedChanges(true);
        });
        expect(result.current.hasUnsavedChanges).toBe(true);
    });
});
