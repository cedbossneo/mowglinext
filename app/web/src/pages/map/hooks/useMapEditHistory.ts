import {useCallback, useEffect, useRef, useState} from "react";
import {App} from "antd";
import {MowingFeature} from "@mowglinext/common";

interface UseMapEditHistoryOptions {
    features: Record<string, MowingFeature>;
    setFeatures: (features: Record<string, MowingFeature>) => void;
    editMap: boolean;
    setEditMap: (v: boolean) => void;
}

export function useMapEditHistory({features, setFeatures, editMap, setEditMap}: UseMapEditHistoryOptions) {
    const {modal} = App.useApp();
    const [editHistory, setEditHistory] = useState<Record<string, MowingFeature>[]>([]);
    const [historyIndex, setHistoryIndex] = useState(-1);
    const [hasUnsavedChanges, setHasUnsavedChanges] = useState(false);
    const skipHistoryRef = useRef(false);

    function exitEditMode() {
        setEditHistory([]);
        setHistoryIndex(-1);
        setHasUnsavedChanges(false);
        setEditMap(false);
    }

    function handleEditMap() {
        if (!editMap) {
            setEditHistory([{...features}]);
            setHistoryIndex(0);
            setEditMap(true);
        } else if (hasUnsavedChanges) {
            modal.confirm({
                title: 'Discard unsaved changes?',
                content: 'You have unsaved map changes. If you cancel now, all changes will be lost.',
                okText: 'Discard',
                okType: 'danger',
                cancelText: 'Keep editing',
                onOk: exitEditMode,
            });
        } else {
            exitEditMode();
        }
    }

    const pushHistory = useCallback((newFeatures: Record<string, MowingFeature>) => {
        setEditHistory(prev => {
            const truncated = prev.slice(0, historyIndex + 1);
            const next = [...truncated, newFeatures].slice(-10);
            setHistoryIndex(next.length - 1);
            return next;
        });
    }, [historyIndex]);

    // Track feature changes during edit mode
    useEffect(() => {
        if (!editMap) return;
        if (skipHistoryRef.current) {
            skipHistoryRef.current = false;
            return;
        }
        if (editHistory.length > 0 && editHistory[historyIndex] === features) return;
        pushHistory(features);
        setHasUnsavedChanges(true);
        // eslint-disable-next-line react-hooks/exhaustive-deps
    }, [features, editMap]);

    const handleUndo = useCallback(() => {
        if (historyIndex <= 0) return;
        const newIndex = historyIndex - 1;
        setHistoryIndex(newIndex);
        skipHistoryRef.current = true;
        setFeatures(editHistory[newIndex]);
    }, [historyIndex, editHistory, setFeatures]);

    const handleRedo = useCallback(() => {
        if (historyIndex >= editHistory.length - 1) return;
        const newIndex = historyIndex + 1;
        setHistoryIndex(newIndex);
        skipHistoryRef.current = true;
        setFeatures(editHistory[newIndex]);
    }, [historyIndex, editHistory, setFeatures]);

    return {
        hasUnsavedChanges,
        setHasUnsavedChanges,
        handleEditMap,
        exitEditMode,
        handleUndo,
        handleRedo,
        historyIndex,
        editHistory,
    };
}
