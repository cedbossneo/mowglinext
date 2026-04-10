import React from 'react';
import {vi} from 'vitest';
import type {HighLevelStatus} from '../types/ros.ts';

// Mock high level status
export const defaultHighLevelStatus: HighLevelStatus = {
    StateName: 'IDLE',
    GpsQualityPercent: 0.95,
    BatteryPercent: 0.75,
    IsCharging: false,
    Emergency: false,
};

// Mock useHighLevelStatus hook
export const mockHighLevelStatus = vi.fn(() => ({
    highLevelStatus: defaultHighLevelStatus,
}));

// Mock useApi hook
export const mockGuiApi = {
    mowglinext: {
        callCreate: vi.fn().mockResolvedValue({}),
    },
    maps: {
        mapCreate: vi.fn().mockResolvedValue({}),
    },
    config: {
        configCreate: vi.fn().mockResolvedValue({}),
        configList: vi.fn().mockResolvedValue({data: {}}),
    },
};

export const mockUseApi = vi.fn(() => mockGuiApi);

// Mock useWS hook
export const createMockWS = () => ({
    start: vi.fn(),
    stop: vi.fn(),
    sendJsonMessage: vi.fn(),
});

export const mockUseWS = vi.fn(() => createMockWS());

// Mock notification
export const mockNotification = {
    success: vi.fn(),
    error: vi.fn(),
    info: vi.fn(),
    warning: vi.fn(),
};

// Mock App.useApp
export const mockUseApp = vi.fn(() => ({
    notification: mockNotification,
    message: {success: vi.fn(), error: vi.fn()},
    modal: {confirm: vi.fn()},
}));

// Mock react-router-dom
export const mockNavigate = vi.fn();
export const mockUseMatches = vi.fn(() => [
    {pathname: '/'},
    {pathname: '/mowglinext'},
]);

// Wrapper for rendering with providers
export function TestWrapper({children}: {children: React.ReactNode}) {
    return <>{children}</>;
}
