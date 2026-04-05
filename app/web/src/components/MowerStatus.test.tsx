import {describe, it, expect, vi} from 'vitest';
import {render, screen} from '@testing-library/react';
import {App} from 'antd';

/**
 * MowerStatus lives in @mowglinext/common and internally uses hooks
 * (useHighLevelStatus, etc.) that rely on ConnectionContext + useWS.
 *
 * Since vi.mock() on @mowglinext/common cannot intercept the component's
 * internal imports, we mock the underlying react-use-websocket and provide
 * a ConnectionContext with valid URLs so that the hooks initialise cleanly.
 */

// Mock react-use-websocket so no real WS connections are attempted
vi.mock('react-use-websocket', () => ({
    __esModule: true,
    default: () => ({
        sendJsonMessage: vi.fn(),
        lastMessage: null,
        readyState: 1,
        getWebSocket: () => null,
    }),
}));

import {MowerStatus, ConnectionProvider, type ConnectionContextValue} from '@mowglinext/common';

const testConnection: ConnectionContextValue = {
    baseHttpUrl: 'http://localhost:4006',
    baseWsUrl: 'ws://localhost:4006',
    authToken: null,
    isConnected: true,
    robotId: null,
    setConnection: vi.fn(),
};

function Wrapper({children}: {children: React.ReactNode}) {
    return (
        <ConnectionProvider value={testConnection}>
            <App>{children}</App>
        </ConnectionProvider>
    );
}

describe('MowerStatus', () => {
    it('renders without crashing', () => {
        render(<MowerStatus/>, {wrapper: Wrapper});
        // With mocked WS, hooks return default empty objects
        // so status shows as "Offline" / "Unknown" with 0% values
        expect(screen.getByText('Offline')).toBeInTheDocument();
        expect(screen.getAllByText('0%')).toHaveLength(2);
    });

    it('handles undefined values gracefully', () => {
        render(<MowerStatus/>, {wrapper: Wrapper});
        // Default state: no data from hooks
        expect(screen.getByText('Offline')).toBeInTheDocument();
    });
});
