import {describe, it, expect, vi, beforeEach} from 'vitest';
import {render, screen} from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import {createMemoryRouter, RouterProvider} from 'react-router-dom';
import Root from './root';

// Mock MowerStatus and hooks from common
vi.mock('@mowglinext/common', async () => {
    const actual = await vi.importActual('@mowglinext/common');
    return {
        ...actual,
        MowerStatus: () => <div data-testid="mower-status">Status</div>,
        useHighLevelStatus: () => ({highLevelStatus: {StateName: 'IDLE'}}),
    };
});

function renderWithRouter(initialPath = '/openmower') {
    const router = createMemoryRouter(
        [
            {
                path: '/',
                element: <Root/>,
                children: [
                    {path: 'openmower', element: <div>Dashboard Content</div>},
                    {path: 'map', element: <div>Map Content</div>},
                    {path: 'settings', element: <div>Settings Content</div>},
                    {path: 'schedule', element: <div>Schedule Content</div>},
                    {path: 'logs', element: <div>Logs Content</div>},
                    {path: 'onboarding', element: <div>Onboarding Content</div>},
                ],
            },
        ],
        {initialEntries: [initialPath]}
    );
    return render(<RouterProvider router={router}/>);
}

describe('Root layout', () => {
    beforeEach(() => {
        Object.defineProperty(window, 'innerWidth', {value: 1440, writable: true});
        window.matchMedia = vi.fn((query: string) => ({
            matches: query.includes('767') ? false : true,
            media: query,
            onchange: null,
            addListener: vi.fn(),
            removeListener: vi.fn(),
            addEventListener: vi.fn(),
            removeEventListener: vi.fn(),
            dispatchEvent: vi.fn(),
        })) as unknown as typeof window.matchMedia;
    });

    it('renders page title based on route', () => {
        renderWithRouter('/openmower');
        expect(screen.getAllByText('Dashboard').length).toBeGreaterThanOrEqual(1);
    });

    it('renders MowerStatus in header', () => {
        renderWithRouter('/openmower');
        expect(screen.getByTestId('mower-status')).toBeInTheDocument();
    });

    it('renders child route content', () => {
        renderWithRouter('/openmower');
        expect(screen.getByText('Dashboard Content')).toBeInTheDocument();
    });

    it('renders navigation menu items', () => {
        renderWithRouter('/openmower');
        // Nav rail icons have aria-labels (text only visible when expanded)
        expect(screen.getByLabelText('Dashboard')).toBeInTheDocument();
        expect(screen.getByLabelText('Map')).toBeInTheDocument();
        expect(screen.getByLabelText('Settings')).toBeInTheDocument();
        expect(screen.getByLabelText('Schedule')).toBeInTheDocument();
        expect(screen.getByLabelText('Logs')).toBeInTheDocument();
    });

    it('renders OpenMower branding when rail expanded', async () => {
        const user = userEvent.setup();
        renderWithRouter('/openmower');
        // Branding text is hidden when rail is collapsed
        expect(screen.queryByText('OpenMower')).not.toBeInTheDocument();
        // Hover over nav to expand rail and show branding
        const nav = screen.getByLabelText('Dashboard').closest('nav')!;
        await user.hover(nav);
        expect(screen.getByText('OpenMower')).toBeInTheDocument();
    });

    it('shows correct title for map page', () => {
        renderWithRouter('/map');
        expect(screen.getAllByText('Map').length).toBeGreaterThanOrEqual(1);
        expect(screen.getByText('Map Content')).toBeInTheDocument();
    });

    it('shows correct title for settings page', () => {
        renderWithRouter('/settings');
        expect(screen.getAllByText('Settings').length).toBeGreaterThanOrEqual(1);
        expect(screen.getByText('Settings Content')).toBeInTheDocument();
    });
});

describe('Root layout - mobile', () => {
    beforeEach(() => {
        Object.defineProperty(window, 'innerWidth', {value: 390, writable: true});
        window.matchMedia = vi.fn((query: string) => ({
            matches: query.includes('767') ? true : false,
            media: query,
            onchange: null,
            addListener: vi.fn(),
            removeListener: vi.fn(),
            addEventListener: vi.fn(),
            removeEventListener: vi.fn(),
            dispatchEvent: vi.fn(),
        })) as unknown as typeof window.matchMedia;
    });

    it('renders bottom navigation bar on mobile', () => {
        renderWithRouter('/openmower');
        expect(screen.getByLabelText('Home')).toBeInTheDocument();
        expect(screen.getByLabelText('Map')).toBeInTheDocument();
        expect(screen.getByLabelText('Schedule')).toBeInTheDocument();
        expect(screen.getByLabelText('Settings')).toBeInTheDocument();
    });

    it('renders hamburger menu on mobile', () => {
        renderWithRouter('/openmower');
        expect(screen.getByLabelText('Open menu')).toBeInTheDocument();
    });

    it('highlights active tab', () => {
        renderWithRouter('/openmower');
        const homeBtn = screen.getByLabelText('Home');
        expect(homeBtn.getAttribute('aria-current')).toBe('page');
    });

    it('opens slide-over menu when hamburger clicked', async () => {
        const user = userEvent.setup();
        renderWithRouter('/openmower');
        await user.click(screen.getByLabelText('Open menu'));
        expect(screen.getByLabelText('Close menu')).toBeInTheDocument();
    });
});
