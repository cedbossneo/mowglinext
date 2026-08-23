import {beforeEach, describe, expect, it} from 'vitest';
import {fireEvent, render, screen, waitFor} from '@testing-library/react';
import {ThemeProvider, useThemeMode} from './ThemeContext.tsx';

function DisplayModeProbe() {
    const {displayMode, setDisplayMode} = useThemeMode();

    return (
        <>
            <output data-testid="display-mode">{displayMode}</output>
            <button onClick={() => setDisplayMode('visual')}>Visual</button>
        </>
    );
}

describe('ThemeProvider display mode', () => {
    beforeEach(() => {
        const storage = new Map<string, string>();
        Object.defineProperty(window, 'localStorage', {
            configurable: true,
            value: {
                getItem: (key: string) => storage.get(key) ?? null,
                setItem: (key: string, value: string) => storage.set(key, value),
                removeItem: (key: string) => storage.delete(key),
                clear: () => storage.clear(),
            },
        });
        document.documentElement.removeAttribute('data-display-mode');
    });

    it('uses Balanced by default and writes it to the document', async () => {
        render(<ThemeProvider><DisplayModeProbe/></ThemeProvider>);

        expect(screen.getByTestId('display-mode')).toHaveTextContent('balanced');
        await waitFor(() => expect(document.documentElement.dataset.displayMode).toBe('balanced'));
    });

    it('restores and persists a selected display mode', async () => {
        window.localStorage.setItem('mowgli.display-mode', 'efficient');
        render(<ThemeProvider><DisplayModeProbe/></ThemeProvider>);

        expect(screen.getByTestId('display-mode')).toHaveTextContent('efficient');
        fireEvent.click(screen.getByRole('button', {name: 'Visual'}));

        await waitFor(() => expect(document.documentElement.dataset.displayMode).toBe('visual'));
        expect(window.localStorage.getItem('mowgli.display-mode')).toBe('visual');
    });
});
