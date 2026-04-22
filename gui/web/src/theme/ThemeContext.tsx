import {createContext, useCallback, useContext, useEffect, useState} from "react";
import type {ThemeMode} from "./colors.ts";
import {getColors, setColors} from "./colors.ts";

interface ThemeContextValue {
    mode: ThemeMode;
    toggleMode: () => void;
    colors: ReturnType<typeof getColors>;
}

const ThemeContext = createContext<ThemeContextValue>({
    mode: 'light',
    toggleMode: () => {},
    colors: getColors('light'),
});

const STORAGE_KEY = 'mowglinext-theme-mode';

function getInitialMode(): ThemeMode {
    try {
        const stored = localStorage.getItem(STORAGE_KEY);
        if (stored === 'light' || stored === 'dark') return stored;
    } catch { /* ignore */ }
    return 'light';
}

export function ThemeProvider({children}: {children: React.ReactNode}) {
    const [mode, setMode] = useState<ThemeMode>(getInitialMode);

    const colors = getColors(mode);

    useEffect(() => {
        setColors(mode);
        try { localStorage.setItem(STORAGE_KEY, mode); } catch { /* ignore */ }
        document.documentElement.style.background = colors.bgBase;
        document.body.style.background = colors.bgBase;
        document.body.style.fontFamily = "'Manrope', 'Inter', -apple-system, BlinkMacSystemFont, sans-serif";
        const meta = document.querySelector('meta[name="theme-color"]');
        if (meta) meta.setAttribute('content', colors.bgBase);
    }, [mode, colors.bgBase]);

    const toggleMode = useCallback(() => {
        setMode(prev => prev === 'light' ? 'dark' : 'light');
    }, []);

    return (
        <ThemeContext.Provider value={{mode, toggleMode, colors}}>
            {children}
        </ThemeContext.Provider>
    );
}

export function useThemeMode() {
    return useContext(ThemeContext);
}
