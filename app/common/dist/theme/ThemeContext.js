import { jsx as _jsx } from "react/jsx-runtime";
import { createContext, useCallback, useContext, useEffect, useState } from "react";
import { getColors, setColors } from "./colors";
const ThemeContext = createContext({
    mode: 'light',
    toggleMode: () => { },
    colors: getColors('light'),
});
const STORAGE_KEY = 'openmower-theme-mode';
function getInitialMode() {
    try {
        const stored = localStorage.getItem(STORAGE_KEY);
        if (stored === 'light' || stored === 'dark')
            return stored;
    }
    catch { /* ignore */ }
    return 'light';
}
export function ThemeProvider({ children }) {
    const [mode, setMode] = useState(getInitialMode);
    const colors = getColors(mode);
    useEffect(() => {
        setColors(mode);
        try {
            localStorage.setItem(STORAGE_KEY, mode);
        }
        catch { /* ignore */ }
        document.documentElement.style.background = colors.bgBase;
        document.body.style.background = colors.bgBase;
        const meta = document.querySelector('meta[name="theme-color"]');
        if (meta)
            meta.setAttribute('content', colors.bgBase);
    }, [mode, colors.bgBase]);
    const toggleMode = useCallback(() => {
        setMode(prev => prev === 'light' ? 'dark' : 'light');
    }, []);
    return (_jsx(ThemeContext.Provider, { value: { mode, toggleMode, colors }, children: children }));
}
export function useThemeMode() {
    return useContext(ThemeContext);
}
