import {
  createContext,
  useCallback,
  useContext,
  useEffect,
  useState,
  type ReactNode,
} from 'react';
import { getColors, type ThemeMode, type ColorTokens } from './colors';
import { Preferences } from '@capacitor/preferences';

interface ThemeContextValue {
  mode: ThemeMode;
  toggleMode: () => void;
  colors: ColorTokens;
}

const ThemeContext = createContext<ThemeContextValue>({
  mode: 'dark',
  toggleMode: () => undefined,
  colors: getColors('dark'),
});

const PREFS_KEY = 'mowgli-theme-mode';

function getSystemMode(): ThemeMode {
  if (typeof window !== 'undefined' && window.matchMedia) {
    return window.matchMedia('(prefers-color-scheme: dark)').matches
      ? 'dark'
      : 'light';
  }
  return 'dark';
}

export function ThemeProvider({ children }: { children: ReactNode }) {
  // Default to dark; load persisted preference asynchronously
  const [mode, setMode] = useState<ThemeMode>('dark');

  useEffect(() => {
    Preferences.get({ key: PREFS_KEY })
      .then(({ value }) => {
        if (value === 'light' || value === 'dark') {
          setMode(value);
        } else {
          setMode(getSystemMode());
        }
      })
      .catch(() => {
        setMode(getSystemMode());
      });
  }, []);

  const toggleMode = useCallback(() => {
    setMode((prev) => {
      const next: ThemeMode = prev === 'dark' ? 'light' : 'dark';
      Preferences.set({ key: PREFS_KEY, value: next }).catch(() => undefined);
      return next;
    });
  }, []);

  const colors = getColors(mode);

  useEffect(() => {
    document.documentElement.style.background = colors.bgBase;
    document.body.style.background = colors.bgBase;
    const meta = document.querySelector('meta[name="theme-color"]');
    if (meta) meta.setAttribute('content', colors.bgBase);
  }, [colors.bgBase]);

  return (
    <ThemeContext.Provider value={{ mode, toggleMode, colors }}>
      {children}
    </ThemeContext.Provider>
  );
}

export function useThemeMode(): ThemeContextValue {
  return useContext(ThemeContext);
}
