import type { ThemeMode } from "./colors";
import { getColors } from "./colors";
interface ThemeContextValue {
    mode: ThemeMode;
    toggleMode: () => void;
    colors: ReturnType<typeof getColors>;
}
export declare function ThemeProvider({ children }: {
    children: React.ReactNode;
}): import("react/jsx-runtime").JSX.Element;
export declare function useThemeMode(): ThemeContextValue;
export {};
//# sourceMappingURL=ThemeContext.d.ts.map