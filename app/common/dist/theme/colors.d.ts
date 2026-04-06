/**
 * OpenMower brand tokens
 * Source: https://xtech.github.io/design-openmower-branding/openmower-ai-tokens.json
 */
export type ThemeMode = 'light' | 'dark';
interface ColorTokens {
    bgBase: string;
    bgCard: string;
    bgElevated: string;
    bgSubtle: string;
    primary: string;
    primaryLight: string;
    primaryDark: string;
    primaryBg: string;
    accent: string;
    accentAmber: string;
    danger: string;
    dangerBg: string;
    warning: string;
    info: string;
    success: string;
    text: string;
    textSecondary: string;
    muted: string;
    border: string;
    borderSubtle: string;
    /** Glassmorphism panel background */
    glassBackground: string;
    glassBorder: string;
    glassShadow: string;
}
export declare function getColors(mode: ThemeMode): ColorTokens;
/** Default export for backwards compatibility — will be overridden by ThemeProvider */
export declare let COLORS: ColorTokens;
export declare function setColors(mode: ThemeMode): void;
export {};
//# sourceMappingURL=colors.d.ts.map