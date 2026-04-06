/**
 * OpenMower brand tokens
 * Source: https://xtech.github.io/design-openmower-branding/openmower-ai-tokens.json
 */
const LIGHT = {
    bgBase: '#FFFFFF',
    bgCard: '#FFFFFF',
    bgElevated: '#EFEFEF',
    bgSubtle: '#E5E5E5',
    primary: '#1B9D52',
    primaryLight: '#2CC76B',
    primaryDark: '#14853F',
    primaryBg: 'rgba(27, 157, 82, 0.08)',
    accent: '#2CC76B',
    accentAmber: '#F5A523',
    danger: '#C93020',
    dangerBg: 'rgba(201, 48, 32, 0.08)',
    warning: '#F5A523',
    info: '#1565C0',
    success: '#1B9D52',
    text: '#1A1A1A',
    textSecondary: '#555555',
    muted: '#9E9E9E',
    border: '#E0E0E0',
    borderSubtle: '#CACACA',
    glassBackground: 'rgba(255, 255, 255, 0.85)',
    glassBorder: '1px solid rgba(0, 0, 0, 0.08)',
    glassShadow: '0 2px 12px rgba(0, 0, 0, 0.12)',
};
const DARK = {
    bgBase: '#141414',
    bgCard: '#1f1f1f',
    bgElevated: '#2a2a2a',
    bgSubtle: '#1a1a1a',
    primary: '#2CC76B',
    primaryLight: '#3DD97E',
    primaryDark: '#1EA856',
    primaryBg: 'rgba(44, 199, 107, 0.08)',
    accent: '#2CC76B',
    accentAmber: '#F5A523',
    danger: '#ff4d4f',
    dangerBg: 'rgba(255, 77, 79, 0.08)',
    warning: '#faad14',
    info: '#1677ff',
    success: '#2CC76B',
    text: '#e8e8e8',
    textSecondary: '#8c8c8c',
    muted: '#a0a0a0',
    border: '#303030',
    borderSubtle: '#252525',
    glassBackground: 'rgba(20, 20, 20, 0.75)',
    glassBorder: '1px solid rgba(255, 255, 255, 0.08)',
    glassShadow: '0 8px 32px rgba(0, 0, 0, 0.4)',
};
export function getColors(mode) {
    return mode === 'dark' ? DARK : LIGHT;
}
/** Default export for backwards compatibility — will be overridden by ThemeProvider */
export let COLORS = LIGHT;
export function setColors(mode) {
    COLORS = getColors(mode);
}
