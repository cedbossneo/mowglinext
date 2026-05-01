export type ThemeMode = 'light' | 'dark';

export interface ColorTokens {
  bgBase: string;
  bgCard: string;
  bgElevated: string;
  bgSubtle: string;
  primary: string;
  primaryLight: string;
  primaryDark: string;
  primaryBg: string;
  accent: string;
  danger: string;
  dangerBg: string;
  warning: string;
  info: string;
  success: string;
  text: string;
  textSecondary: string;
  textDim: string;
  textMuted: string;
  muted: string;
  border: string;
  borderSubtle: string;
  panel: string;
  panelHi: string;
  accentSoft: string;
  sky: string;
  skySoft: string;
  amber: string;
  amberSoft: string;
}

const DARK: ColorTokens = {
  bgBase: '#0F1210',
  bgCard: '#1A201C',
  bgElevated: '#1F2721',
  bgSubtle: '#151A17',
  primary: '#3EE084',
  primaryLight: '#52E897',
  primaryDark: '#2FC56E',
  primaryBg: 'rgba(62, 224, 132, 0.12)',
  accent: '#3EE084',
  danger: '#FF6B6B',
  dangerBg: 'rgba(255, 107, 107, 0.14)',
  warning: '#FFC567',
  info: '#7BC6FF',
  success: '#3EE084',
  text: '#ECF3EE',
  textSecondary: 'rgba(236, 243, 238, 0.60)',
  textDim: 'rgba(236, 243, 238, 0.60)',
  textMuted: 'rgba(236, 243, 238, 0.38)',
  muted: 'rgba(236, 243, 238, 0.40)',
  border: 'rgba(255, 255, 255, 0.07)',
  borderSubtle: 'rgba(255, 255, 255, 0.04)',
  panel: '#1A201C',
  panelHi: '#1F2721',
  accentSoft: 'rgba(62, 224, 132, 0.12)',
  sky: '#7BC6FF',
  skySoft: 'rgba(123, 198, 255, 0.14)',
  amber: '#FFC567',
  amberSoft: 'rgba(255, 197, 103, 0.14)',
};

const LIGHT: ColorTokens = {
  bgBase: '#FAFAF7',
  bgCard: '#FFFFFF',
  bgElevated: '#F2F2EF',
  bgSubtle: '#EDEDEA',
  primary: '#1B9D52',
  primaryLight: '#2CC76B',
  primaryDark: '#14853F',
  primaryBg: 'rgba(27, 157, 82, 0.08)',
  accent: '#1B9D52',
  danger: '#C93020',
  dangerBg: 'rgba(201, 48, 32, 0.08)',
  warning: '#F5A523',
  info: '#1565C0',
  success: '#1B9D52',
  text: '#141614',
  textSecondary: 'rgba(20, 22, 20, 0.62)',
  textDim: 'rgba(20, 22, 20, 0.62)',
  textMuted: 'rgba(20, 22, 20, 0.40)',
  muted: '#9E9E9E',
  border: 'rgba(0, 0, 0, 0.08)',
  borderSubtle: 'rgba(0, 0, 0, 0.05)',
  panel: '#FFFFFF',
  panelHi: '#F6F6F3',
  accentSoft: 'rgba(27, 157, 82, 0.10)',
  sky: '#3A8FD9',
  skySoft: 'rgba(58, 143, 217, 0.10)',
  amber: '#E8A028',
  amberSoft: 'rgba(232, 160, 40, 0.12)',
};

export function getColors(mode: ThemeMode): ColorTokens {
  return mode === 'dark' ? DARK : LIGHT;
}
