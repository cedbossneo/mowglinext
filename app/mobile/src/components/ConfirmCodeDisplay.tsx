import { useThemeMode } from '@/theme/ThemeProvider';

interface ConfirmCodeDisplayProps {
  code: string;
}

export function ConfirmCodeDisplay({ code }: ConfirmCodeDisplayProps) {
  const { colors } = useThemeMode();
  const digits = code.padStart(4, '0').slice(0, 4).split('');

  return (
    <div style={{ display: 'flex', gap: 12, justifyContent: 'center' }}>
      {digits.map((digit, i) => (
        <div
          key={i}
          style={{
            width: 64,
            height: 80,
            display: 'flex',
            alignItems: 'center',
            justifyContent: 'center',
            background: colors.bgElevated,
            borderRadius: 16,
            border: `2px solid ${colors.primary}`,
            fontSize: 40,
            fontWeight: 700,
            color: colors.primary,
            fontVariantNumeric: 'tabular-nums',
          }}
        >
          {digit}
        </div>
      ))}
    </div>
  );
}
