import {InputNumber} from "antd";
import {useThemeMode} from "@mowglinext/common";

interface MapOffsetPanelProps {
    offsetX: number;
    offsetY: number;
    onChangeX: (v: number) => void;
    onChangeY: (v: number) => void;
}

export const MapOffsetPanel = ({offsetX, offsetY, onChangeX, onChangeY}: MapOffsetPanelProps) => {
    const {colors} = useThemeMode();
    return (
    <div>
        <div style={{
            fontSize: 12,
            fontWeight: 600,
            color: colors.muted,
            textTransform: 'uppercase' as const,
            letterSpacing: '0.05em',
            marginBottom: 6,
        }}>
            Map Offset
        </div>
        <div style={{display: 'flex', gap: 8}}>
            <div style={{flex: 1}}>
                <label style={{fontSize: 11, color: colors.textSecondary, display: 'block', marginBottom: 2}}>X</label>
                <InputNumber size="small" value={offsetX} onChange={(v) => onChangeX(v ?? 0)} min={-30} max={30} step={0.01} style={{width: '100%'}}/>
            </div>
            <div style={{flex: 1}}>
                <label style={{fontSize: 11, color: colors.textSecondary, display: 'block', marginBottom: 2}}>Y</label>
                <InputNumber size="small" value={offsetY} onChange={(v) => onChangeY(v ?? 0)} min={-30} max={30} step={0.01} style={{width: '100%'}}/>
            </div>
        </div>
    </div>
    );
};
