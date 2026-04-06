import {useNavigate} from "react-router-dom";
import {useSettings} from "../hooks/useSettings";
import {useThemeMode} from "@mowglinext/common";
import {MapPage} from "../pages/MapPage";

export const MiniMap: React.FC<{height?: number}> = ({height = 180}) => {
    const {colors} = useThemeMode();
    const navigate = useNavigate();
    const {settings} = useSettings();

    const datumLon = parseFloat(settings["datum_lon"] ?? "0");
    const datumLat = parseFloat(settings["datum_lat"] ?? "0");
    const hasDatum = !!(datumLon && datumLat) && isFinite(datumLon) && isFinite(datumLat);

    if (!hasDatum) {
        return (
            <div
                onClick={() => navigate("/map")}
                style={{
                    height,
                    borderRadius: 12,
                    background: colors.bgCard,
                    display: 'flex',
                    alignItems: 'center',
                    justifyContent: 'center',
                    cursor: 'pointer',
                    color: colors.textSecondary,
                    fontSize: 14,
                }}
            >
                Map — waiting for GPS datum
            </div>
        );
    }

    return (
        <div
            onClick={() => navigate("/map")}
            style={{
                height,
                borderRadius: 12,
                overflow: "hidden",
                cursor: "pointer",
                position: "relative",
            }}
        >
            <MapPage compact />
            <div style={{
                position: 'absolute',
                inset: 0,
                zIndex: 1,
            }} />
            <div style={{
                position: 'absolute',
                bottom: 8,
                right: 8,
                background: 'rgba(0,0,0,0.5)',
                backdropFilter: 'blur(4px)',
                borderRadius: 6,
                padding: '2px 8px',
                fontSize: 11,
                color: colors.textSecondary,
                zIndex: 2,
            }}>
                Tap to expand
            </div>
        </div>
    );
};
