import { useEffect, useRef, useState } from "react";
import { Typography } from "antd";
import { usePose, useGPS, useThemeMode } from "@mowglinext/common";

export function MapScreen() {
  const { colors } = useThemeMode();
  const pose = usePose();
  const gps = useGPS();
  const mapContainerRef = useRef<HTMLDivElement>(null);
  const [mapError, setMapError] = useState<string | null>(null);

  // Position display fallback when map is not available
  const lat = gps.Pose?.Pose?.Position?.Y ?? 0;
  const lon = gps.Pose?.Pose?.Position?.X ?? 0;

  return (
    <div style={{ height: "100%", position: "relative" }}>
      <div
        ref={mapContainerRef}
        style={{ width: "100%", height: "100%", background: colors.bgSubtle }}
      >
        {/* Mapbox GL map will be initialized here */}
        <div
          style={{
            position: "absolute",
            bottom: 80,
            left: 16,
            right: 16,
            background: colors.glassBackground,
            backdropFilter: "blur(12px)",
            border: colors.glassBorder,
            borderRadius: 12,
            padding: 12,
            zIndex: 10,
          }}
        >
          <Typography.Text style={{ color: colors.text, fontSize: 12 }}>
            Position: {lat.toFixed(7)}, {lon.toFixed(7)}
          </Typography.Text>
          {pose.VehicleHeading != null && (
            <Typography.Text
              style={{ color: colors.textSecondary, fontSize: 12, marginLeft: 12 }}
            >
              Heading: {((pose.VehicleHeading * 180) / Math.PI).toFixed(1)}°
            </Typography.Text>
          )}
        </div>
      </div>
    </div>
  );
}
