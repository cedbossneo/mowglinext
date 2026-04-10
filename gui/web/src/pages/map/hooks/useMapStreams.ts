import React, { useEffect, useState } from "react";
import type { Map as MapboxMap } from "mapbox-gl";
import { useWS } from "../../../hooks/useWS.ts";
import { useHighLevelStatus } from "../../../hooks/useHighLevelStatus.ts";
import {
    AbsolutePose,
    LaserScan,
    Map as MapType,
    MarkerArray,
    ObstacleArray,
    OccupancyGrid,
    Path,
    TrackedObstacle,
} from "../../../types/ros.ts";
import {
    LineFeatureBase,
    MowingFeature,
    MowerFeatureBase,
    MowerFootprintFeature,
    PathFeature,
} from "../../../types/map.ts";
import { drawLine, drawRobotFootprint, transpose } from "../../../utils/map.tsx";

interface UseMapStreamsOptions {
    editMap: boolean;
    settings: Record<string, string>;
    offsetX: number;
    offsetY: number;
    datum: [number, number, number];
    setFeatures: React.Dispatch<React.SetStateAction<Record<string, MowingFeature>>>;
    setEditMap: React.Dispatch<React.SetStateAction<boolean>>;
    setMapKey: React.Dispatch<React.SetStateAction<string>>;
    mapInstanceRef: React.RefObject<MapboxMap | null>;
    robotPoseRef: React.RefObject<{ x: number; y: number; heading: number } | null>;
}

export function useMapStreams({
    editMap,
    settings,
    offsetX,
    offsetY,
    datum,
    setFeatures,
    setEditMap,
    setMapKey,
    mapInstanceRef,
    robotPoseRef,
}: UseMapStreamsOptions) {
    const [map, setMap] = useState<MapType | undefined>(undefined);
    const [path, setPath] = useState<MarkerArray | undefined>(undefined);
    const [plan, setPlan] = useState<Path | undefined>(undefined);
    const [lidarCollection, setLidarCollection] = useState<GeoJSON.FeatureCollection>({
        type: "FeatureCollection",
        features: [],
    });
    const [dynamicObstacles, setDynamicObstacles] = useState<TrackedObstacle[]>([]);
    const [coverageCellsImage, setCoverageCellsImage] = useState<{
        url: string;
        coordinates: [[number, number], [number, number], [number, number], [number, number]];
    } | null>(null);

    const highLevelStatus = useHighLevelStatus();

    const poseStream = useWS<string>(
        () => {
            console.log({ message: "Pose Stream closed" });
        },
        () => {
            console.log({ message: "Pose Stream connected" });
        },
        (e) => {
            const pose = JSON.parse(e) as AbsolutePose;
            const mower_lonlat = transpose(
                offsetX,
                offsetY,
                datum,
                pose.Pose?.Pose?.Position?.Y!!,
                pose.Pose?.Pose?.Position?.X!!
            );
            robotPoseRef.current = {
                x: pose.Pose?.Pose?.Position?.X ?? 0,
                y: pose.Pose?.Pose?.Position?.Y ?? 0,
                heading: pose.MotionHeading ?? 0,
            };
            setFeatures((oldFeatures) => {
                const orientation = pose.MotionHeading!!;
                const posX = pose.Pose?.Pose?.Position?.X!!;
                const posY = pose.Pose?.Pose?.Position?.Y!!;
                const line = drawLine(offsetX, offsetY, datum, posY, posX, orientation);
                // Robot footprint from config (chassis dimensions)
                const ccx = parseFloat(settings["chassis_center_x"] ?? "0.18");
                const cl = parseFloat(settings["chassis_length"] ?? "0.54");
                const cw = parseFloat(settings["chassis_width"] ?? "0.40");
                const footprintRing = drawRobotFootprint(
                    offsetX, offsetY, datum, posY, posX, orientation,
                    ccx + cl / 2, ccx - cl / 2, cw / 2
                );
                return {
                    ...oldFeatures,
                    mower: new MowerFeatureBase(mower_lonlat),
                    ["mower-footprint"]: new MowerFootprintFeature(footprintRing),
                    ["mower-heading"]: new LineFeatureBase(
                        "mower-heading",
                        [mower_lonlat, line],
                        "#ff0000",
                        "heading"
                    ),
                };
            });
        }
    );

    const mapStream = useWS<string>(
        () => {
            console.log({ message: "MAP Stream closed" });
        },
        () => {
            console.log({ message: "MAP Stream connected" });
        },
        (e) => {
            const parse = JSON.parse(e) as MapType;
            if (console.debug) console.debug(parse);
            setMap(parse);
            setMapKey("live");
        }
    );

    const pathStream = useWS<string>(
        () => {
            console.log({ message: "PATH Stream closed" });
        },
        () => {
            console.log({ message: "PATH Stream connected" });
        },
        (e) => {
            const parse = JSON.parse(e) as MarkerArray;
            setPath(parse);
        }
    );

    const planStream = useWS<string>(
        () => {
            console.log({ message: "PLAN Stream closed" });
        },
        () => {
            console.log({ message: "PLAN Stream connected" });
        },
        (e) => {
            const parse = JSON.parse(e) as Path;
            setPlan(parse);
        }
    );

    const joyStream = useWS<string>(
        () => {
            console.log({ message: "Joystick Stream closed" });
        },
        () => {
            console.log({ message: "Joystick Stream connected" });
        },
        () => {}
    );

    const lidarStream = useWS<string>(
        () => {
            console.log({ message: "Lidar Stream closed" });
        },
        () => {
            console.log({ message: "Lidar Stream connected" });
        },
        (e) => {
            const scan = JSON.parse(e) as LaserScan;
            const pose = robotPoseRef.current;
            if (!pose || !scan.Ranges) return;

            const rays: GeoJSON.Feature[] = [];
            const angleMin = scan.AngleMin ?? 0;
            const angleInc = scan.AngleIncrement ?? 0;
            const rangeMin = scan.RangeMin ?? 0;
            const rangeMax = scan.RangeMax ?? 12;

            // Downsample: take every Nth point for performance
            const step = Math.max(1, Math.floor(scan.Ranges.length / 90));
            for (let i = 0; i < scan.Ranges.length; i += step) {
                const range = scan.Ranges[i];
                if (range < rangeMin || range > rangeMax) continue;

                const angle = angleMin + i * angleInc + pose.heading;
                const endX = pose.x + range * Math.cos(angle);
                const endY = pose.y + range * Math.sin(angle);
                const endLonLat = transpose(offsetX, offsetY, datum, endY, endX);

                rays.push({
                    type: "Feature",
                    properties: { intensity: range < rangeMax * 0.8 ? "hit" : "far" },
                    geometry: {
                        type: "Point",
                        coordinates: endLonLat,
                    },
                });
            }
            setLidarCollection({
                type: "FeatureCollection",
                features: rays,
            });
        }
    );

    const obstaclesStream = useWS<string>(
        () => {},
        () => { console.log({ message: "Obstacles Stream connected" }); },
        (e) => {
            const parsed = JSON.parse(e) as ObstacleArray;
            if (parsed.Obstacles) {
                // Only show persistent obstacles (status=1)
                setDynamicObstacles(parsed.Obstacles.filter(o => o.Status === 1));

                // Render obstacle polygons on the map
                setFeatures((oldFeatures) => {
                    const newFeatures = { ...oldFeatures };
                    // Remove old dynamic obstacle features
                    Object.keys(newFeatures).forEach(k => {
                        if (k.startsWith("dyn-obs-")) delete newFeatures[k];
                    });
                    // Add current obstacles as semi-transparent polygons
                    (parsed.Obstacles ?? []).filter(o => o.Status === 1).forEach((obs) => {
                        if (obs.Polygon?.Points && obs.Polygon.Points.length >= 3) {
                            const coords = obs.Polygon.Points.map(p =>
                                transpose(offsetX, offsetY, datum, p.Y ?? 0, p.X ?? 0)
                            );
                            // Close the polygon
                            coords.push(coords[0]);
                            newFeatures["dyn-obs-" + obs.Id] = new PathFeature(
                                "dyn-obs-" + obs.Id,
                                coords,
                                "rgba(255, 100, 100, 0.4)",
                                0.1
                            );
                        }
                    });
                    return newFeatures;
                });
            }
        }
    );

    const recordingTrajectoryStream = useWS<string>(
        () => {
            console.log({ message: "RecordingTrajectory Stream closed" });
        },
        () => {
            console.log({ message: "RecordingTrajectory Stream connected" });
        },
        (e) => {
            const path = JSON.parse(e) as Path;
            if (!path.Poses || path.Poses.length === 0) {
                // Recording cleared — remove trajectory feature
                setFeatures((oldFeatures) => {
                    const newFeatures = { ...oldFeatures };
                    delete newFeatures["recording-trajectory"];
                    return newFeatures;
                });
                return;
            }
            // Draw the recording trajectory as a line on the map
            const coords = path.Poses.map(p =>
                transpose(offsetX, offsetY, datum, p.Pose?.Position?.Y ?? 0, p.Pose?.Position?.X ?? 0)
            );
            setFeatures((oldFeatures) => ({
                ...oldFeatures,
                ["recording-trajectory"]: new PathFeature(
                    "recording-trajectory",
                    coords,
                    "#ff6600",
                    0.3,
                ),
            }));
        }
    );

    const coverageCellsStream = useWS<string>(
        () => {
            console.log({ message: "CoverageCells Stream closed" });
        },
        () => {
            console.log({ message: "CoverageCells Stream connected" });
        },
        (e) => {
            const grid = JSON.parse(e) as OccupancyGrid;
            if (!grid.Info || !grid.Data) return;

            const width = grid.Info.Width ?? 0;
            const height = grid.Info.Height ?? 0;
            const resolution = grid.Info.Resolution ?? 0.1;
            const originX = grid.Info.Origin?.Position?.X ?? 0;
            const originY = grid.Info.Origin?.Position?.Y ?? 0;

            if (width === 0 || height === 0) return;

            // Render OccupancyGrid to a canvas image
            const canvas = document.createElement("canvas");
            canvas.width = width;
            canvas.height = height;
            const ctx = canvas.getContext("2d");
            if (!ctx) return;

            const imageData = ctx.createImageData(width, height);
            for (let row = 0; row < height; row++) {
                for (let col = 0; col < width; col++) {
                    // OccupancyGrid row 0 = bottom, canvas row 0 = top -> flip vertically
                    const gridIdx = row * width + col;
                    const canvasIdx = ((height - 1 - row) * width + col) * 4;
                    const val = grid.Data[gridIdx];

                    if (val === 60) {
                        // To-mow: light green
                        imageData.data[canvasIdx] = 100;
                        imageData.data[canvasIdx + 1] = 220;
                        imageData.data[canvasIdx + 2] = 100;
                        imageData.data[canvasIdx + 3] = 140;
                    } else if (val === 100) {
                        // Obstacle: red
                        imageData.data[canvasIdx] = 255;
                        imageData.data[canvasIdx + 1] = 60;
                        imageData.data[canvasIdx + 2] = 60;
                        imageData.data[canvasIdx + 3] = 160;
                    } else {
                        // 0 (mowed), -1 (unknown), anything else: transparent
                        imageData.data[canvasIdx] = 0;
                        imageData.data[canvasIdx + 1] = 0;
                        imageData.data[canvasIdx + 2] = 0;
                        imageData.data[canvasIdx + 3] = 0;
                    }
                }
            }
            ctx.putImageData(imageData, 0, 0);

            // Compute geographic bounds: origin is bottom-left corner of the grid
            const gridWidth = width * resolution;
            const gridHeight = height * resolution;

            // Mapbox image source coordinates: [top-left, top-right, bottom-right, bottom-left]
            const topLeft = transpose(offsetX, offsetY, datum, originY + gridHeight, originX) as [number, number];
            const topRight = transpose(offsetX, offsetY, datum, originY + gridHeight, originX + gridWidth) as [number, number];
            const bottomRight = transpose(offsetX, offsetY, datum, originY, originX + gridWidth) as [number, number];
            const bottomLeft = transpose(offsetX, offsetY, datum, originY, originX) as [number, number];

            setCoverageCellsImage({
                url: canvas.toDataURL(),
                coordinates: [topLeft, topRight, bottomRight, bottomLeft],
            });
        }
    );

    // Keep lidar layer on top of draw layers
    useEffect(() => {
        const m = mapInstanceRef.current;
        if (!m) return;
        try {
            if (m.getLayer("lidar-points")) {
                m.moveLayer("lidar-points");
            }
        } catch { /* layer may not exist yet */ }
    }, [lidarCollection]);

    // Start/stop streams when editMap changes
    useEffect(() => {
        if (editMap) {
            mapStream.stop();
            poseStream.stop();
            pathStream.stop();
            planStream.stop();
            lidarStream.stop();
            obstaclesStream.stop();
            coverageCellsStream.stop();
            recordingTrajectoryStream.stop();
            highLevelStatus.stop();
            setPath(undefined);
            setPlan(undefined);
            setLidarCollection({ type: "FeatureCollection", features: [] });
            setCoverageCellsImage(null);
        } else {
            if (
                settings["datum_lon"] == undefined ||
                settings["datum_lat"] == undefined
            ) {
                return;
            }
            highLevelStatus.start("/api/mowglinext/subscribe/highLevelStatus");
            poseStream.start("/api/mowglinext/subscribe/pose");
            mapStream.start("/api/mowglinext/subscribe/map");
            pathStream.start("/api/mowglinext/subscribe/path");
            planStream.start("/api/mowglinext/subscribe/plan");
            lidarStream.start("/api/mowglinext/subscribe/lidar");
            obstaclesStream.start("/api/mowglinext/subscribe/obstacles");
            coverageCellsStream.start("/api/mowglinext/subscribe/coverageCells");
        }
    }, [editMap]);

    // Start joy + recording trajectory streams on RECORDING state
    useEffect(() => {
        const stateName = highLevelStatus.highLevelStatus.StateName;
        if (stateName === "RECORDING" || stateName === "AREA_RECORDING") {
            joyStream.start("/api/mowglinext/publish/joy");
            recordingTrajectoryStream.start("/api/mowglinext/subscribe/recordingTrajectory");
            setEditMap(false);
            return;
        }
        joyStream.stop();
        recordingTrajectoryStream.stop();
        // Clear trajectory feature when leaving recording mode
        setFeatures((oldFeatures) => {
            const newFeatures = { ...oldFeatures };
            delete newFeatures["recording-trajectory"];
            return newFeatures;
        });
    }, [highLevelStatus.highLevelStatus.StateName]);

    // Restart all streams on settings change
    useEffect(() => {
        if (
            settings["datum_lon"] == undefined ||
            settings["datum_lat"] == undefined
        ) {
            return;
        }
        highLevelStatus.start("/api/mowglinext/subscribe/highLevelStatus");
        poseStream.start("/api/mowglinext/subscribe/pose");
        mapStream.start("/api/mowglinext/subscribe/map");
        pathStream.start("/api/mowglinext/subscribe/path");
        planStream.start("/api/mowglinext/subscribe/plan");
        lidarStream.start("/api/mowglinext/subscribe/lidar");
        obstaclesStream.start("/api/mowglinext/subscribe/obstacles");
        coverageCellsStream.start("/api/mowglinext/subscribe/coverageCells");
    }, [settings]);

    // Cleanup all streams on unmount
    useEffect(() => {
        return () => {
            poseStream.stop();
            mapStream.stop();
            pathStream.stop();
            joyStream.stop();
            planStream.stop();
            lidarStream.stop();
            obstaclesStream.stop();
            coverageCellsStream.stop();
            recordingTrajectoryStream.stop();
            highLevelStatus.stop();
        };
    }, []);

    return {
        map,
        dynamicObstacles,
        coverageCellsImage,
        setMap,
        path,
        plan,
        lidarCollection,
        highLevelStatus,
        joyStream,
    };
}
