import React, { useEffect, useState } from "react";
import type { Map as MapboxMap } from "mapbox-gl";
import { drawLine, transpose } from "../../../utils/map";
import {AbsolutePose, LaserScan, Map as MapType, MarkerArray, ObstacleArray, Path, TrackedObstacle, LineFeatureBase, MowingFeature, MowerFeatureBase, PathFeature, useWS, useHighLevelStatus} from "@mowglinext/common";

interface UseMapStreamsOptions {
    editMap: boolean;
    settings: Record<string, string>;
    offsetX: number;
    offsetY: number;
    datum: [number, number, number];
    mowingToolWidth: number;
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
    mowingToolWidth,
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
                const line = drawLine(
                    offsetX,
                    offsetY,
                    datum,
                    pose.Pose?.Pose?.Position?.Y!!,
                    pose.Pose?.Pose?.Position?.X!!,
                    orientation
                );
                return {
                    ...oldFeatures,
                    mower: new MowerFeatureBase(mower_lonlat),
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

    const mowingPathStream = useWS<string>(
        () => {
            console.log({ message: "Mowing PATH Stream closed" });
        },
        () => {
            console.log({ message: "Mowing PATH Stream connected" });
        },
        (e) => {
            const mowingPaths = JSON.parse(e) as Path[];
            setFeatures((oldFeatures) => {
                const newFeatures = { ...oldFeatures };
                mowingPaths.forEach((mowingPath, index) => {
                    if (mowingPath?.Poses) {
                        const line = mowingPath.Poses.map((pose) => {
                            return transpose(
                                offsetX,
                                offsetY,
                                datum,
                                pose.Pose?.Position?.Y!,
                                pose.Pose?.Position?.X!
                            );
                        });
                        newFeatures["mowingPath-" + index.toString()] = new PathFeature(
                            "mowingPath-" + index.toString(),
                            line,
                            `rgba(107, 255, 188, 0.68)`,
                            mowingToolWidth
                        );
                    }
                });
                return newFeatures;
            });
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
            mowingPathStream.stop();
            lidarStream.stop();
            obstaclesStream.stop();
            highLevelStatus.stop();
            setPath(undefined);
            setPlan(undefined);
            setLidarCollection({ type: "FeatureCollection", features: [] });
        } else {
            if (
                settings["datum_lon"] == undefined ||
                settings["datum_lat"] == undefined
            ) {
                return;
            }
            highLevelStatus.start("/api/openmower/subscribe/highLevelStatus");
            poseStream.start("/api/openmower/subscribe/pose");
            mapStream.start("/api/openmower/subscribe/map");
            pathStream.start("/api/openmower/subscribe/path");
            planStream.start("/api/openmower/subscribe/plan");
            mowingPathStream.start("/api/openmower/subscribe/mowingPath");
            lidarStream.start("/api/openmower/subscribe/lidar");
            obstaclesStream.start("/api/openmower/subscribe/obstacles");
        }
    }, [editMap]);

    // Start joy stream on AREA_RECORDING state
    useEffect(() => {
        if (highLevelStatus.highLevelStatus.StateName == "AREA_RECORDING") {
            joyStream.start("/api/openmower/publish/joy");
            setEditMap(false);
            return;
        }
        joyStream.stop();
    }, [highLevelStatus.highLevelStatus.StateName]);

    // Restart all streams on settings change
    useEffect(() => {
        if (
            settings["datum_lon"] == undefined ||
            settings["datum_lat"] == undefined
        ) {
            return;
        }
        highLevelStatus.start("/api/openmower/subscribe/highLevelStatus");
        poseStream.start("/api/openmower/subscribe/pose");
        mapStream.start("/api/openmower/subscribe/map");
        pathStream.start("/api/openmower/subscribe/path");
        planStream.start("/api/openmower/subscribe/plan");
        mowingPathStream.start("/api/openmower/subscribe/mowingPath");
        lidarStream.start("/api/openmower/subscribe/lidar");
        obstaclesStream.start("/api/openmower/subscribe/obstacles");
    }, [settings]);

    // Cleanup all streams on unmount
    useEffect(() => {
        return () => {
            poseStream.stop();
            mapStream.stop();
            pathStream.stop();
            joyStream.stop();
            planStream.stop();
            mowingPathStream.stop();
            lidarStream.stop();
            obstaclesStream.stop();
            highLevelStatus.stop();
        };
    }, []);

    return {
        map,
        dynamicObstacles,
        setMap,
        path,
        plan,
        lidarCollection,
        highLevelStatus,
        joyStream,
    };
}
