export type ColorRGBA = {
    R: number;
    G: number;
    B: number;
    A: number;
};
export type Joy = {
    Axes?: number[];
    Buttons?: number[];
};
export type Marker = {
    Ns: string;
    Id: number;
    Type: number;
    Action: number;
    Pose: Pose;
    Scale: Vector3;
    Color: ColorRGBA;
    Lifetime: number;
    FrameLocked: boolean;
    Points: Point[];
    Colors: ColorRGBA[];
    Text: string;
    MeshResource: string;
    MeshUseEmbeddedMaterials: boolean;
};
export type PoseStamped = {
    Pose?: Pose;
};
export type Path = {
    Poses?: PoseStamped[];
};
export type MarkerArray = {
    Markers: Marker[];
};
export type Point32 = {
    X?: number;
    Y?: number;
    Z?: number;
};
export type Twist = {
    Linear?: Vector3;
    Angular?: Vector3;
};
export type Polygon = {
    Points?: Point32[];
};
export type MapArea = {
    Name?: string;
    Area?: Polygon;
    Obstacles?: Polygon[];
};
export type Map = {
    MapWidth?: number;
    MapHeight?: number;
    MapCenterX?: number;
    MapCenterY?: number;
    NavigationAreas?: MapArea[];
    WorkingArea?: MapArea[];
    DockX?: number;
    DockY?: number;
    DockHeading?: number;
};
export type TrackedObstacle = {
    Id?: number;
    Polygon?: Polygon;
    Centroid?: Point;
    Radius?: number;
    ObservationCount?: number;
    Status?: number;
};
export type ObstacleArray = {
    Obstacles?: TrackedObstacle[];
};
export type WheelTick = {
    WheelTickFactor?: number;
    ValidWheels?: number;
    WheelDirectionFl?: number;
    WheelTicksFl?: number;
    WheelDirectionFr?: number;
    WheelTicksFr?: number;
    WheelDirectionRl?: number;
    WheelTicksRl?: number;
    WheelDirectionRr?: number;
    WheelTicksRr?: number;
};
export type Status = {
    MowerStatus?: number;
    RaspberryPiPower?: boolean;
    IsCharging?: boolean;
    EscPower?: boolean;
    RainDetected?: boolean;
    SoundModuleAvailable?: boolean;
    SoundModuleBusy?: boolean;
    UiBoardAvailable?: boolean;
    MowEnabled?: boolean;
    MowerEscStatus?: number;
    MowerEscTemperature?: number;
    MowerEscCurrent?: number;
    MowerMotorTemperature?: number;
    MowerMotorRpm?: number;
};
export type Power = {
    VCharge?: number;
    VBattery?: number;
    ChargeCurrent?: number;
    ChargerEnabled?: boolean;
    ChargerStatus?: string;
};
export type Emergency = {
    ActiveEmergency?: boolean;
    LatchedEmergency?: boolean;
    Reason?: string;
};
export type DockingSensor = {
    DockPresent?: boolean;
    DockDistance?: number;
};
export type Point = {
    X?: number;
    Y?: number;
    Z?: number;
};
export type Quaternion = {
    X?: number;
    Y?: number;
    Z?: number;
    W?: number;
};
export type Pose = {
    Position?: Point;
    Orientation?: Quaternion;
};
export type PoseWithCovariance = {
    Pose?: Pose;
    Covariance?: number[];
};
export type Vector3 = {
    X?: number;
    Y?: number;
    Z?: number;
};
export type HighLevelStatus = {
    State?: number;
    StateName?: string;
    SubStateName?: string;
    GpsQualityPercent?: number;
    BatteryPercent?: number;
    IsCharging?: boolean;
    Emergency?: boolean;
    CurrentArea?: number;
    CurrentPath?: number;
    CurrentPathIndex?: number;
};
export declare const enum AbsolutePoseFlags {
    RTK = 1,
    FIXED = 2,
    FLOAT = 4,
    DEAD_RECKONING = 8
}
export type AbsolutePose = {
    SensorStamp?: number;
    ReceivedStamp?: number;
    Source?: number;
    Flags?: AbsolutePoseFlags;
    OrientationValid?: number;
    MotionVectorValid?: number;
    PositionAccuracy?: number;
    OrientationAccuracy?: number;
    Pose?: PoseWithCovariance;
    MotionVector?: Vector3;
    VehicleHeading?: number;
    MotionHeading?: number;
};
export type LaserScan = {
    AngleMin?: number;
    AngleMax?: number;
    AngleIncrement?: number;
    RangeMin?: number;
    RangeMax?: number;
    Ranges?: number[];
};
export type Imu = {
    Orientation?: Quaternion;
    OrientationCovariance?: number[];
    AngularVelocity?: Vector3;
    AngularVelocityCovariance?: number[];
    LinearAcceleration?: Vector3;
    LinearAccelerationCovariance?: number[];
};
//# sourceMappingURL=ros.d.ts.map