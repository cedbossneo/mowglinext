export interface ApiContainer {
    id?: string;
    labels?: Record<string, string>;
    names?: string[];
    state?: string;
}
export interface ApiContainerListResponse {
    containers?: ApiContainer[];
}
export interface ApiErrorResponse {
    error?: string;
}
export interface ApiGetConfigResponse {
    tileUri?: string;
}
export interface ApiGetSettingsResponse {
    settings?: Record<string, string>;
}
export interface ApiOkResponse {
    ok?: string;
}
export interface GeometryMsgsPoint {
    "msg.Package"?: number;
    x?: number;
    y?: number;
    z?: number;
}
export interface GeometryMsgsPoint32 {
    "msg.Package"?: number;
    x?: number;
    y?: number;
    z?: number;
}
export interface GeometryMsgsPolygon {
    "msg.Package"?: number;
    points?: GeometryMsgsPoint32[];
}
export interface GeometryMsgsPose {
    "msg.Package"?: number;
    orientation?: GeometryMsgsQuaternion;
    position?: GeometryMsgsPoint;
}
export interface GeometryMsgsQuaternion {
    "msg.Package"?: number;
    w?: number;
    x?: number;
    y?: number;
    z?: number;
}
export interface MowerMapAddMowingAreaSrvReq {
    area?: MowerMapMapArea;
    isNavigationArea?: boolean;
    "msg.Package"?: number;
}
export interface MowerMapReplaceArea {
    area?: MowerMapMapArea;
    isNavigationArea?: boolean;
}
export interface MowerReplaceMapSrvReq {
    areas: MowerMapReplaceArea[];
    "msg.Package"?: number;
}
export interface MowerMapMapArea {
    area?: GeometryMsgsPolygon;
    "msg.Package"?: number;
    name?: string;
    obstacles?: GeometryMsgsPolygon[];
}
export interface MowerMapSetDockingPointSrvReq {
    dockingPose?: GeometryMsgsPose;
    "msg.Package"?: number;
}
export interface TypesFirmwareConfig {
    batChargeCutoffVoltage?: number;
    boardType?: string;
    bothWheelsLiftEmergencyMillis?: number;
    branch?: string;
    debugType?: string;
    disableEmergency?: boolean;
    externalImuAcceleration?: boolean;
    externalImuAngular?: boolean;
    file?: string;
    limitVoltage150MA?: number;
    masterJ18?: boolean;
    maxChargeCurrent?: number;
    maxChargeVoltage?: number;
    maxMps?: number;
    oneWheelLiftEmergencyMillis?: number;
    panelType?: string;
    playButtonClearEmergencyMillis?: number;
    repository?: string;
    stopButtonEmergencyMillis?: number;
    tickPerM?: number;
    tiltEmergencyMillis?: number;
    version?: string;
    wheelBase?: number;
}
export type QueryParamsType = Record<string | number, any>;
export type ResponseFormat = keyof Omit<Body, "body" | "bodyUsed">;
export interface FullRequestParams extends Omit<RequestInit, "body"> {
    /** set parameter to `true` for call `securityWorker` for this request */
    secure?: boolean;
    /** request path */
    path: string;
    /** content type of request body */
    type?: ContentType;
    /** query params */
    query?: QueryParamsType;
    /** format of response (i.e. response.json() -> format: "json") */
    format?: ResponseFormat;
    /** request body */
    body?: unknown;
    /** base url */
    baseUrl?: string;
    /** request cancellation token */
    cancelToken?: CancelToken;
}
export type RequestParams = Omit<FullRequestParams, "body" | "method" | "query" | "path">;
export interface ApiConfig<SecurityDataType = unknown> {
    baseUrl?: string;
    baseApiParams?: Omit<RequestParams, "baseUrl" | "cancelToken" | "signal">;
    securityWorker?: (securityData: SecurityDataType | null) => Promise<RequestParams | void> | RequestParams | void;
    customFetch?: typeof fetch;
}
export interface HttpResponse<D extends unknown, E extends unknown = unknown> extends Response {
    data: D;
    error: E;
}
type CancelToken = Symbol | string | number;
export declare enum ContentType {
    Json = "application/json",
    FormData = "multipart/form-data",
    UrlEncoded = "application/x-www-form-urlencoded",
    Text = "text/plain"
}
export declare class HttpClient<SecurityDataType = unknown> {
    baseUrl: string;
    private securityData;
    private securityWorker?;
    private abortControllers;
    private customFetch;
    private baseApiParams;
    constructor(apiConfig?: ApiConfig<SecurityDataType>);
    setSecurityData: (data: SecurityDataType | null) => void;
    protected encodeQueryParam(key: string, value: any): string;
    protected addQueryParam(query: QueryParamsType, key: string): string;
    protected addArrayQueryParam(query: QueryParamsType, key: string): any;
    protected toQueryString(rawQuery?: QueryParamsType): string;
    protected addQueryParams(rawQuery?: QueryParamsType): string;
    private contentFormatters;
    protected mergeRequestParams(params1: RequestParams, params2?: RequestParams): RequestParams;
    protected createAbortSignal: (cancelToken: CancelToken) => AbortSignal | undefined;
    abortRequest: (cancelToken: CancelToken) => void;
    request: <T = any, E = any>({ body, secure, path, type, query, format, baseUrl, cancelToken, ...params }: FullRequestParams) => Promise<HttpResponse<T, E>>;
}
/**
 * @title No title
 * @contact
 */
export declare class Api<SecurityDataType extends unknown> extends HttpClient<SecurityDataType> {
    config: {
        /**
         * @description get config env from backend
         *
         * @tags config
         * @name EnvsList
         * @summary get config env from backend
         * @request GET:/config/envs
         */
        envsList: (params?: RequestParams) => Promise<HttpResponse<ApiGetConfigResponse, ApiErrorResponse>>;
        /**
         * @description get config from backend
         *
         * @tags config
         * @name KeysGetCreate
         * @summary get config from backend
         * @request POST:/config/keys/get
         */
        keysGetCreate: (settings: Record<string, string>, params?: RequestParams) => Promise<HttpResponse<Record<string, string>, ApiErrorResponse>>;
        /**
         * @description set config to backend
         *
         * @tags config
         * @name KeysSetCreate
         * @summary set config to backend
         * @request POST:/config/keys/set
         */
        keysSetCreate: (settings: Record<string, string>, params?: RequestParams) => Promise<HttpResponse<Record<string, string>, ApiErrorResponse>>;
    };
    containers: {
        /**
         * @description list all containers
         *
         * @tags containers
         * @name ContainersList
         * @summary list all containers
         * @request GET:/containers
         */
        containersList: (params?: RequestParams) => Promise<HttpResponse<ApiContainerListResponse, ApiErrorResponse>>;
        /**
         * @description get container logs
         *
         * @tags containers
         * @name LogsDetail
         * @summary get container logs
         * @request GET:/containers/{containerId}/logs
         */
        logsDetail: (containerId: string, params?: RequestParams) => Promise<HttpResponse<any, any>>;
        /**
         * @description execute a command on a container
         *
         * @tags containers
         * @name ContainersCreate
         * @summary execute a command on a container
         * @request POST:/containers/{containerId}/{command}
         */
        containersCreate: (containerId: string, command: string, params?: RequestParams) => Promise<HttpResponse<ApiOkResponse, ApiErrorResponse>>;
    };
    openmower: {
        /**
         * @description call a service
         *
         * @tags openmower
         * @name CallCreate
         * @summary call a service
         * @request POST:/openmower/call/{command}
         */
        callCreate: (command: string, CallReq: Record<string, any>, params?: RequestParams) => Promise<HttpResponse<ApiOkResponse, ApiErrorResponse>>;
        /**
         * @description clear the map
         *
         * @tags openmower
         * @name DeleteOpenmower
         * @summary clear the map
         * @request DELETE:/openmower/map
         */
        deleteOpenmower: (params?: RequestParams) => Promise<HttpResponse<ApiOkResponse, ApiErrorResponse>>;
        /**
         * @description add a map area
         *
         * @tags openmower
         * @name MapAreaAddCreate
         * @summary add a map area
         * @request POST:/openmower/map/area/add
         */
        mapAreaAddCreate: (CallReq: MowerMapAddMowingAreaSrvReq, params?: RequestParams) => Promise<HttpResponse<ApiOkResponse, ApiErrorResponse>>;
        /**
         * @description replace entire map in a single transaction
         *
         * @tags openmower
         * @name mapReplace
         * @summary Delete the current map and rplace all areas
         * @request PUT:/openmower/map
         */
        mapReplace: (CallReq: MowerReplaceMapSrvReq, params?: RequestParams) => Promise<HttpResponse<ApiOkResponse, ApiErrorResponse>>;
        /**
         * @description set the docking point
         *
         * @tags openmower
         * @name MapDockingCreate
         * @summary set the docking point
         * @request POST:/openmower/map/docking
         */
        mapDockingCreate: (CallReq: MowerMapSetDockingPointSrvReq, params?: RequestParams) => Promise<HttpResponse<ApiOkResponse, ApiErrorResponse>>;
        /**
         * @description publish to a topic
         *
         * @tags openmower
         * @name PublishDetail
         * @summary publish to a topic
         * @request GET:/openmower/publish/{topic}
         */
        publishDetail: (topic: string, params?: RequestParams) => Promise<HttpResponse<any, any>>;
        /**
         * @description subscribe to a topic
         *
         * @tags openmower
         * @name SubscribeDetail
         * @summary subscribe to a topic
         * @request GET:/openmower/subscribe/{topic}
         */
        subscribeDetail: (topic: string, params?: RequestParams) => Promise<HttpResponse<any, any>>;
    };
    settings: {
        /**
         * @description returns a JSON object with the settings
         *
         * @tags settings
         * @name SettingsList
         * @summary returns a JSON object with the settings
         * @request GET:/settings
         */
        settingsList: (params?: RequestParams) => Promise<HttpResponse<ApiGetSettingsResponse, ApiErrorResponse>>;
        /**
         * @description saves the settings to the mower_config.sh file
         *
         * @tags settings
         * @name SettingsCreate
         * @summary saves the settings to the mower_config.sh file
         * @request POST:/settings
         */
        settingsCreate: (settings: Record<string, any>, params?: RequestParams) => Promise<HttpResponse<ApiOkResponse, ApiErrorResponse>>;
        /**
         * @description returns the JSON Schema for mower configuration parameters
         *
         * @tags settings
         * @name SettingsSchemaList
         * @summary returns the mower config JSON Schema
         * @request GET:/settings/schema
         */
        settingsSchemaList: (params?: RequestParams) => Promise<HttpResponse<Record<string, any>, ApiErrorResponse>>;
        /**
         * @description returns the current YAML mower configuration values
         *
         * @tags settings
         * @name SettingsYamlList
         * @summary returns the current YAML mower configuration
         * @request GET:/settings/yaml
         */
        settingsYamlList: (params?: RequestParams) => Promise<HttpResponse<Record<string, any>, ApiErrorResponse>>;
        /**
         * @description saves the mower configuration as YAML
         *
         * @tags settings
         * @name SettingsYamlCreate
         * @summary saves the mower configuration as YAML
         * @request POST:/settings/yaml
         */
        settingsYamlCreate: (settings: Record<string, any>, params?: RequestParams) => Promise<HttpResponse<ApiOkResponse, ApiErrorResponse>>;
    };
    setup: {
        /**
         * @description flash the mower board with the given config
         *
         * @tags setup
         * @name FlashBoardCreate
         * @summary flash the mower board with the given config
         * @request POST:/setup/flashBoard
         */
        flashBoardCreate: (settings: TypesFirmwareConfig, params?: RequestParams) => Promise<HttpResponse<ApiOkResponse, ApiErrorResponse>>;
    };
}
export {};
//# sourceMappingURL=Api.d.ts.map