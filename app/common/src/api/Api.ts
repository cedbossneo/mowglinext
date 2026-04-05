/* eslint-disable */
/* tslint:disable */
/*
 * ---------------------------------------------------------------
 * ## THIS FILE WAS GENERATED VIA SWAGGER-TYPESCRIPT-API        ##
 * ##                                                           ##
 * ## AUTHOR: acacode                                           ##
 * ## SOURCE: https://github.com/acacode/swagger-typescript-api ##
 * ---------------------------------------------------------------
 */

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

export enum ContentType {
  Json = "application/json",
  FormData = "multipart/form-data",
  UrlEncoded = "application/x-www-form-urlencoded",
  Text = "text/plain",
}

export class HttpClient<SecurityDataType = unknown> {
  public baseUrl: string = "";
  private securityData: SecurityDataType | null = null;
  private securityWorker?: ApiConfig<SecurityDataType>["securityWorker"];
  private abortControllers = new Map<CancelToken, AbortController>();
  private customFetch = (...fetchParams: Parameters<typeof fetch>) => fetch(...fetchParams);

  private baseApiParams: RequestParams = {
    credentials: "same-origin",
    headers: {},
    redirect: "follow",
    referrerPolicy: "no-referrer",
  };

  constructor(apiConfig: ApiConfig<SecurityDataType> = {}) {
    Object.assign(this, apiConfig);
  }

  public setSecurityData = (data: SecurityDataType | null) => {
    this.securityData = data;
  };

  protected encodeQueryParam(key: string, value: any) {
    const encodedKey = encodeURIComponent(key);
    return `${encodedKey}=${encodeURIComponent(typeof value === "number" ? value : `${value}`)}`;
  }

  protected addQueryParam(query: QueryParamsType, key: string) {
    return this.encodeQueryParam(key, query[key]);
  }

  protected addArrayQueryParam(query: QueryParamsType, key: string) {
    const value = query[key];
    return value.map((v: any) => this.encodeQueryParam(key, v)).join("&");
  }

  protected toQueryString(rawQuery?: QueryParamsType): string {
      const query = rawQuery || {};
      const keys = Object.keys(query).filter((key) => "undefined" !== typeof query[key]);
      return keys
          .map((key) => (Array.isArray(query[key]) ? this.addArrayQueryParam(query, key) : this.addQueryParam(query, key)))
          .join("&");
  }

  protected addQueryParams(rawQuery?: QueryParamsType): string {
    const queryString = this.toQueryString(rawQuery);
    return queryString ? `?${queryString}` : "";
  }

  private contentFormatters: Record<ContentType, (input: any) => any> = {
      [ContentType.Json]: (input: any) =>
          input !== null && (typeof input === "object" || typeof input === "string") ? JSON.stringify(input) : input,
      [ContentType.Text]: (input: any) => (input !== null && typeof input !== "string" ? JSON.stringify(input) : input),
      [ContentType.FormData]: (input: any) =>
          Object.keys(input || {}).reduce((formData, key) => {
              const property = input[key];
              formData.append(
                  key,
                  property instanceof Blob
                      ? property
                      : typeof property === "object" && property !== null
                          ? JSON.stringify(property)
                          : `${property}`,
              );
              return formData;
          }, new FormData()),
      [ContentType.UrlEncoded]: (input: any) => this.toQueryString(input),
  };

  protected mergeRequestParams(params1: RequestParams, params2?: RequestParams): RequestParams {
    return {
      ...this.baseApiParams,
      ...params1,
      ...(params2 || {}),
      headers: {
        ...(this.baseApiParams.headers || {}),
        ...(params1.headers || {}),
        ...((params2 && params2.headers) || {}),
      },
    };
  }

  protected createAbortSignal = (cancelToken: CancelToken): AbortSignal | undefined => {
    if (this.abortControllers.has(cancelToken)) {
      const abortController = this.abortControllers.get(cancelToken);
      if (abortController) {
        return abortController.signal;
      }
      return void 0;
    }

    const abortController = new AbortController();
    this.abortControllers.set(cancelToken, abortController);
    return abortController.signal;
  };

  public abortRequest = (cancelToken: CancelToken) => {
    const abortController = this.abortControllers.get(cancelToken);

    if (abortController) {
      abortController.abort();
      this.abortControllers.delete(cancelToken);
    }
  };

  public request = async <T = any, E = any>({
                                                body,
                                                secure,
                                                path,
                                                type,
                                                query,
                                                format,
                                                baseUrl,
                                                cancelToken,
                                                ...params
                                            }: FullRequestParams): Promise<HttpResponse<T, E>> => {
      const secureParams =
          ((typeof secure === "boolean" ? secure : this.baseApiParams.secure) &&
              this.securityWorker &&
              (await this.securityWorker(this.securityData))) ||
          {};
      const requestParams = this.mergeRequestParams(params, secureParams);
      const queryString = query && this.toQueryString(query);
      const payloadFormatter = this.contentFormatters[type || ContentType.Json];
      const responseFormat = format || requestParams.format;

      return this.customFetch(`${baseUrl || this.baseUrl || ""}${path}${queryString ? `?${queryString}` : ""}`, {
          ...requestParams,
          headers: {
              ...(requestParams.headers || {}),
              ...(type && type !== ContentType.FormData ? {"Content-Type": type} : {}),
          },
          signal: (cancelToken ? this.createAbortSignal(cancelToken) : requestParams.signal) || null,
          body: typeof body === "undefined" || body === null ? null : payloadFormatter(body),
      }).then(async (response) => {
          const r = response as HttpResponse<T, E>;
          r.data = null as unknown as T;
          r.error = null as unknown as E;

          const data = !responseFormat
              ? r
              : await response[responseFormat]()
                  .then((data) => {
                      if (r.ok) {
                          r.data = data;
                      } else {
                          r.error = data;
                      }
                      return r;
                  })
                  .catch((e) => {
                      r.error = e;
                      return r;
                  });

          if (cancelToken) {
              this.abortControllers.delete(cancelToken);
          }

          if (!response.ok) throw data;
          return data;
      });
  };
}

/**
 * @title No title
 * @contact
 */
export class Api<SecurityDataType extends unknown> extends HttpClient<SecurityDataType> {
    config = {
        /**
         * @description get config env from backend
         *
         * @tags config
         * @name EnvsList
         * @summary get config env from backend
         * @request GET:/config/envs
         */
        envsList: (params: RequestParams = {}) =>
            this.request<ApiGetConfigResponse, ApiErrorResponse>({
                path: `/config/envs`,
                method: "GET",
                format: "json",
                ...params,
            }),

        /**
         * @description get config from backend
         *
         * @tags config
         * @name KeysGetCreate
         * @summary get config from backend
         * @request POST:/config/keys/get
         */
        keysGetCreate: (settings: Record<string, string>, params: RequestParams = {}) =>
            this.request<Record<string, string>, ApiErrorResponse>({
                path: `/config/keys/get`,
                method: "POST",
                body: settings,
                format: "json",
                ...params,
            }),

        /**
         * @description set config to backend
         *
         * @tags config
         * @name KeysSetCreate
         * @summary set config to backend
         * @request POST:/config/keys/set
         */
        keysSetCreate: (settings: Record<string, string>, params: RequestParams = {}) =>
            this.request<Record<string, string>, ApiErrorResponse>({
                path: `/config/keys/set`,
                method: "POST",
                body: settings,
                format: "json",
                ...params,
            }),
    };
  containers = {
      /**
       * @description list all containers
       *
       * @tags containers
       * @name ContainersList
       * @summary list all containers
       * @request GET:/containers
       */
      containersList: (params: RequestParams = {}) =>
          this.request<ApiContainerListResponse, ApiErrorResponse>({
              path: `/containers`,
              method: "GET",
              format: "json",
              ...params,
          }),

      /**
       * @description get container logs
       *
       * @tags containers
       * @name LogsDetail
       * @summary get container logs
       * @request GET:/containers/{containerId}/logs
       */
      logsDetail: (containerId: string, params: RequestParams = {}) =>
          this.request<any, any>({
              path: `/containers/${containerId}/logs`,
              method: "GET",
              ...params,
          }),

      /**
       * @description execute a command on a container
       *
       * @tags containers
       * @name ContainersCreate
       * @summary execute a command on a container
       * @request POST:/containers/{containerId}/{command}
       */
      containersCreate: (containerId: string, command: string, params: RequestParams = {}) =>
          this.request<ApiOkResponse, ApiErrorResponse>({
              path: `/containers/${containerId}/${command}`,
              method: "POST",
              format: "json",
              ...params,
          }),
  };
  openmower = {
      /**
       * @description call a service
       *
       * @tags openmower
       * @name CallCreate
       * @summary call a service
       * @request POST:/openmower/call/{command}
       */
      callCreate: (command: string, CallReq: Record<string, any>, params: RequestParams = {}) =>
          this.request<ApiOkResponse, ApiErrorResponse>({
              path: `/openmower/call/${command}`,
              method: "POST",
              body: CallReq,
              type: ContentType.Json,
              format: "json",
              ...params,
          }),

      /**
       * @description clear the map
       *
       * @tags openmower
       * @name DeleteOpenmower
       * @summary clear the map
       * @request DELETE:/openmower/map
       */
      deleteOpenmower: (params: RequestParams = {}) =>
          this.request<ApiOkResponse, ApiErrorResponse>({
              path: `/openmower/map`,
              method: "DELETE",
              type: ContentType.Json,
              format: "json",
              ...params,
          }),

      /**
       * @description add a map area
       *
       * @tags openmower
       * @name MapAreaAddCreate
       * @summary add a map area
       * @request POST:/openmower/map/area/add
       */
      mapAreaAddCreate: (CallReq: MowerMapAddMowingAreaSrvReq, params: RequestParams = {}) =>
          this.request<ApiOkResponse, ApiErrorResponse>({
              path: `/openmower/map/area/add`,
              method: "POST",
              body: CallReq,
              type: ContentType.Json,
              format: "json",
              ...params,
          }),

      /**
       * @description replace entire map in a single transaction
       *
       * @tags openmower
       * @name mapReplace
       * @summary Delete the current map and rplace all areas
       * @request PUT:/openmower/map
       */
      mapReplace: (CallReq: MowerReplaceMapSrvReq, params: RequestParams = {}) =>
        this.request<ApiOkResponse, ApiErrorResponse>({
            path: `/openmower/map`,
            method: "PUT",
            body: CallReq,
            type: ContentType.Json,
            format: "json",
            ...params,
        }),


      /**
       * @description set the docking point
       *
       * @tags openmower
       * @name MapDockingCreate
       * @summary set the docking point
       * @request POST:/openmower/map/docking
       */
      mapDockingCreate: (CallReq: MowerMapSetDockingPointSrvReq, params: RequestParams = {}) =>
          this.request<ApiOkResponse, ApiErrorResponse>({
              path: `/openmower/map/docking`,
              method: "POST",
              body: CallReq,
              type: ContentType.Json,
              format: "json",
              ...params,
          }),

      /**
       * @description publish to a topic
       *
       * @tags openmower
       * @name PublishDetail
       * @summary publish to a topic
       * @request GET:/openmower/publish/{topic}
       */
      publishDetail: (topic: string, params: RequestParams = {}) =>
          this.request<any, any>({
              path: `/openmower/publish/${topic}`,
              method: "GET",
              ...params,
          }),

      /**
       * @description subscribe to a topic
       *
       * @tags openmower
       * @name SubscribeDetail
       * @summary subscribe to a topic
       * @request GET:/openmower/subscribe/{topic}
       */
      subscribeDetail: (topic: string, params: RequestParams = {}) =>
          this.request<any, any>({
              path: `/openmower/subscribe/${topic}`,
              method: "GET",
              ...params,
          }),
  };
  settings = {
      /**
       * @description returns a JSON object with the settings
       *
       * @tags settings
       * @name SettingsList
       * @summary returns a JSON object with the settings
       * @request GET:/settings
       */
      settingsList: (params: RequestParams = {}) =>
          this.request<ApiGetSettingsResponse, ApiErrorResponse>({
              path: `/settings`,
              method: "GET",
              format: "json",
              ...params,
          }),

      /**
       * @description saves the settings to the mower_config.sh file
       *
       * @tags settings
       * @name SettingsCreate
       * @summary saves the settings to the mower_config.sh file
       * @request POST:/settings
       */
      settingsCreate: (settings: Record<string, any>, params: RequestParams = {}) =>
          this.request<ApiOkResponse, ApiErrorResponse>({
              path: `/settings`,
              method: "POST",
              body: settings,
              type: ContentType.Json,
              format: "json",
              ...params,
          }),

      /**
       * @description returns the JSON Schema for mower configuration parameters
       *
       * @tags settings
       * @name SettingsSchemaList
       * @summary returns the mower config JSON Schema
       * @request GET:/settings/schema
       */
      settingsSchemaList: (params: RequestParams = {}) =>
          this.request<Record<string, any>, ApiErrorResponse>({
              path: `/settings/schema`,
              method: "GET",
              format: "json",
              ...params,
          }),

      /**
       * @description returns the current YAML mower configuration values
       *
       * @tags settings
       * @name SettingsYamlList
       * @summary returns the current YAML mower configuration
       * @request GET:/settings/yaml
       */
      settingsYamlList: (params: RequestParams = {}) =>
          this.request<Record<string, any>, ApiErrorResponse>({
              path: `/settings/yaml`,
              method: "GET",
              format: "json",
              ...params,
          }),

      /**
       * @description saves the mower configuration as YAML
       *
       * @tags settings
       * @name SettingsYamlCreate
       * @summary saves the mower configuration as YAML
       * @request POST:/settings/yaml
       */
      settingsYamlCreate: (settings: Record<string, any>, params: RequestParams = {}) =>
          this.request<ApiOkResponse, ApiErrorResponse>({
              path: `/settings/yaml`,
              method: "POST",
              body: settings,
              type: ContentType.Json,
              format: "json",
              ...params,
          }),
  };
    setup = {
        /**
         * @description flash the mower board with the given config
         *
         * @tags setup
         * @name FlashBoardCreate
         * @summary flash the mower board with the given config
         * @request POST:/setup/flashBoard
         */
        flashBoardCreate: (settings: TypesFirmwareConfig, params: RequestParams = {}) =>
            this.request<ApiOkResponse, ApiErrorResponse>({
                path: `/setup/flashBoard`,
                method: "POST",
                body: settings,
                type: ContentType.Json,
                ...params,
            }),
  };
}
