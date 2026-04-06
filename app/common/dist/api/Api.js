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
export var ContentType;
(function (ContentType) {
    ContentType["Json"] = "application/json";
    ContentType["FormData"] = "multipart/form-data";
    ContentType["UrlEncoded"] = "application/x-www-form-urlencoded";
    ContentType["Text"] = "text/plain";
})(ContentType || (ContentType = {}));
export class HttpClient {
    constructor(apiConfig = {}) {
        Object.defineProperty(this, "baseUrl", {
            enumerable: true,
            configurable: true,
            writable: true,
            value: ""
        });
        Object.defineProperty(this, "securityData", {
            enumerable: true,
            configurable: true,
            writable: true,
            value: null
        });
        Object.defineProperty(this, "securityWorker", {
            enumerable: true,
            configurable: true,
            writable: true,
            value: void 0
        });
        Object.defineProperty(this, "abortControllers", {
            enumerable: true,
            configurable: true,
            writable: true,
            value: new Map()
        });
        Object.defineProperty(this, "customFetch", {
            enumerable: true,
            configurable: true,
            writable: true,
            value: (...fetchParams) => fetch(...fetchParams)
        });
        Object.defineProperty(this, "baseApiParams", {
            enumerable: true,
            configurable: true,
            writable: true,
            value: {
                credentials: "same-origin",
                headers: {},
                redirect: "follow",
                referrerPolicy: "no-referrer",
            }
        });
        Object.defineProperty(this, "setSecurityData", {
            enumerable: true,
            configurable: true,
            writable: true,
            value: (data) => {
                this.securityData = data;
            }
        });
        Object.defineProperty(this, "contentFormatters", {
            enumerable: true,
            configurable: true,
            writable: true,
            value: {
                [ContentType.Json]: (input) => input !== null && (typeof input === "object" || typeof input === "string") ? JSON.stringify(input) : input,
                [ContentType.Text]: (input) => (input !== null && typeof input !== "string" ? JSON.stringify(input) : input),
                [ContentType.FormData]: (input) => Object.keys(input || {}).reduce((formData, key) => {
                    const property = input[key];
                    formData.append(key, property instanceof Blob
                        ? property
                        : typeof property === "object" && property !== null
                            ? JSON.stringify(property)
                            : `${property}`);
                    return formData;
                }, new FormData()),
                [ContentType.UrlEncoded]: (input) => this.toQueryString(input),
            }
        });
        Object.defineProperty(this, "createAbortSignal", {
            enumerable: true,
            configurable: true,
            writable: true,
            value: (cancelToken) => {
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
            }
        });
        Object.defineProperty(this, "abortRequest", {
            enumerable: true,
            configurable: true,
            writable: true,
            value: (cancelToken) => {
                const abortController = this.abortControllers.get(cancelToken);
                if (abortController) {
                    abortController.abort();
                    this.abortControllers.delete(cancelToken);
                }
            }
        });
        Object.defineProperty(this, "request", {
            enumerable: true,
            configurable: true,
            writable: true,
            value: async ({ body, secure, path, type, query, format, baseUrl, cancelToken, ...params }) => {
                const secureParams = ((typeof secure === "boolean" ? secure : this.baseApiParams.secure) &&
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
                        ...(type && type !== ContentType.FormData ? { "Content-Type": type } : {}),
                    },
                    signal: (cancelToken ? this.createAbortSignal(cancelToken) : requestParams.signal) || null,
                    body: typeof body === "undefined" || body === null ? null : payloadFormatter(body),
                }).then(async (response) => {
                    const r = response;
                    r.data = null;
                    r.error = null;
                    const data = !responseFormat
                        ? r
                        : await response[responseFormat]()
                            .then((data) => {
                            if (r.ok) {
                                r.data = data;
                            }
                            else {
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
                    if (!response.ok)
                        throw data;
                    return data;
                });
            }
        });
        Object.assign(this, apiConfig);
    }
    encodeQueryParam(key, value) {
        const encodedKey = encodeURIComponent(key);
        return `${encodedKey}=${encodeURIComponent(typeof value === "number" ? value : `${value}`)}`;
    }
    addQueryParam(query, key) {
        return this.encodeQueryParam(key, query[key]);
    }
    addArrayQueryParam(query, key) {
        const value = query[key];
        return value.map((v) => this.encodeQueryParam(key, v)).join("&");
    }
    toQueryString(rawQuery) {
        const query = rawQuery || {};
        const keys = Object.keys(query).filter((key) => "undefined" !== typeof query[key]);
        return keys
            .map((key) => (Array.isArray(query[key]) ? this.addArrayQueryParam(query, key) : this.addQueryParam(query, key)))
            .join("&");
    }
    addQueryParams(rawQuery) {
        const queryString = this.toQueryString(rawQuery);
        return queryString ? `?${queryString}` : "";
    }
    mergeRequestParams(params1, params2) {
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
}
/**
 * @title No title
 * @contact
 */
export class Api extends HttpClient {
    constructor() {
        super(...arguments);
        Object.defineProperty(this, "config", {
            enumerable: true,
            configurable: true,
            writable: true,
            value: {
                /**
                 * @description get config env from backend
                 *
                 * @tags config
                 * @name EnvsList
                 * @summary get config env from backend
                 * @request GET:/config/envs
                 */
                envsList: (params = {}) => this.request({
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
                keysGetCreate: (settings, params = {}) => this.request({
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
                keysSetCreate: (settings, params = {}) => this.request({
                    path: `/config/keys/set`,
                    method: "POST",
                    body: settings,
                    format: "json",
                    ...params,
                }),
            }
        });
        Object.defineProperty(this, "containers", {
            enumerable: true,
            configurable: true,
            writable: true,
            value: {
                /**
                 * @description list all containers
                 *
                 * @tags containers
                 * @name ContainersList
                 * @summary list all containers
                 * @request GET:/containers
                 */
                containersList: (params = {}) => this.request({
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
                logsDetail: (containerId, params = {}) => this.request({
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
                containersCreate: (containerId, command, params = {}) => this.request({
                    path: `/containers/${containerId}/${command}`,
                    method: "POST",
                    format: "json",
                    ...params,
                }),
            }
        });
        Object.defineProperty(this, "openmower", {
            enumerable: true,
            configurable: true,
            writable: true,
            value: {
                /**
                 * @description call a service
                 *
                 * @tags openmower
                 * @name CallCreate
                 * @summary call a service
                 * @request POST:/openmower/call/{command}
                 */
                callCreate: (command, CallReq, params = {}) => this.request({
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
                deleteOpenmower: (params = {}) => this.request({
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
                mapAreaAddCreate: (CallReq, params = {}) => this.request({
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
                mapReplace: (CallReq, params = {}) => this.request({
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
                mapDockingCreate: (CallReq, params = {}) => this.request({
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
                publishDetail: (topic, params = {}) => this.request({
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
                subscribeDetail: (topic, params = {}) => this.request({
                    path: `/openmower/subscribe/${topic}`,
                    method: "GET",
                    ...params,
                }),
            }
        });
        Object.defineProperty(this, "settings", {
            enumerable: true,
            configurable: true,
            writable: true,
            value: {
                /**
                 * @description returns a JSON object with the settings
                 *
                 * @tags settings
                 * @name SettingsList
                 * @summary returns a JSON object with the settings
                 * @request GET:/settings
                 */
                settingsList: (params = {}) => this.request({
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
                settingsCreate: (settings, params = {}) => this.request({
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
                settingsSchemaList: (params = {}) => this.request({
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
                settingsYamlList: (params = {}) => this.request({
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
                settingsYamlCreate: (settings, params = {}) => this.request({
                    path: `/settings/yaml`,
                    method: "POST",
                    body: settings,
                    type: ContentType.Json,
                    format: "json",
                    ...params,
                }),
            }
        });
        Object.defineProperty(this, "setup", {
            enumerable: true,
            configurable: true,
            writable: true,
            value: {
                /**
                 * @description flash the mower board with the given config
                 *
                 * @tags setup
                 * @name FlashBoardCreate
                 * @summary flash the mower board with the given config
                 * @request POST:/setup/flashBoard
                 */
                flashBoardCreate: (settings, params = {}) => this.request({
                    path: `/setup/flashBoard`,
                    method: "POST",
                    body: settings,
                    type: ContentType.Json,
                    ...params,
                }),
            }
        });
    }
}
