import {Api} from "../api/Api.ts";

// One shared Api instance for the whole app — previously `useApi` returned
// `new Api()` on every render, which allocated fetch config and broke any
// consumer that tried to memoize against the api reference.
const api = new Api();
api.baseUrl = "/api";

export const useApi = () => api;
