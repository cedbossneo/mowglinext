import { Api } from "../api/Api";
import { useConnection } from "../connection/ConnectionContext";
export const useApi = () => {
    const { baseHttpUrl, authToken } = useConnection();
    const api = new Api();
    api.baseUrl = `${baseHttpUrl}/api`;
    if (authToken) {
        api.setSecurityData(authToken);
    }
    return api;
};
