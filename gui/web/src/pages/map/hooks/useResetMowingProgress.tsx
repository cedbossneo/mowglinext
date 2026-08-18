import { useCallback } from "react";
import { App } from "antd";
import { useTranslation } from "react-i18next";
import { useApi } from "../../../hooks/useApi.ts";

export const useResetMowingProgress = () => {
  const guiApi = useApi();
  const { modal, notification } = App.useApp();
  const { t } = useTranslation();

  return useCallback(() => {
    modal.confirm({
      title: t("resetMowingProgress.confirmTitle"),
      content: (
        <div>
          <p>{t("resetMowingProgress.confirmBody")}</p>
          <p style={{ marginBottom: 0 }}>
            {t("resetMowingProgress.confirmHint")}
          </p>
        </div>
      ),
      okText: t("resetMowingProgress.confirmAction"),
      okType: "danger",
      cancelText: t("resetMowingProgress.cancel"),
      onOk: async () => {
        try {
          const response = await guiApi.mowglinext.callCreate(
            "coverage_clear_resume",
            {},
          );
          if (response.error) {
            throw new Error(response.error.error);
          }
          notification.success({
            message: t("resetMowingProgress.success"),
          });
        } catch (error: unknown) {
          const description =
            error instanceof Error
              ? error.message
              : t("resetMowingProgress.unknownError");
          notification.error({
            message: t("resetMowingProgress.error"),
            description,
          });
          // Rejecting keeps the Ant Design confirmation open so the
          // operator can retry or cancel after reading the error.
          throw error;
        }
      },
    });
  }, [guiApi, modal, notification, t]);
};
