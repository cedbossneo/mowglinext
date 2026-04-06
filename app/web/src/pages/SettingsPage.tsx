import { Col, Row, Tabs } from "antd";
import { SettingsComponent } from "../components/SettingsComponent";
import { SchemaSettingsComponent } from "../components/SchemaSettingsComponent";
import { App, Button } from "antd";
import {AsyncButton, useApi} from "@mowglinext/common";

export const SettingsPage = () => {
    const guiApi = useApi();
    const { notification } = App.useApp();

    const restartOpenMower = async () => {
        try {
            const resContainersList = await guiApi.containers.containersList();
            if (resContainersList.error) throw new Error(resContainersList.error.error);
            const container = resContainersList.data.containers?.find(
                (c) => c.labels?.app === "openmower" || c.names?.includes("/openmower")
            );
            if (container?.id) {
                const res = await guiApi.containers.containersCreate(container.id, "restart");
                if (res.error) throw new Error(res.error.error);
                notification.success({ message: "OpenMower restarted" });
            } else {
                throw new Error("OpenMower container not found");
            }
        } catch (e: any) {
            notification.error({ message: "Failed to restart OpenMower", description: e.message });
        }
    };

    const restartGui = async () => {
        try {
            const resContainersList = await guiApi.containers.containersList();
            if (resContainersList.error) throw new Error(resContainersList.error.error);
            const container = resContainersList.data.containers?.find(
                (c) => c.labels?.app === "gui" || c.names?.includes("/mowglinext")
            );
            if (container?.id) {
                const res = await guiApi.containers.containersCreate(container.id, "restart");
                if (res.error) throw new Error(res.error.error);
                notification.success({ message: "GUI restarted" });
            } else {
                throw new Error("GUI container not found");
            }
        } catch (e: any) {
            notification.error({ message: "Failed to restart GUI", description: e.message });
        }
    };

    const items = [
        {
            key: "mower",
            label: "Mower Configuration",
            children: (
                <SchemaSettingsComponent
                    onRestartOM={restartOpenMower}
                    onRestartGUI={restartGui}
                />
            ),
        },
        {
            key: "system",
            label: "System Settings",
            children: (
                <SettingsComponent
                    actions={(form, save, restartOM, restartGUI) => [
                        <Button key="save" type="primary" loading={form.loading} onClick={() => {
                            form.submit(save).catch(() => {});
                        }}>
                            Save settings
                        </Button>,
                        <AsyncButton key="restart-om" onAsyncClick={restartOM}>
                            Restart OpenMower
                        </AsyncButton>,
                        <AsyncButton key="restart-gui" onAsyncClick={restartGUI}>
                            Restart GUI
                        </AsyncButton>,
                    ]}
                />
            ),
        },
    ];

    return (
        <Row>
            <Col span={24}>
                <Tabs items={items} defaultActiveKey="mower" />
            </Col>
        </Row>
    );
};

export default SettingsPage;
