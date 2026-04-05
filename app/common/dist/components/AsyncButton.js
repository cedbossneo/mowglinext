import { jsx as _jsx } from "react/jsx-runtime";
import { App, Button } from "antd";
import * as React from "react";
export const AsyncButton = (props) => {
    const { notification } = App.useApp();
    const { onAsyncClick, ...rest } = props;
    const [loading, setLoading] = React.useState(false);
    const handleClick = (event) => {
        if (onAsyncClick !== undefined) {
            setLoading(true);
            onAsyncClick(event).then(() => {
                setLoading(false);
            }).catch((e) => {
                setLoading(false);
                if (console.error)
                    console.error(e);
                notification.error({
                    message: 'An error occured',
                    description: e.message,
                });
            });
        }
    };
    return _jsx(Button, { loading: loading, onClick: handleClick, ...rest, children: props.children });
};
export default AsyncButton;
