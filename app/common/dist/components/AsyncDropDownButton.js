import { jsx as _jsx } from "react/jsx-runtime";
import { Dropdown } from "antd";
import * as React from "react";
export const AsyncDropDownButton = (props) => {
    const [loading, setLoading] = React.useState(false);
    const handleClick = (event) => {
        if (props.menu.onAsyncClick !== undefined) {
            setLoading(true);
            props.menu.onAsyncClick(event).then(() => {
                setLoading(false);
            }).catch(() => {
                setLoading(false);
            });
        }
    };
    const { menu, ...rest } = props;
    return _jsx(Dropdown.Button, { loading: loading, ...rest, menu: {
            items: menu.items,
            onClick: handleClick,
        }, children: props.children });
};
export default AsyncDropDownButton;
