import {Dropdown} from "antd";
import * as React from "react";
import {DropdownButtonProps} from "antd/es/dropdown";

export const AsyncDropDownButton: React.FC<DropdownButtonProps & {
    menu: DropdownButtonProps["menu"] & {
        onAsyncClick: (event: any) => Promise<any>
    }
}> = (props) => {
    const [loading, setLoading] = React.useState(false)
    const handleClick = (event: any) => {
        if (props.menu.onAsyncClick !== undefined) {
            setLoading(true)
            props.menu.onAsyncClick(event).then(() => {
                setLoading(false)
            }).catch(() => {
                setLoading(false)
            })
        }
    }
    const {menu, ...rest} = props
    return <Dropdown.Button loading={loading} {...rest} menu={{
        items: menu.items,
        onClick: handleClick,
    }}>{props.children}</Dropdown.Button>
}

export default AsyncDropDownButton;
