import {App, Button, ButtonProps} from "antd";
import * as React from "react";


export const AsyncButton: React.FC<ButtonProps & {
    onAsyncClick: (event: React.MouseEvent) => Promise<any>
}> = (props) => {

    const {notification} = App.useApp();
    const {onAsyncClick, ...rest} = props;
    const [loading, setLoading] = React.useState(false)
    const handleClick = (event: React.MouseEvent) => {
        if (onAsyncClick !== undefined) {
            setLoading(true)
            onAsyncClick(event).then(() => {
                setLoading(false)
            }).catch((e) => {
                setLoading(false)
                if (console.error)
                    console.error(e);
                notification.error({
                    message: 'An error occured',
                    description: e.message,
                })
            })
        }
    }
    return <Button loading={loading} onClick={handleClick} {...rest}>{props.children}</Button>
}

export default AsyncButton;
