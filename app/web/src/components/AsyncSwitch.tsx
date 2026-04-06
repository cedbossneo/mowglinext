import {Switch, SwitchProps} from "antd";
import * as React from "react";

export const AsyncSwitch: React.FC<SwitchProps & {
    onAsyncChange: (checked: boolean, event: React.MouseEvent<HTMLButtonElement> | React.KeyboardEvent<HTMLButtonElement>) => Promise<any>
}> = (props) => {
    const {onAsyncChange, ...rest} = props;
    const [loading, setLoading] = React.useState(false)
    const handleChange = (checked: boolean, event: React.MouseEvent<HTMLButtonElement> | React.KeyboardEvent<HTMLButtonElement>) => {
        if (props.onChange !== undefined) {
            props.onChange(checked, event)
        } else if (onAsyncChange !== undefined) {
            setLoading(true)
            onAsyncChange(checked, event).then(() => {
                setLoading(false)
            }).catch(() => {
                setLoading(false)
            })
        }
    }
    return <Switch loading={loading} onChange={handleChange} {...rest}/>
}

export default AsyncSwitch;
