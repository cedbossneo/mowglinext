import * as React from "react";
import { DropdownButtonProps } from "antd/es/dropdown";
export declare const AsyncDropDownButton: React.FC<DropdownButtonProps & {
    menu: DropdownButtonProps["menu"] & {
        onAsyncClick: (event: any) => Promise<any>;
    };
}>;
export default AsyncDropDownButton;
//# sourceMappingURL=AsyncDropDownButton.d.ts.map