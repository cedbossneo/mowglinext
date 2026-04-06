import {Joystick} from "react-joystick-component";
import {IJoystickUpdateEvent} from "react-joystick-component/build/lib/Joystick";
import {CheckOutlined, HomeOutlined} from "@ant-design/icons";
import {AsyncButton} from "@mowglinext/common";

interface JoystickOverlayProps {
    visible: boolean;
    isRecording?: boolean;
    onMove: (event: IJoystickUpdateEvent) => void;
    onStop: () => void;
    onFinishRecording?: () => Promise<void>;
    onHome?: () => Promise<void>;
}

export const JoystickOverlay = ({visible, isRecording, onMove, onStop, onFinishRecording, onHome}: JoystickOverlayProps) => {
    if (!visible) return null;
    return (
        <div style={{position: "absolute", bottom: 30, right: 30, zIndex: 100, display: 'flex', alignItems: 'flex-end', gap: 12}}>
            {isRecording && (
                <div style={{
                    display: 'flex',
                    flexDirection: 'column',
                    gap: 8,
                    marginBottom: 10,
                }}>
                    <AsyncButton
                        type="primary"
                        icon={<CheckOutlined />}
                        onAsyncClick={onFinishRecording!}
                        style={{height: 44, borderRadius: 10, fontWeight: 600}}
                    >
                        Finish
                    </AsyncButton>
                    <AsyncButton
                        icon={<HomeOutlined />}
                        onAsyncClick={onHome!}
                        style={{height: 44, borderRadius: 10}}
                    >
                        Home
                    </AsyncButton>
                </div>
            )}
            <Joystick move={onMove} stop={onStop}/>
        </div>
    );
};
