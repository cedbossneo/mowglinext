import {beforeEach, describe, expect, it, vi} from "vitest";
import {act, fireEvent, render, screen} from "@testing-library/react";
import userEvent from "@testing-library/user-event";
import {App} from "antd";

import {ManualControllerPage} from "./ManualControllerPage.tsx";
import en from "../i18n/locales/en.json";

const mowerAction = vi.fn(() => vi.fn().mockResolvedValue(undefined));
const joyStream = {start: vi.fn(), stop: vi.fn(), sendJsonMessage: vi.fn()};
const highLevelStatus = {state_name: "IDLE"};

vi.mock("../components/MowerActions.tsx", () => ({useMowerAction: () => mowerAction}));
vi.mock("../hooks/useHighLevelStatus.ts", () => ({useHighLevelStatus: () => ({highLevelStatus})}));
vi.mock("../hooks/useMowingMap.ts", () => ({useMowingMap: () => ({})}));
vi.mock("../hooks/useFusionOdom.ts", () => ({useFusionOdom: () => ({})}));
vi.mock("../hooks/useWS.ts", () => ({useWS: () => joyStream}));

function renderController() {
    return render(<App><ManualControllerPage/></App>);
}

async function selectTransportAndConfirm(user: ReturnType<typeof userEvent.setup>) {
    await user.click(screen.getByRole("button", {name: en.manualController.transport}));
    highLevelStatus.state_name = "MANUAL_DRIVING";
}

describe("ManualControllerPage", () => {
    beforeEach(() => {
        vi.clearAllMocks();
        highLevelStatus.state_name = "IDLE";
    });

    it("renders the combined controller and safety controls", () => {
        renderController();
        expect(screen.getByText(en.manualController.combinedStick)).toBeInTheDocument();
        expect(screen.getByRole("button", {name: en.manualController.emergency})).toBeInTheDocument();
    });

    it("selects blade-disabled transport mode", async () => {
        const user = userEvent.setup();
        const view = renderController();
        await selectTransportAndConfirm(user);
        view.rerender(<App><ManualControllerPage/></App>);
        await act(async () => {
            await new Promise(resolve => setTimeout(resolve, 650));
        });
        expect(mowerAction).toHaveBeenCalledWith("high_level_control", {Command: 9});
        expect(joyStream.start).toHaveBeenCalledWith("/api/mowglinext/publish/joy");
    });

    it("sends and clears keyboard drive velocity only after a mode is selected", async () => {
        const user = userEvent.setup();
        const view = renderController();
        fireEvent.keyDown(window, {key: "w"});
        expect(joyStream.sendJsonMessage).not.toHaveBeenCalled();

        await selectTransportAndConfirm(user);
        view.rerender(<App><ManualControllerPage/></App>);
        await act(async () => {
            await new Promise(resolve => setTimeout(resolve, 650));
        });
        fireEvent.keyDown(window, {key: "w"});
        expect(joyStream.sendJsonMessage).toHaveBeenLastCalledWith(expect.objectContaining({
            twist: expect.objectContaining({linear: expect.objectContaining({x: 0.25})}),
        }));
        fireEvent.keyUp(window, {key: "w"});
        expect(joyStream.sendJsonMessage).toHaveBeenLastCalledWith(expect.objectContaining({
            twist: expect.objectContaining({linear: expect.objectContaining({x: 0})}),
        }));
    });

    it("combines forward input with left and right steering", async () => {
        const user = userEvent.setup();
        const view = renderController();
        await selectTransportAndConfirm(user);
        view.rerender(<App><ManualControllerPage/></App>);
        await act(async () => {
            await new Promise(resolve => setTimeout(resolve, 650));
        });

        fireEvent.keyDown(window, {key: "w"});
        fireEvent.keyDown(window, {key: "a"});
        expect(joyStream.sendJsonMessage).toHaveBeenLastCalledWith(expect.objectContaining({
            twist: expect.objectContaining({
                linear: expect.objectContaining({x: 0.25}),
                angular: expect.objectContaining({z: 0.6}),
            }),
        }));

        fireEvent.keyUp(window, {key: "a"});
        fireEvent.keyDown(window, {key: "d"});
        expect(joyStream.sendJsonMessage).toHaveBeenLastCalledWith(expect.objectContaining({
            twist: expect.objectContaining({
                linear: expect.objectContaining({x: 0.25}),
                angular: expect.objectContaining({z: -0.6}),
            }),
        }));
    });

    it("uses the stop-in-place command", async () => {
        const user = userEvent.setup();
        renderController();
        await user.click(screen.getByRole("button", {name: en.manualController.stop}));
        expect(mowerAction).toHaveBeenCalledWith("high_level_control", {Command: 8});
    });
});
