import {beforeEach, describe, expect, it} from "vitest";
import {fireEvent, render, screen} from "@testing-library/react";
import {ThemeProvider, useThemeMode} from "../../theme/ThemeContext.tsx";
import {DisplayModeSection} from "./DisplayModeSection.tsx";

function ModeValue() {
    return <output data-testid="display-mode-value">{useThemeMode().displayMode}</output>;
}

describe("DisplayModeSection", () => {
    beforeEach(() => {
        const storage = new Map<string, string>();
        Object.defineProperty(window, "localStorage", {
            configurable: true,
            value: {
                getItem: (key: string) => storage.get(key) ?? null,
                setItem: (key: string, value: string) => storage.set(key, value),
                removeItem: (key: string) => storage.delete(key),
                clear: () => storage.clear(),
            },
        });
    });

    it("offers all modes and updates the shared preference", () => {
        render(
            <ThemeProvider>
                <DisplayModeSection/>
                <ModeValue/>
            </ThemeProvider>,
        );

        expect(screen.getByRole("radio", {name: /^Visual/})).toBeInTheDocument();
        expect(screen.getByRole("radio", {name: /^Balanced/})).toBeChecked();
        expect(screen.getByRole("radio", {name: /^Efficient/})).toBeInTheDocument();

        fireEvent.click(screen.getByRole("radio", {name: /^Efficient/}));
        expect(screen.getByTestId("display-mode-value")).toHaveTextContent("efficient");
    });
});
