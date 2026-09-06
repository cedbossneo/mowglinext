import {test, expect} from '@playwright/test';
import {installMockBackend} from './mock/mockBackend';
import {pack} from 'msgpackr';

for (const route of ['diagnostics', 'mowglinext']) {
    test(`requested blade direction on ${route}`, async ({page}) => {
        await page.addInitScript(() => localStorage.setItem('mowglinext.lang', 'en'));
        const status = {
                mower_motor_rpm: 3100, mower_esc_current: 0.6,
                mower_motor_temperature: 25, mower_esc_temperature: 24,
                mower_esc_status: 1, mow_enabled: true,
                blade_requested_direction: 'reverse',
                firmware_compatible: true, firmware_version: '1.9.205',
        };
        await installMockBackend(page, {name: 'blade-reverse'});
        let stopUpdates = () => {};
        await page.routeWebSocket(/\/api\/mowglinext\/multiplex/, (ws) => {
            const timer = setInterval(() => ws.send(Buffer.from(pack({topic: 'status', data: status}))), 250);
            stopUpdates = () => clearInterval(timer);
            ws.onClose(() => { clearInterval(timer); ws.close(); });
        });
        await page.goto(`/#/${route}`);
        if (route === 'diagnostics') await page.getByRole('tab', {name: /Robot/}).click();
        const display = page.getByLabel('Requested direction: Reverse', {exact: true});
        await expect(display).toBeVisible();
        await display.scrollIntoViewIfNeeded();
        await page.screenshot({path: `tests/e2e/.artifacts/blade-direction-${route}.png`});
        stopUpdates();
        // A stopped stream must not leave a
        // apparently live rotation direction displayed indefinitely.
        await expect(page.getByLabel('Requested direction: Unknown', {exact: true})).toBeVisible({timeout: 7000});
    });
}
