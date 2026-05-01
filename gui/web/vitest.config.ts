import {defineConfig} from 'vitest/config'
import react from '@vitejs/plugin-react'

export default defineConfig({
    plugins: [react()],
    test: {
        environment: 'jsdom',
        globals: true,
        setupFiles: ['./src/test/setup.ts'],
        css: true,
        // Playwright e2e tests are NOT Vitest specs — exclude the e2e/
        // directory so `yarn test --run` doesn't try to import them.
        exclude: ['**/node_modules/**', '**/dist/**', 'e2e/**'],
    },
})
