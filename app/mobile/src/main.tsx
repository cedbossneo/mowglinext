import { StrictMode } from 'react';
import { createRoot } from 'react-dom/client';
import { ThemeProvider } from '@/theme/ThemeProvider';
import { AuthProvider } from '@/auth/AuthProvider';
import { RobotsProvider } from '@/connection/RobotsProvider';
import { App } from './App';

const rootEl = document.getElementById('root');
if (!rootEl) throw new Error('Root element #root not found');

createRoot(rootEl).render(
  <StrictMode>
    <ThemeProvider>
      <AuthProvider>
        <RobotsProvider>
          <App />
        </RobotsProvider>
      </AuthProvider>
    </ThemeProvider>
  </StrictMode>,
);
