import { defineConfig } from 'vite';

export default defineConfig({
  base: './',
  server: {
    strictPort: true,
    proxy: {
      '/hivedocs': {
        target: 'http://127.0.0.1:5174',
        changeOrigin: true,
        ws: true
      }
    }
  }
});
