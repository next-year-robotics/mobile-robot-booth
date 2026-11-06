import { defineConfig } from 'vite';

// 배포는 브리지가 dist를 직접 낸다. `npm run dev`로 화면만 고칠 때는 /ws를 브리지로 넘긴다.
export default defineConfig({
  base: './',
  build: { outDir: 'dist', assetsInlineLimit: 0, chunkSizeWarningLimit: 1500 },
  server: { proxy: { '/ws': { target: 'ws://127.0.0.1:8080', ws: true } } },
});
