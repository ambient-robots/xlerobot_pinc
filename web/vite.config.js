import tailwindcss from '@tailwindcss/vite';
import { defineConfig } from 'vite';
import fs from 'fs';
import path from 'path';

const URDF_DIR = path.resolve(import.meta.dirname, '../xlerobot_pinc_urdf');

function urdfAssets() {
  return {
    name: 'urdf-assets',

    configureServer(server) {
      server.middlewares.use((req, res, next) => {
        const reqPath = req.url.split('?')[0];
        if (reqPath.endsWith('.urdf') || reqPath.startsWith('/assets/')) {
          const filePath = path.join(URDF_DIR, reqPath);
          if (fs.existsSync(filePath)) {
            const ext = path.extname(filePath);
            const types = { '.urdf': 'application/xml', '.stl': 'model/stl' };
            if (types[ext]) res.setHeader('Content-Type', types[ext]);
            return res.end(fs.readFileSync(filePath));
          }
        }
        next();
      });
    },

    writeBundle() {
      const outDir = path.resolve(import.meta.dirname, 'dist');

      for (const f of ['xlerobot.urdf', 'robot.urdf', 'gripper.urdf']) {
        fs.copyFileSync(path.join(URDF_DIR, f), path.join(outDir, f));
      }

      const assetsOut = path.join(outDir, 'assets');
      fs.mkdirSync(assetsOut, { recursive: true });
      for (const f of fs.readdirSync(path.join(URDF_DIR, 'assets'))) {
        if (f.endsWith('.stl')) {
          fs.copyFileSync(
            path.join(URDF_DIR, 'assets', f),
            path.join(assetsOut, f)
          );
        }
      }
    },
  };
}

export default defineConfig({
  plugins: [tailwindcss(), urdfAssets()],
  base: './',
  build: {
    outDir: 'dist',
  },
});
