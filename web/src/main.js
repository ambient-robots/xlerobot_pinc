import './style.css';
import { createScene } from './scene.js';
import { loadRobotAssembly } from './robot-loader.js';
import { JointController } from './joint-controller.js';
import { createUI } from './ui.js';

async function init() {
  const viewerContainer = document.getElementById('viewer');
  const loadingEl = document.getElementById('loading');

  const { scene } = createScene(viewerContainer);

  try {
    const assembly = await loadRobotAssembly(scene);
    const controller = new JointController(assembly);
    createUI(document.body, controller);
    loadingEl?.remove();
  } catch (err) {
    console.error('Failed to load robot:', err);
    if (loadingEl) {
      loadingEl.innerHTML = `
        <div class="text-center px-6">
          <p class="text-sm text-stone-600 font-medium">Failed to load robot model</p>
          <p class="text-xs text-stone-400 mt-1">${err.message || 'Unknown error'}</p>
        </div>
      `;
    }
  }
}

init();
