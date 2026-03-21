import { ARM_JOINTS, GRIPPER_JOINT } from './joint-controller.js';
import { HOME_VALUES, ANIMATION_SEQUENCE } from './presets.js';

const RAD_TO_DEG = 180 / Math.PI;

const SIDES = [
  { key: 'left', label: 'Left Arm' },
  { key: 'right', label: 'Right Arm' },
];

export function createUI(container, jointController) {
  // Panel
  const panel = document.createElement('div');
  panel.id = 'control-panel';
  panel.className = [
    'fixed bottom-0 left-0 right-0 z-50',
    'bg-white/90 backdrop-blur-md',
    'border-t border-stone-200/80',
    'flex flex-col',
    'max-h-[55vh]',
    'transition-transform duration-300 ease-in-out',
    'shadow-lg',
  ].join(' ');

  // Mobile drag handle
  const handle = document.createElement('div');
  handle.className = 'flex justify-center py-2.5 cursor-pointer select-none';
  handle.innerHTML = '<div class="w-10 h-1 rounded-full bg-stone-300"></div>';
  panel.appendChild(handle);

  // Header
  const header = document.createElement('div');
  header.className = 'px-5 py-3 border-b border-stone-100 flex items-center justify-between';
  header.innerHTML = `
    <div>
      <h2 class="text-xs font-semibold text-stone-800 tracking-widest uppercase">Joint Control</h2>
      <p class="text-[10px] text-stone-400 mt-0.5">XLerobot Pinc — Dual Arm</p>
    </div>
  `;
  panel.appendChild(header);

  // Action buttons row
  const actionsRow = document.createElement('div');
  actionsRow.className = 'px-5 py-3 flex gap-2 border-b border-stone-100';

  const btnBase = [
    'flex-shrink-0 px-4 py-2 rounded-full text-sm font-medium',
    'active:scale-95',
    'transition-all duration-150',
  ].join(' ');

  // Home button
  const homeBtn = document.createElement('button');
  homeBtn.className = `${btnBase} bg-stone-100 text-stone-600 hover:bg-stone-800 hover:text-white`;
  homeBtn.textContent = 'Home';

  // Animate button
  const animateBtn = document.createElement('button');
  animateBtn.className = `${btnBase} bg-stone-100 text-stone-600 hover:bg-stone-800 hover:text-white`;
  animateBtn.textContent = 'Animate';

  let animating = false;
  let stopAnimation = null;

  function startAnimation() {
    animating = true;
    animateBtn.textContent = 'Stop';
    animateBtn.classList.remove('bg-stone-100', 'text-stone-600');
    animateBtn.classList.add('bg-stone-800', 'text-white');

    let cancelled = false;
    stopAnimation = () => { cancelled = true; };

    (async () => {
      while (!cancelled) {
        for (const step of ANIMATION_SEQUENCE) {
          if (cancelled) break;
          await jointController.animateTo(step.values, step.duration);
          if (cancelled) break;
          await new Promise((r) => setTimeout(r, 200));
        }
      }
      animating = false;
      stopAnimation = null;
      animateBtn.textContent = 'Animate';
      animateBtn.classList.remove('bg-stone-800', 'text-white');
      animateBtn.classList.add('bg-stone-100', 'text-stone-600');
    })();
  }

  function stopAnim() {
    if (stopAnimation) stopAnimation();
  }

  homeBtn.addEventListener('click', () => {
    stopAnim();
    jointController.animateTo(HOME_VALUES, 600);
  });

  animateBtn.addEventListener('click', () => {
    if (animating) {
      stopAnim();
    } else {
      startAnimation();
    }
  });

  actionsRow.appendChild(homeBtn);
  actionsRow.appendChild(animateBtn);
  panel.appendChild(actionsRow);

  // Arm tabs
  const tabsRow = document.createElement('div');
  tabsRow.className = 'flex border-b border-stone-100';
  const tabButtons = [];
  const tabPanels = [];

  for (const side of SIDES) {
    const tabBtn = document.createElement('button');
    tabBtn.className = 'flex-1 py-3 text-sm font-semibold tracking-wide uppercase transition-colors';
    tabBtn.textContent = side.label;
    tabBtn.dataset.side = side.key;
    tabButtons.push(tabBtn);
    tabsRow.appendChild(tabBtn);
  }
  panel.appendChild(tabsRow);

  // Scrollable sliders container
  const slidersContainer = document.createElement('div');
  slidersContainer.className = 'flex-1 overflow-y-auto';

  const sliderElements = {};

  for (const side of SIDES) {
    const tabPanel = document.createElement('div');
    tabPanel.className = 'px-5 py-4 space-y-5';
    tabPanel.dataset.side = side.key;

    const joints = [...ARM_JOINTS, GRIPPER_JOINT];

    for (const joint of joints) {
      const prefixedName = `${side.key}_${joint.name}`;

      const wrapper = document.createElement('div');

      const labelRow = document.createElement('div');
      labelRow.className = 'flex justify-between items-center mb-1.5';

      const label = document.createElement('label');
      label.className = 'text-sm font-medium text-stone-600';
      label.textContent = joint.label;

      const valueDisplay = document.createElement('span');
      valueDisplay.className = 'text-[11px] font-mono text-stone-400 tabular-nums';
      valueDisplay.textContent = '0.0°';

      labelRow.appendChild(label);
      labelRow.appendChild(valueDisplay);

      const slider = document.createElement('input');
      slider.type = 'range';
      slider.min = joint.min;
      slider.max = joint.max;
      slider.step = 0.01;
      slider.value = 0;
      slider.className = 'w-full h-1.5';

      slider.addEventListener('input', () => {
        stopAnim();
        const val = parseFloat(slider.value);
        jointController.setJoint(prefixedName, val);
        valueDisplay.textContent = `${(val * RAD_TO_DEG).toFixed(1)}°`;
      });

      sliderElements[prefixedName] = { slider, valueDisplay };

      wrapper.appendChild(labelRow);
      wrapper.appendChild(slider);
      tabPanel.appendChild(wrapper);
    }

    tabPanels.push(tabPanel);
    slidersContainer.appendChild(tabPanel);
  }

  panel.appendChild(slidersContainer);

  // Tab switching
  function activateTab(sideKey) {
    tabButtons.forEach((btn) => {
      if (btn.dataset.side === sideKey) {
        btn.classList.add('text-stone-800', 'border-b-2', 'border-stone-800');
        btn.classList.remove('text-stone-400');
      } else {
        btn.classList.remove('text-stone-800', 'border-b-2', 'border-stone-800');
        btn.classList.add('text-stone-400');
      }
    });
    tabPanels.forEach((p) => {
      p.style.display = p.dataset.side === sideKey ? '' : 'none';
    });
  }

  tabButtons.forEach((btn) => {
    btn.addEventListener('click', () => activateTab(btn.dataset.side));
  });

  activateTab('left');

  // Sync sliders on external changes (presets)
  jointController.onChange((name, value) => {
    const el = sliderElements[name];
    if (el) {
      el.slider.value = value;
      el.valueDisplay.textContent = `${(value * RAD_TO_DEG).toFixed(1)}°`;
    }
  });

  // Mobile collapse
  let collapsed = false;
  handle.addEventListener('click', () => {
    collapsed = !collapsed;
    panel.style.transform = collapsed ? 'translateY(calc(100% - 36px))' : '';
  });

  container.appendChild(panel);
}
