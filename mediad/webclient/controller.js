// Controller edits are staged locally; defaults and allowed choices come from the robot.
(function (scope) {
  class ControllerDraft {
    constructor(report) {
      this.settings = report.settings;
      this.changes = {};
    }
    value(setting) {
      return Object.hasOwn(this.changes, setting.key)
        ? (this.changes[setting.key] === null ? setting.default_value : this.changes[setting.key])
        : setting.value;
    }
    set(setting, value) {
      if (JSON.stringify(value) === JSON.stringify(setting.value)) delete this.changes[setting.key];
      // A profile edit must stay explicit: clearing it would revive legacy pad_axes overrides.
      else this.changes[setting.key] = setting.kind !== 'modes' && JSON.stringify(value) === JSON.stringify(setting.default_value) ? null : value;
    }
    reset() {
      for (const setting of this.settings) {
        if (setting.overridden || Object.hasOwn(this.changes, setting.key) || JSON.stringify(this.value(setting)) !== JSON.stringify(setting.default_value)) this.changes[setting.key] = null;
      }
    }
    cancel() { this.changes = {}; }
    get dirty() { return Object.keys(this.changes).length > 0; }
  }

  function createControllerEditor(root, call, connected) {
    let draft = null;
    let busy = false;
    const form = root.querySelector('form');
    const status = root.querySelector('[data-controller-status]');
    const fields = root.querySelector('[data-controller-fields]');
    const save = root.querySelector('[data-controller-save]');
    const cancel = root.querySelector('[data-controller-cancel]');
    const reset = root.querySelector('[data-controller-reset]');
    const load = root.querySelector('[data-controller-load]');
    const mode = root.querySelector('[data-controller-mode]');
    const modeName = root.querySelector('[data-mode-name]');
    const modeButton = root.querySelector('[data-mode-button]');
    const addMode = root.querySelector('[data-mode-add]');
    const removeMode = root.querySelector('[data-mode-remove]');
    const upMode = root.querySelector('[data-mode-up]');
    const downMode = root.querySelector('[data-mode-down]');
    const modeKey = 'pad_modes.profiles';
    const clone = value => JSON.parse(JSON.stringify(value));
    const modeSetting = () => draft?.settings.find(s => s.key === modeKey);
    const profiles = () => modeSetting() ? clone(draft.value(modeSetting())) : [];
    const buttons = { none: 'No button', a: 'A', b: 'B', x: 'X', y: 'Y', lb: 'Left bumper', rb: 'Right bumper', dpad_up: 'D-pad up', dpad_right: 'D-pad right', dpad_down: 'D-pad down', dpad_left: 'D-pad left', left_stick: 'Left stick click', right_stick: 'Right stick click' };
    function stageModes(list) { draft.set(modeSetting(), list); message('Changes are not saved yet.'); update(); }
    function reserved(button) {
      return profiles().some(m => m.button === button) || draft.settings.some(s => ['pad_modes.next_button', 'pad_modes.previous_button'].includes(s.key) && draft.value(s) === button);
    }
    const axes = { none: 'Disabled', left_x: 'Left stick — horizontal', left_y: 'Left stick — vertical', right_x: 'Right stick — horizontal', right_y: 'Right stick — vertical' };
    const labels = {
      vx: 'Forward / backward', vy: 'Sideways', vyaw: 'Turn', neck_pitch: 'Neck tilt',
      head_pitch: 'Head tilt', head_yaw: 'Look left / right', head_roll: 'Head lean', next_button: 'Next mode button', previous_button: 'Previous mode button',
      z: 'Rise / crouch', pitch: 'Body tilt', roll: 'Body lean', deadzone: 'Stick deadzone',
      enabled: 'Enable controller tilt input', gain: 'Tilt sensitivity',
    };
    const sections = { pad_modes: 'Mode switching', pad: 'Button skills', pad_axes: 'Stick settings', pad_drive: 'Walking speed limits', pad_head: 'Head travel limits', pad_body: 'Body travel limits', pad_roller: 'Roller speed limits', pad_imu_head_control: 'Controller tilt' };

    function message(text, error = false) {
      status.textContent = text;
      status.classList.toggle('controller-error', error);
    }
    function update() {
      const live = connected() && !busy;
      const list = profiles();
      const index = list.findIndex(p => p.id === mode.value);
      addMode.disabled = !live || !draft;
      removeMode.disabled = !live || list.length < 2;
      upMode.disabled = !live || index < 1;
      downMode.disabled = !live || index < 0 || index >= list.length - 1;
      mode.disabled = busy || !draft;
      modeName.disabled = modeButton.disabled = !live || index < 0;
      save.disabled = !live || !draft?.dirty;
      cancel.disabled = busy || !draft?.dirty;
      reset.disabled = !live || !draft;
      load.disabled = !live;
      for (const input of fields.querySelectorAll('input, select, button')) input.disabled = !live || input.dataset.modeReserved === 'true';
    }
    function element(tag, text, parent) {
      const el = document.createElement(tag);
      if (text !== undefined) el.textContent = text;
      if (parent) parent.append(el);
      return el;
    }
    function label(setting) {
      const parts = setting.key.split('.');
      const name = parts.at(-1);
      if (parts[0] === 'pad') return name.toUpperCase();
      if (parts[0] === 'pad_axes' && parts.length > 2) return { source: 'Stick axis', invert: 'Reverse direction', gain: 'Sensitivity' }[name];
      if (name.endsWith('_max') || name.endsWith('_min')) {
        const axis = name.replace(/_(max|min)$/, '');
        const unit = parts[0] === 'pad_head' ? 'rad' : parts[0] === 'pad_body' ? (axis === 'z' ? 'm' : 'rad') : axis === 'vyaw' ? 'rad/s' : 'm/s';
        const limit = parts[0] === 'pad_head' || (parts[0] === 'pad_body' && axis !== 'z') ? 'maximum travel' : name.endsWith('_max') ? 'positive limit' : 'negative limit';
        return `${labels[axis] || axis} — ${limit} (${unit})`;
      }
      return labels[name] || name;
    }
    function inputFor(setting, parent, onChange) {
      const row = element('label', undefined, parent);
      row.className = 'controller-field';
      row.title = setting.description || '';
      element('span', label(setting), row);
      let input;
      if (setting.kind === 'skill' || setting.kind === 'choice') {
        input = element('select', undefined, row);
        const choices = setting.kind === 'skill' ? ['', ...setting.choices] : [...setting.choices];
        const value = draft.value(setting);
        if (!choices.includes(value)) choices.push(value);
        for (const choice of choices) {
          const option = element('option', setting.kind === 'skill' ? (choice || 'No action') : (setting.key.startsWith('pad_modes.') ? buttons[choice] : axes[choice]) || choice, input);
          option.value = choice;
        }
        input.value = value;
      } else {
        input = element('input', undefined, row);
        input.type = setting.kind === 'boolean' ? 'checkbox' : 'number';
        if (input.type === 'checkbox') input.checked = draft.value(setting);
        else { input.step = 'any'; input.value = draft.value(setting); input.required = true; }
      }
      input.dataset.setting = setting.key;
      if (setting.kind === 'skill' && reserved(setting.key.split('.')[1])) input.dataset.modeReserved = 'true';
      const marker = element('small', '', row);
      function mark() {
        marker.textContent = input.dataset.modeReserved === 'true' ? 'Used for mode switching' : Object.hasOwn(draft.changes, setting.modeField ? modeKey : setting.key) ? 'Unsaved' : setting.overridden ? 'Custom' : 'Default';
      }
      mark();
      input.addEventListener('input', () => {
        const value = setting.kind === 'boolean' ? input.checked : setting.kind === 'number' ? input.valueAsNumber : input.value;
        if (setting.kind === 'number' && !Number.isFinite(value)) { input.setCustomValidity('Enter a finite number.'); save.disabled = true; return; }
        input.setCustomValidity('');
        if (onChange) onChange(value);
        else { draft.set(setting, value); if (setting.key.startsWith("pad_modes.")) render(); }
        mark(); update();
        message(draft.dirty ? 'Changes are not saved yet.' : 'No unsaved changes.');
      });
    }
    function render() {
      fields.replaceChildren();
      if (!draft) return;
      const list = profiles();
      const selected = list.find(p => p.id === mode.value) || list[0];
      mode.replaceChildren();
      for (const profile of list) { const option = element('option', profile.name, mode); option.value = profile.id; }
      if (selected) mode.value = selected.id;
      modeName.value = selected?.name || '';
      modeButton.replaceChildren();
      for (const button of modeSetting()?.choices || []) { const option = element('option', buttons[button] || button, modeButton); option.value = button; }
      modeButton.value = selected?.button || 'none';
      for (const [section, title] of Object.entries(sections)) {
        const settings = draft.settings.filter(s => s.key.startsWith(`${section}.`) && s.kind !== 'modes' &&
          (section !== 'pad_axes' || s.key.split('.').length === 2));
        if (!settings.length) continue;
        const group = element('fieldset', undefined, fields);
        element('legend', title, group);
        for (const setting of settings) inputFor(setting, group);
      }
      if (selected) {
        const mappingGroups = [];
        const group = element('fieldset', undefined, fields);
        element('legend', 'Controls in this mode', group);
        mappingGroups.push(group);
        for (const [key, title] of Object.entries({ drive: 'Movement', head: 'Head', body: 'Body posture', imu_head: 'Controller tilt for head', imu_drive: 'Separate stick mappings while tilting' })) {
          const row = element('label', title, group);
          row.className = 'controller-field';
          const input = element('input', undefined, row); input.type = 'checkbox'; input.checked = Boolean(selected[key]);
          input.dataset.modeChannel = key;
          input.addEventListener('change', () => {
            if (input.checked) {
              if ((key === 'imu_head' && !selected.head) || (key === 'imu_drive' && (!selected.imu_head || !selected.drive))) { input.checked = false; message('Enable head control for tilt, and movement control for separate tilt mappings.', true); return; }
              selected[key] = key === 'imu_head' ? true : clone(key === 'imu_drive' ? selected.drive : modeSetting().default_value.find(p => p[key])[key]);
            } else {
              if (key === 'imu_head') selected.imu_head = false; else delete selected[key];
              if (key === 'head') selected.imu_head = false;
              if (['head', 'drive', 'imu_head'].includes(key)) delete selected.imu_drive;
            }
            stageModes(list); render();
          });
        }
        for (const [channel, title] of Object.entries({ drive: 'Movement mappings', head: 'Head mappings', body: 'Body mappings', imu_drive: 'Movement mappings while tilting' })) {
          if (!selected[channel]) continue;
          const group = element('fieldset', undefined, fields); element('legend', title, group); mappingGroups.push(group);
          for (const [axis, binding] of Object.entries(selected[channel])) {
            const axisGroup = element('div', undefined, group); axisGroup.className = 'controller-axis';
            element('h3', labels[axis] || axis, axisGroup);
            for (const [property, kind] of Object.entries({ source: 'choice', invert: 'boolean', gain: 'number' })) {
              const shipped = modeSetting().default_value.find(p => p.id === selected.id)?.[channel]?.[axis]?.[property];
              const sources = draft.settings.find(s => s.key.startsWith('pad_axes.') && s.key.endsWith('.source'))?.choices || [];
              const setting = { key: `pad_axes.${selected.id}.${channel}.${axis}.${property}`, kind, modeField: true, value: binding[property], default_value: shipped ?? binding[property], overridden: shipped !== binding[property], choices: sources, description: labels[axis] || axis };
              inputFor(setting, axisGroup, value => { binding[property] = value; stageModes(list); });
            }
          }
        }
        const before = fields.children[1];
        for (const group of mappingGroups) fields.insertBefore(group, before);
      }
      update();
    }
    modeName.addEventListener('input', () => {
      const list = profiles(), selected = list.find(p => p.id === mode.value);
      if (!selected) return;
      selected.name = modeName.value;
      stageModes(list); render();
      mode.querySelector('option:checked').textContent = selected.name;
    });
    modeButton.addEventListener('change', () => {
      const list = profiles(), selected = list.find(p => p.id === mode.value);
      if (!selected) return;
      if (modeButton.value !== 'none') selected.button = modeButton.value; else delete selected.button;
      stageModes(list); render();
    });
    addMode.addEventListener('click', () => {
      const list = profiles();
      const profile = clone(list.find(p => p.id === mode.value) || modeSetting().default_value[0]);
      let number = 1; while (list.some(p => p.id === `mode_${number}`)) number++;
      profile.id = `mode_${number}`; profile.name = `Mode ${number}`; delete profile.button;
      list.push(profile); stageModes(list); render(); mode.value = profile.id; render(); modeName.focus();
    });
    removeMode.addEventListener('click', () => {
      const list = profiles().filter(p => p.id !== mode.value);
      if (!list.length) return;
      stageModes(list); render();
    });
    function moveMode(offset) {
      const list = profiles(), index = list.findIndex(p => p.id === mode.value), other = index + offset;
      if (index < 0 || other < 0 || other >= list.length) return;
      [list[index], list[other]] = [list[other], list[index]]; stageModes(list); render();
    }
    upMode.addEventListener('click', () => moveMode(-1));
    downMode.addEventListener('click', () => moveMode(1));
    async function refresh() {
      if (busy) return;
      if (!connected()) { message('Connect to a robot to edit its controller.'); update(); return; }
      // Reconnection must not silently discard edits made before the connection dropped.
      if (draft?.dirty) { message('Your unsaved changes are still here. Save or cancel them before reloading.'); update(); return; }
      busy = true; update(); message('Reading controller settings…');
      try { draft = new ControllerDraft(await call('pad.config')); render(); message('Settings loaded. Changes apply within a second after saving.'); }
      catch (error) { message(error.message || String(error), true); }
      finally { busy = false; update(); }
    }
    form.addEventListener('submit', async event => {
      event.preventDefault();
      if (!draft?.dirty || busy || !connected() || !form.reportValidity()) return;
      busy = true; update(); message('Saving…');
      let saved = false;
      try {
        const result = await call('pad.setConfig', { changes: { ...draft.changes } });
        if (!result.accepted) throw new Error(result.reason || 'The robot refused these settings.');
        saved = true;
        draft = null;
        render();
        draft = new ControllerDraft(await call('pad.config'));
        render(); message('Saved. Controller settings apply within a second.');
      } catch (error) { message(saved ? 'Saved, but settings could not be reloaded. Reconnect and click Reload.' : (error.message || String(error)), true); }
      finally { busy = false; update(); }
    });
    cancel.addEventListener('click', () => { draft?.cancel(); render(); message('Unsaved changes discarded.'); });
    reset.addEventListener('click', () => { draft?.reset(); render(); message('Defaults selected. Click Save to apply them.'); });
    load.addEventListener('click', refresh);
    mode.addEventListener('change', render);
    update();
    return { refresh, connectionChanged: update };
  }
  scope.ControllerDraft = ControllerDraft;
  scope.createControllerEditor = createControllerEditor;
})(typeof window === 'undefined' ? globalThis : window);
