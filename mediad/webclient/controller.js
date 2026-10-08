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
      if (value === setting.value) delete this.changes[setting.key];
      else this.changes[setting.key] = value === setting.default_value ? null : value;
    }
    reset() {
      for (const setting of this.settings) {
        if (setting.overridden || this.value(setting) !== setting.default_value) this.changes[setting.key] = null;
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
    const modes = { drive: 'Move', head: 'Head', head_drive: 'Head + move', body_pose: 'Body + head' };
    const axes = { none: 'Disabled', left_x: 'Left stick — horizontal', left_y: 'Left stick — vertical', right_x: 'Right stick — horizontal', right_y: 'Right stick — vertical' };
    const labels = {
      vx: 'Forward / backward', vy: 'Sideways', vyaw: 'Turn', neck_pitch: 'Neck tilt',
      head_pitch: 'Head tilt', head_yaw: 'Look left / right', head_roll: 'Head lean',
      z: 'Rise / crouch', pitch: 'Body tilt', roll: 'Body lean', deadzone: 'Stick deadzone',
      enabled: 'Tilt the controller to move the head', gain: 'Tilt sensitivity',
    };
    const sections = { pad: 'Buttons', pad_axes: 'Stick settings', pad_drive: 'Walking speed limits', pad_head: 'Head travel limits', pad_body: 'Body travel limits', pad_roller: 'Roller speed limits', pad_imu_head_control: 'Controller tilt' };

    function message(text, error = false) {
      status.textContent = text;
      status.classList.toggle('controller-error', error);
    }
    function update() {
      const live = connected() && !busy;
      save.disabled = !live || !draft?.dirty;
      cancel.disabled = busy || !draft?.dirty;
      reset.disabled = !live || !draft;
      load.disabled = !live;
      for (const input of fields.querySelectorAll('input, select, button')) input.disabled = !live;
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
    function inputFor(setting, parent) {
      const row = element('label', undefined, parent);
      row.className = 'controller-field';
      row.title = setting.description;
      element('span', label(setting), row);
      let input;
      if (setting.kind === 'skill' || setting.kind === 'choice') {
        input = element('select', undefined, row);
        const choices = setting.kind === 'skill' ? ['', ...setting.choices] : [...setting.choices];
        const value = draft.value(setting);
        if (!choices.includes(value)) choices.push(value);
        for (const choice of choices) {
          const option = element('option', setting.kind === 'skill' ? (choice || 'No action') : (axes[choice] || choice), input);
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
      const marker = element('small', '', row);
      function mark() {
        marker.textContent = Object.hasOwn(draft.changes, setting.key) ? 'Unsaved' : setting.overridden ? 'Custom' : 'Default';
      }
      mark();
      input.addEventListener('input', () => {
        const value = setting.kind === 'boolean' ? input.checked : setting.kind === 'number' ? input.valueAsNumber : input.value;
        if (setting.kind === 'number' && !Number.isFinite(value)) { input.setCustomValidity('Enter a finite number.'); save.disabled = true; return; }
        input.setCustomValidity('');
        draft.set(setting, value); mark(); update();
        message(draft.dirty ? 'Changes are not saved yet.' : 'No unsaved changes.');
      });
    }
    function render() {
      fields.replaceChildren();
      if (!draft) return;
      for (const [section, title] of Object.entries(sections)) {
        const settings = draft.settings.filter(s => s.key.startsWith(`${section}.`) &&
          (section !== 'pad_axes' || s.key.split('.').length === 2 || s.key.startsWith(`pad_axes.${mode.value}.`)));
        if (!settings.length) continue;
        const group = element('fieldset', undefined, fields);
        element('legend', title, group);
        let axisGroup = null, axisPrefix = null;
        for (const setting of settings) {
          if (section === 'pad_axes' && setting.key.split('.').length > 2) {
            const prefix = setting.key.split('.').slice(0, -1).join('.');
            if (prefix !== axisPrefix) {
              axisPrefix = prefix;
              axisGroup = element('div', undefined, group); axisGroup.className = 'controller-axis';
              element('h3', labels[prefix.split('.').at(-1)] || prefix, axisGroup);
            }
            inputFor(setting, axisGroup);
          } else inputFor(setting, group);
        }
      }
      update();
    }
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
    for (const [key, title] of Object.entries(modes)) { const option = element('option', title, mode); option.value = key; }
    update();
    return { refresh, connectionChanged: update };
  }
  scope.ControllerDraft = ControllerDraft;
  scope.createControllerEditor = createControllerEditor;
})(typeof window === 'undefined' ? globalThis : window);
