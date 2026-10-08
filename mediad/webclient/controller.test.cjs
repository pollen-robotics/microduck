const { test } = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const vm = require('node:vm');
const context = {};
vm.runInNewContext(fs.readFileSync(`${__dirname}/controller.js`, 'utf8'), context);
const settings = [
  { key: 'pad.a', value: 'roulade', default_value: 'sit_toggle', overridden: true },
  { key: 'pad_axes.drive.vx.gain', value: 0.5, default_value: 1, overridden: true },
  { key: 'pad_axes.head.head_yaw.invert', value: true, default_value: true, overridden: false },
];
const changes = draft => JSON.parse(JSON.stringify(draft.changes));
test('edits stay local and cancellation restores the robot snapshot', () => {
  const draft = new context.ControllerDraft({ settings });
  draft.set(settings[1], 0.8);
  assert.equal(draft.value(settings[1]), 0.8);
  assert.equal(settings[1].value, 0.5);
  assert.deepEqual(changes(draft), { 'pad_axes.drive.vx.gain': 0.8 });
  draft.cancel();
  assert.equal(draft.value(settings[1]), 0.5);
  assert.equal(draft.dirty, false);
});
test('reset removes overrides on save instead of pinning a copy of defaults', () => {
  const draft = new context.ControllerDraft({ settings });
  draft.set(settings[2], false);
  draft.reset();
  assert.deepEqual(changes(draft), { 'pad.a': null, 'pad_axes.drive.vx.gain': null, 'pad_axes.head.head_yaw.invert': null });
  assert.equal(draft.value(settings[0]), 'sit_toggle');
  assert.equal(draft.value(settings[2]), true);
});
test('returning to the loaded value drops the edit; choosing its default sends reset', () => {
  const draft = new context.ControllerDraft({ settings });
  draft.set(settings[1], 0.8); draft.set(settings[1], 0.5);
  assert.equal(draft.dirty, false);
  draft.set(settings[1], 1);
  assert.deepEqual(changes(draft), { 'pad_axes.drive.vx.gain': null });
});
