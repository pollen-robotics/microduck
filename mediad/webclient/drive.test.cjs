const assert = require("node:assert/strict");
const { readFileSync } = require("node:fs");
const { test } = require("node:test");
const vm = require("node:vm");

function emit(target, type, fields = {}) {
  const event = new Event(type);
  Object.assign(event, fields);
  target.dispatchEvent(event);
}

class Pad extends EventTarget {
  style = {};
  captures = new Set();
  getBoundingClientRect() { return { left: 0, top: 0, width: 100, height: 100 }; }
  setPointerCapture(id) { this.captures.add(id); }
  hasPointerCapture(id) { return this.captures.has(id); }
  releasePointerCapture(id) {
    this.captures.delete(id);
    emit(this, "lostpointercapture", { pointerId: id });
  }
}

function drive() {
  const window = new EventTarget();
  const elements = Object.fromEntries(
    ["pad-move", "knob-move", "pad-turn", "knob-turn"].map(id => [id, new Pad()]),
  );
  const sent = [], timers = [];
  const page = readFileSync(`${__dirname}/index.html`, "utf8");
  // Run the shipped driving code; only the DOM, clock and outgoing transport are replaced.
  const start = page.indexOf("const twist = { vx: 0, vy: 0, vyaw: 0 };");
  const end = page.indexOf('const video = $("video");', start);
  assert.ok(start >= 0 && end > start);
  vm.runInNewContext(page.slice(start, end), {
    MAX_LINEAR: 0.3, MAX_ANGULAR: 1.5, INTENT_HZ: 10,
    $: id => elements[id],
    open: () => true,
    log: () => {},
    tell: (method, params) => sent.push({ method, params: { ...params } }),
    setInterval: fn => timers.push(fn),
    addEventListener: window.addEventListener.bind(window),
  });
  return { window, elements, sent, tick: () => timers.forEach(fn => fn()) };
}

const zero = { method: "robot.move", params: { vx: 0, vy: 0, vyaw: 0 } };
const press = (pad, pointerId = 1) =>
  emit(pad, "pointerdown", { pointerId, clientX: 100, clientY: 0 });

test("losing focus stops both joysticks and held keys until a new press", () => {
  const { window, elements, sent, tick } = drive();
  const move = elements["pad-move"], turn = elements["pad-turn"];
  emit(window, "keydown", { key: "w" });
  press(move);
  press(turn, 2);
  tick();
  assert.deepEqual(sent.at(-1).params, { vx: 0.3, vy: -0.3, vyaw: -1.5 });

  // Losing focus while dragging must stop the heartbeat, even without a pointerup.
  emit(window, "blur");
  assert.deepEqual(sent.at(-1), zero);
  const stopped = sent.length;
  emit(move, "pointermove", { pointerId: 1, clientX: 0, clientY: 100 });
  emit(turn, "pointermove", { pointerId: 2, clientX: 0, clientY: 100 });
  tick();
  assert.equal(sent.length, stopped, "a stale drag must not restart movement");
  for (const id of ["knob-move", "knob-turn"]) {
    assert.deepEqual(elements[id].style, { left: "50%", top: "50%" });
  }
  press(move);
  tick();
  assert.equal(sent.at(-1).params.vx, 0.3, "a fresh press still drives");
});

test("losing pointer capture stops the joystick heartbeat", () => {
  const { elements, sent, tick } = drive();
  const pad = elements["pad-move"];
  press(pad);
  tick();
  assert.equal(sent.at(-1).params.vx, 0.3);
  pad.releasePointerCapture(1);
  assert.deepEqual(sent.at(-1), zero);
  const stopped = sent.length;
  tick();
  assert.equal(sent.length, stopped);
});

for (const type of ["pointerup", "pointercancel"]) {
  test(`${type} stops the joystick heartbeat`, () => {
    const { elements, sent, tick } = drive();
    press(elements["pad-turn"]);
    tick();
    assert.equal(sent.at(-1).params.vyaw, -1.5);
    emit(elements["pad-turn"], type, { pointerId: 1 });
    assert.deepEqual(sent.at(-1), zero);
    const stopped = sent.length;
    tick();
    assert.equal(sent.length, stopped);
  });
}

test("another pointer cannot take over or release a held joystick", () => {
  const { elements, sent, tick } = drive();
  const pad = elements["pad-move"];
  press(pad);
  emit(pad, "pointerdown", { pointerId: 2, clientX: 0, clientY: 100 });
  emit(pad, "pointerup", { pointerId: 2 });
  tick();
  assert.deepEqual(sent.at(-1).params, { vx: 0.3, vy: -0.3, vyaw: 0 });
  emit(pad, "pointerup", { pointerId: 1 });
  assert.deepEqual(sent.at(-1), zero);
});
