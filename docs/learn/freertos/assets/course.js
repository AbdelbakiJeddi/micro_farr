/* FreeRTOS, the Hard Way: shared navigation, timeline engine and verified simulations. */

const PAGES = [
  ['index', 'index.html', 'Index'],
  ['why', '01-why-rtos.html', '1 · Why an RTOS'],
  ['tasks', '02-tasks.html', '2 · Tasks'],
  ['sched', '03-scheduler.html', '3 · Scheduler'],
  ['timing', '04-timing.html', '4 · Timing'],
  ['queues', '05-queues.html', '5 · Queues'],
  ['mailbox', '06-mailbox.html', '6 · Mailbox'],
  ['events', '07-events.html', '7 · Events'],
  ['shared', '08-shared-data.html', '8 · Shared data'],
  ['dual', '09-dual-core.html', '9 · Dual core'],
  ['design', '10-design.html', '10 · Design'],
  ['synth', '11-synthesis.html', '11 · Synthesis'],
];
function nav(id) {
  const i = PAGES.findIndex(p => p[0] === id);
  const n = document.getElementById('nav');
  if (n) n.innerHTML = PAGES.map(p => `<a href="${p[1]}" class="${p[0] === id ? 'on' : ''}">${p[2]}</a>`).join('');
  const pn = document.getElementById('pn');
  if (pn) pn.innerHTML = (i > 0 ? `<a href="${PAGES[i - 1][1]}">← ${PAGES[i - 1][2]}</a>` : '<span></span>') +
    (i < PAGES.length - 1 ? `<a href="${PAGES[i + 1][1]}">${PAGES[i + 1][2]} →</a>` : '<span></span>');
}

/*SIM-START*/
const cell = (k, t) => ({ k, t });
const blank = H => Array.from({ length: H }, () => cell('off'));
const notesOf = H => Array.from({ length: H }, () => []);

const SIM = {};

/* 1. super-loop vs RTOS: sensor 2 ms every 10 ms, control 1 ms after it, OLED 25 ms at 3 and 33 */
SIM.why = ({ sched, oled }) => {
  const H = 60, P = 10, COST = 25, AT = [3, 33];
  const run = Array(H).fill('idle'), nt = notesOf(H);
  if (sched === 'super') {
    let t = 0, next = AT[0];
    while (t < H) {
      const start = t;
      for (let i = 0; i < 2 && t < H; i++) run[t++] = 'sensor';
      if (t < H) run[t++] = 'control';
      if (oled && t >= next && t < H) {
        nt[t].push('loop enters oled_draw(): nothing else runs until it returns');
        for (let i = 0; i < COST && t < H; i++) run[t++] = 'oled';
        next += 30;
      }
      t = Math.max(Math.ceil(t / P) * P, start + P);
    }
    for (let k = 1; k < H / P; k++) if (run[k * P] === 'oled') nt[k * P].push(`sample ${k * P} ms is due, but the loop is still inside oled_draw()`);
  } else {
    let s = 0, c = 0; const q = [];
    for (let t = 0; t < H; t++) {
      if (t % P === 0) { s = 2; nt[t].push(`sample ${t} ms due: sensor task READY`); }
      if (oled && AT.includes(t)) { q.push(COST); nt[t].push('OLED redraw requested'); }
      const pick = c > 0 ? 'control' : s > 0 ? 'sensor' : q.length ? 'oled' : 'idle';
      if (t > 0 && run[t - 1] === 'oled' && pick !== 'oled' && q.length) nt[t].push(`${pick} preempts the OLED task`);
      run[t] = pick;
      if (pick === 'sensor' && --s === 0) c = 1;
      else if (pick === 'control') c--;
      else if (pick === 'oled' && --q[0] === 0) { q.shift(); nt[t].push('OLED frame finished'); }
    }
  }
  const DO = sched === 'super'
    ? { sensor: 'loop: read_sensors()', control: 'loop: control()', oled: 'loop: oled_draw()', idle: 'loop: waiting for the next 10 ms' }
    : { sensor: 'running: sensor task (prio 2)', control: 'running: control task (prio 3)', oled: 'running: OLED task (prio 1)', idle: 'every task blocked: idle task runs' };
  const rows = [['sensor', 'sensor'], ['control', 'control'], ['oled', 'OLED']]
    .map(([r, label]) => ({ label, cells: run.map(x => cell(x === r ? 'run' : 'off')) }));
  const ok = Array.from({ length: H / P }, (_, k) => run.slice(k * P, (k + 1) * P).includes('control'));
  const marks = ok.map((o, k) => ({ at: (k + 1) * P - 1, x: k * P + P / 2, k: o ? 'good' : 'bad', t: o ? '✓ on time' : '✗ missed' }));
  const miss = ok.filter(o => !o).length;
  return { H, unit: 'ms', rows, marks, markLabel: 'deadline',
    cap: run.map((x, t) => [DO[x], ...nt[t]].join(' · ')),
    value: miss, summary: `Control deadlines: <span class="ok">${6 - miss} on time</span>, <span class="ko">${miss} missed</span>.` };
};

/* 3. starvation: control (prio 3) blocks on the sample, or busy-polls a flag */
SIM.starve = ({ ctrl }) => {
  const H = 40, P = 10;
  const run = Array(H).fill('idle'), nt = notesOf(H), sensorReady = Array(H).fill(false);
  let s = 0, c = 0, samples = 0;
  for (let t = 0; t < H; t++) {
    if (t % P === 0) { s = 2; nt[t].push(`sample ${t} ms due: sensor task READY`); }
    sensorReady[t] = s > 0;
    if (ctrl === 'busy') {
      run[t] = 'control';
      if (t % P === 0) nt[t].push('sensor READY, but control (prio 3) never blocks: sensor and idle starve');
      continue;
    }
    const pick = c > 0 ? 'control' : s > 0 ? 'sensor' : 'idle';
    run[t] = pick;
    if (pick === 'sensor' && --s === 0) { c = 1; samples++; } else if (pick === 'control') c--;
  }
  const DO = { control: ctrl === 'busy' ? 'control: while (!new_sample) {} — polling' : 'control: runs the control law, then blocks', sensor: 'sensor: reads and publishes the sample', idle: 'idle task (prio 0) runs and feeds the watchdog' };
  const rows = [
    { label: 'control (3)', cells: run.map(x => cell(x === 'control' ? (ctrl === 'busy' ? 'bad' : 'run') : 'off')) },
    { label: 'sensor (2)', cells: run.map((x, t) => cell(x === 'sensor' ? 'run' : (ctrl === 'busy' && sensorReady[t]) ? 'wait' : 'off')) },
    { label: 'idle (0)', cells: run.map(x => cell(x === 'idle' ? 'run' : ctrl === 'busy' ? 'wait' : 'off')) },
  ];
  return { H, unit: 'ms', rows,
    cap: run.map((x, t) => [DO[x], ...nt[t]].join(' · ')),
    value: samples,
    summary: ctrl === 'busy'
      ? `Samples read: <span class="ko">0</span>. Idle never runs: the task watchdog fires after 5 s.`
      : `Samples read: <span class="ok">${samples}</span>. The CPU is idle ${run.filter(x => x === 'idle').length} of ${H} ms.` };
};

/* 4. vTaskDelay vs vTaskDelayUntil with work of 2–5 ms per pass */
SIM.timing = ({ api }) => {
  const H = 60, P = 10, W = [2, 4, 3, 5, 2, 4, 3];
  const run = Array(H).fill(false), nt = notesOf(H), starts = [];
  let t = 0, i = 0;
  while (t < H) {
    starts.push(t);
    const w = W[i % W.length], end = t + w, next = api === 'delay' ? end + P : (i + 1) * P;
    nt[t].push(`pass ${i + 1} starts at ${t} ms (${w} ms of work)`);
    for (let j = t; j < end && j < H; j++) run[j] = true;
    if (end < H) nt[end].push(api === 'delay' ? `vTaskDelay(10): sleeps until ${end} + 10 = ${end + P} ms` : `vTaskDelayUntil: sleeps until ${next} ms`);
    t = next; i++;
  }
  const rows = [{ label: 'task', cells: run.map(r => cell(r ? 'run' : 'wait')) }];
  const marks = starts.map(s => ({ at: s, x: s + 0.5, k: s % P === 0 ? 'good' : 'bad', t: String(s) }));
  return { H, unit: 'ms', rows, marks, markLabel: 'starts at',
    cap: run.map((r, t) => [r ? 'running: loop body' : 'blocked in the delay', ...nt[t]].join(' · ')),
    value: starts.length,
    summary: `${starts.length} passes in 60 ms; starts at ${starts.join(', ')} ms. ` +
      (api === 'delay' ? '<span class="ko">The period drifts with the work time.</span>' : '<span class="ok">Every pass starts on the 10 ms grid.</span>') };
};

/* 5. queue of 4: producer tries every 2 ms, consumer takes one at 5, 15, 25, 35 ms */
SIM.queue = ({ wait }) => {
  const H = 40, LEN = 4;
  const prod = blank(H), cons = blank(H), depth = blank(H), nt = notesOf(H);
  let d = 0, blocked = false, lost = 0, blockedMs = 0;
  for (let t = 0; t < H; t++) {
    if (t % 10 === 5 && d > 0) { d--; cons[t] = cell('run', 'rx'); nt[t].push('consumer: xQueueReceive copies out the oldest item'); }
    if (blocked) {
      if (d < LEN) { d++; blocked = false; prod[t] = cell('good', 'tx'); nt[t].push('a slot freed: the blocked xQueueSend completes'); }
      else { blockedMs++; prod[t] = cell('bad', '…'); nt[t].push('producer BLOCKED in xQueueSend: queue full'); }
    } else if (t % 2 === 0) {
      if (d < LEN) { d++; prod[t] = cell('run', 'tx'); nt[t].push('producer: xQueueSend copies the item in'); }
      else if (wait === 'zero') { lost++; prod[t] = cell('bad', '✗'); nt[t].push('queue full, timeout 0: xQueueSend returns errQUEUE_FULL; the item is lost'); }
      else { blocked = true; blockedMs++; prod[t] = cell('bad', '…'); nt[t].push('queue full: the producer blocks until a slot frees'); }
    }
    depth[t] = cell(d === LEN ? 'bad' : 'off', String(d));
  }
  const rows = [{ label: 'producer', cells: prod }, { label: 'consumer', cells: cons }, { label: 'items', cells: depth }];
  return { H, unit: 'ms', rows,
    cap: nt.map((n, t) => n.length ? n.join(' · ') : (prod[t].k === 'off' && cons[t].k === 'off' ? `queue holds ${depth[t].t} item(s)` : '')),
    value: lost,
    summary: wait === 'zero'
      ? `<span class="ko">${lost} items lost</span>; the producer never waits.`
      : `<span class="ok">0 items lost</span>; the producer spent <span class="ko">${blockedMs} ms blocked</span> (back-pressure).` };
};

/* 7. stale event bit: move 1 ends after the app stopped waiting */
SIM.events = ({ clear }) => {
  const H = 32;
  const app = blank(H), mot = blank(H), bit = blank(H), nt = notesOf(H);
  let b = false, ret = null;
  for (let t = 0; t < H; t++) {
    if (t >= 1 && t <= 13) mot[t] = cell('run', t === 1 ? 'mv1' : '');
    if (t === 14) { mot[t] = cell('good', 'end'); b = true; nt[t].push('move 1 ends: motion sets MOVE_DONE, but nobody is waiting'); }
    if (t >= 17 && t <= 28) mot[t] = cell('run', t === 17 ? 'mv2' : '');
    if (t === 29) { mot[t] = cell('good', 'end'); b = true; nt[t].push('move 2 ends: motion sets MOVE_DONE'); }

    if (t === 0) { app[t] = cell('run', 'send'); nt[t].push('app: motion_straight() queues move 1, then waits for MOVE_DONE'); }
    if (t >= 1 && t <= 7) app[t] = cell('wait');
    if (t === 8) { app[t] = cell('bad', 't/o'); nt[t].push('app stops waiting (timeout, or DEC pressed): MOVE_DONE will arrive later, unconsumed'); }
    if (t === 16) {
      app[t] = cell('run', 'send');
      if (clear) { b = false; nt[t].push('app clears MOVE_DONE | MOVE_FAULT, then queues move 2 and waits'); }
      else nt[t].push('app queues move 2 and waits, without clearing: the old MOVE_DONE is still set');
    }
    bit[t] = cell(b ? 'good' : 'off', b ? '1' : '0');
    if (t >= 17 && ret === null) {
      if (b) {
        ret = t; b = false;
        app[t] = cell(t === 29 ? 'good' : 'bad', 'ret');
        nt[t].push(t === 29 ? 'wait returns: move 2 is really done; the bit is cleared on exit'
          : 'wait returns AT ONCE on the stale bit from move 1: the app believes move 2 is done while the robot is still driving');
      } else app[t] = cell('wait');
    }
    if (!clear && t === 18) { app[t] = cell('bad', 'next'); nt[t].push('app queues the next move: motion rejects it, "move ignored: motion busy"'); }
  }
  const rows = [{ label: 'app', cells: app }, { label: 'motion', cells: mot }, { label: 'MOVE_DONE', cells: bit }];
  return { H, unit: 'ms', rows,
    cap: nt.map((n, t) => n.length ? n.join(' · ') : (mot[t].k === 'run' ? 'motion: driving' : app[t].k === 'wait' ? 'app: blocked in xEventGroupWaitBits' : '…')),
    value: ret,
    summary: ret === 29 ? '<span class="ok">The wait returned at 29 ms, when move 2 really ended.</span>'
      : `<span class="ko">The wait returned at ${ret} ms, 12 ms before move 2 ended.</span>` };
};

/* 8a. lost update: two tasks do count++ */
SIM.race = ({ lock }) => {
  const S = lock === 'none'
    ? [
        ['run:read 0', 'off', 0, 'A reads count (0) into a register'],
        ['wait', 'run:read 0', 0, 'B (higher priority) wakes and preempts A between its read and its write; B reads 0'],
        ['wait', 'run:+1', 0, 'B adds 1 in its own register'],
        ['wait', 'run:write 1', 1, 'B writes 1, then blocks again'],
        ['run:+1', 'off', 1, 'A resumes with its stale register (0) and adds 1'],
        ['run:write 1', 'off', 1, "A writes 1: B's update is lost. Two increments, count = 1"],
      ]
    : [
        ['run:take', 'off', 0, 'A takes the mutex'],
        ['run:read 0', 'off', 0, 'A reads count (0)'],
        ['run:+1', 'bad:take', 0, 'B wakes, preempts A, tries to take the mutex: BLOCKED. A runs on'],
        ['run:write 1', 'bad:wait', 1, 'A writes 1'],
        ['run:give', 'bad:wait', 1, 'A gives the mutex: B becomes ready and preempts A'],
        ['off', 'run:take', 1, 'B takes the mutex'],
        ['off', 'run:read 1', 1, 'B reads count (1)'],
        ['off', 'run:+1', 1, 'B adds 1'],
        ['off', 'run:write 2', 2, 'B writes 2: both increments counted'],
        ['off', 'run:give', 2, 'B gives the mutex'],
      ];
  const mk = s => { const [k, t] = s.split(':'); return cell(k, t); };
  const H = S.length, final = S[H - 1][2];
  const rows = [
    { label: 'task A (1)', cells: S.map(s => mk(s[0])) },
    { label: 'task B (2)', cells: S.map(s => mk(s[1])) },
    { label: 'count', cells: S.map(s => cell('off', String(s[2]))) },
  ];
  return { H, unit: 'step', rows, cap: S.map(s => s[3]), value: final,
    summary: final === 2 ? '<span class="ok">count = 2: correct.</span>' : '<span class="ko">count = 1: one increment lost.</span>' };
};

/* 8b. priority inversion: Low holds the lock 5 ms, High needs it at 2 ms, Medium runs 12 ms from 4 ms */
SIM.inversion = ({ inherit }) => {
  const H = 26;
  let lWork = 5, lHas = true, hState = 'sleep', hWork = 3, mWork = 12, mReady = false, hBlocked = 0, hDone = null;
  const rows = { H: blank(H), M: blank(H), L: blank(H), P: blank(H) }, nt = notesOf(H);
  for (let t = 0; t < H; t++) {
    if (t === 2) hState = 'ready';
    if (t === 4) { mReady = true; nt[t].push('Medium wakes: CPU-bound for 12 ms, never touches the lock'); }
    if (hState === 'ready' && lHas) { hState = 'blocked'; nt[t].push('High wakes and tries to take the mutex held by Low: BLOCKED'); }
    const lPrio = (hState === 'blocked' && inherit) ? 3 : 1;
    const cand = [];
    if (hState === 'ready' && hWork > 0) cand.push(['H', 3]);
    if (mReady && mWork > 0) cand.push(['M', 2]);
    if (lWork > 0) cand.push(['L', lPrio]);
    cand.sort((a, b) => b[1] - a[1]);
    const pick = cand.length ? cand[0][0] : null;
    if (hState === 'blocked') { rows.H[t] = cell('bad', ''); hBlocked++; }
    if (mReady && mWork > 0 && pick !== 'M') rows.M[t] = cell('wait');
    if (lWork > 0 && pick !== 'L') rows.L[t] = cell('wait');
    rows.P[t] = cell(lPrio === 3 ? 'good' : 'off', String(lPrio));
    if (pick === 'H') { rows.H[t] = cell('run'); if (--hWork === 0) { hDone = t + 1; nt[t].push('High finishes'); } }
    if (pick === 'M') { rows.M[t] = cell('run'); if (--mWork === 0) nt[t].push('Medium finishes'); }
    if (pick === 'L') {
      rows.L[t] = cell('run', lHas ? '🔒' : '');
      if (lPrio === 3 && t === 2) nt[t].push('priority inheritance: Low now runs at High\'s priority (3)');
      if (--lWork === 0) { lHas = false; nt[t].push('Low gives the mutex'); if (hState === 'blocked') { hState = 'ready'; nt[t].push('High unblocks'); } }
    }
    if (!nt[t].length) nt[t].push(pick === 'M' && hState === 'blocked' ? 'Medium runs while High waits for Low: priority inversion' : pick ? `${{ H: 'High', M: 'Medium', L: 'Low' }[pick]} runs` : 'idle');
  }
  return { H, unit: 'ms', rows: [
      { label: 'High (3)', cells: rows.H }, { label: 'Medium (2)', cells: rows.M },
      { label: 'Low (1)', cells: rows.L }, { label: "Low's prio", cells: rows.P }],
    cap: nt.map(n => n.join(' · ')), value: hBlocked,
    summary: `High was blocked <span class="${hBlocked > 5 ? 'ko' : 'ok'}">${hBlocked} ms</span> and finished at ${hDone} ms.` };
};
/*SIM-END*/

const COLORS = { run: '#343a40', wait: '#e4e1da', bad: '#e8590c', good: '#2f9e44' };
const TXT = { run: '#fff', bad: '#fff', good: '#fff', wait: '#111', off: '#111' };

function mount(id, cfg) {
  const root = document.getElementById(id);
  const st = Object.assign({}, cfg.init, { cur: 0, guess: null, timer: null });
  let data = cfg.sim(st);
  root.className = 'tl';
  const pr = cfg.predict;
  root.innerHTML =
    (pr ? `<div class="predict"><b>Predict first.</b> ${pr.q}<div class="choices">${pr.choices.map(([v, l]) => `<button data-g="${v}">${l}</button>`).join('')}</div><div class="verdict">Pick an answer, then step to the end with ${pr.whenText}.</div></div>` : '') +
    `<div class="ctl">${cfg.toggles.map(tg => `<span class="lbl">${tg.label}</span><span class="seg" role="group" aria-label="${tg.label}">${tg.options.map(([v, l]) => `<button data-key="${tg.key}" data-v="${v}" class="${st[tg.key] === v ? 'sel' : ''}">${l}</button>`).join('')}</span>`).join('')}</div>` +
    `<div class="ctl"><button data-a="reset" aria-label="Back to the start">⏮</button><button data-a="prev" aria-label="One step back">◀</button><button data-a="next" aria-label="One step forward">▶</button><button data-a="play">Play</button><button data-a="end" aria-label="Jump to the end">⏭</button><span class="lbl clock"></span></div>` +
    `<div class="svgwrap"><svg role="img" aria-label="${cfg.label}"></svg></div><div class="cap" aria-live="polite"></div><div class="res"></div>`;
  const svg = root.querySelector('svg'), cap = root.querySelector('.cap'), res = root.querySelector('.res');
  const clock = root.querySelector('.clock'), verdict = root.querySelector('.verdict');

  function render() {
    const { H, rows, marks = [], unit } = data;
    const cw = H <= 12 ? 62 : H <= 32 ? 24 : H <= 40 ? 15 : 10;
    const LX = 96, RH = 22, G = 8, top = 6;
    const W = LX + H * cw + 14, axisY = top + rows.length * (RH + G) + 14, markY = axisY + 20;
    const HT = marks.length ? markY + 10 : axisY + 6;
    let s = '';
    rows.forEach((r, i) => {
      const y = top + i * (RH + G);
      s += `<text x="4" y="${y + 15}" font-size="11.5" fill="#111">${r.label}</text>`;
      r.cells.forEach((c, t) => {
        const x = LX + t * cw, op = t >= st.cur ? 0.14 : 1;
        if (c.k !== 'off') s += `<rect x="${x + 0.5}" y="${y}" width="${cw - 1}" height="${RH}" rx="2" fill="${COLORS[c.k]}" opacity="${op}"/>`;
        if (c.t && cw >= 15) s += `<text x="${x + cw / 2}" y="${y + 15}" text-anchor="middle" font-size="${cw >= 40 ? 11 : 10}" fill="${TXT[c.k]}" opacity="${t >= st.cur ? 0.25 : 1}">${c.t}</text>`;
      });
    });
    const step = unit === 'step' ? 1 : H <= 32 ? 4 : 10;
    for (let t = 0; t <= H; t += step) {
      const x = LX + t * cw;
      s += `<line x1="${x}" x2="${x}" y1="${top - 2}" y2="${axisY - 10}" stroke="#e4e1da" stroke-dasharray="2 3"/>`;
      if (unit !== 'step') s += `<text x="${x}" y="${axisY + 2}" text-anchor="middle" font-size="10" fill="#4a5058">${t}</text>`;
    }
    s += `<text x="4" y="${axisY + 2}" font-size="10" fill="#4a5058">${unit === 'step' ? '' : 'ms'}</text>`;
    if (marks.length) {
      s += `<text x="4" y="${markY + 2}" font-size="10" fill="#4a5058">${data.markLabel}</text>`;
      marks.forEach(m => { if (st.cur > m.at) s += `<text x="${LX + m.x * cw}" y="${markY + 2}" text-anchor="middle" font-size="10.5" font-weight="700" fill="${COLORS[m.k]}">${m.t}</text>`; });
    }
    const cx = LX + st.cur * cw;
    s += `<line x1="${cx}" x2="${cx}" y1="${top - 4}" y2="${axisY - 8}" stroke="#e8590c" stroke-width="2"/>`;
    svg.setAttribute('viewBox', `0 0 ${W} ${HT}`);
    svg.style.width = '100%'; svg.style.minWidth = Math.round(W * 0.72) + 'px';
    svg.innerHTML = s;

    clock.textContent = unit === 'step' ? `step ${st.cur} / ${H}` : `t = ${st.cur} ms`;
    if (st.cur === 0) cap.textContent = 'Press ▶ to advance one step, or Play.';
    else { const t = st.cur - 1; cap.textContent = (unit === 'step' ? `step ${t + 1} · ` : `${t}–${t + 1} ms · `) + data.cap[t]; }
    res.innerHTML = st.cur === H ? data.summary : '';
    checkGuess();
  }
  function checkGuess() {
    if (!pr || st.guess === null || st.cur < data.H || !pr.when(st)) return;
    const right = String(st.guess) === String(data.value);
    verdict.className = 'verdict ' + (right ? 'good' : 'bad');
    verdict.innerHTML = (right ? 'Right. ' : `You said ${pr.label(st.guess)}; the answer is ${pr.label(data.value)}. `) + pr.why;
  }
  function stop() { clearInterval(st.timer); st.timer = null; root.querySelector('[data-a="play"]').textContent = 'Play'; }
  function go(n) { st.cur = Math.max(0, Math.min(data.H, n)); if (st.cur === data.H) stop(); render(); }

  root.addEventListener('click', e => {
    const b = e.target.closest('button'); if (!b) return;
    if (b.dataset.key) {
      st[b.dataset.key] = b.dataset.v;
      root.querySelectorAll(`[data-key="${b.dataset.key}"]`).forEach(x => x.classList.toggle('sel', x === b));
      data = cfg.sim(st); go(st.cur);
    } else if (b.dataset.g !== undefined) {
      st.guess = b.dataset.g;
      root.querySelectorAll('[data-g]').forEach(x => x.classList.toggle('sel', x === b));
      verdict.className = 'verdict';
      verdict.textContent = `Your guess: ${pr.label(st.guess)}. Now step to the end with ${pr.whenText}.`;
      checkGuess();
    } else {
      const a = b.dataset.a;
      if (a === 'play') {
        if (st.timer) return stop();
        if (st.cur === data.H) st.cur = 0;
        b.textContent = 'Pause';
        st.timer = setInterval(() => go(st.cur + 1), data.unit === 'step' ? 700 : 110);
        return;
      }
      stop();
      go(a === 'reset' ? 0 : a === 'prev' ? st.cur - 1 : a === 'next' ? st.cur + 1 : data.H);
    }
  });
  render();
}

