#!/usr/bin/env python3
"""Pass/fail rules for a SIL run (generic: written once, applied to every commit and scenario).

  python3 verdict.py <run_dir> [<run_dir> ...]      -> prints one line per run, writes <run_dir>/verdict.json

Rules (each is skipped when the scenario does not exercise it):
  boot      the stack reaches IDLE and nothing e-stops before START
  hold      while held at the pelvis in STAND, joint velocity RMS stays below 0.5 rad/s (no oscillation or vibration)
  stand     after the support is released in STAND: no fall, tilt stays below 20 deg
  walk      in WALK: no fall and no e-stop; with a forward command, mean forward speed >= 50% of the command
  stop      after STOP returns to STAND: no fall within 3 s
  reentry   on the second WALK entry: peak joint torque in the first 0.5 s <= max(1.5x, +10 Nm) of the first entry's
  gains     the gain each motor executes is within 5% of the gain the controller commanded (5th-95th percentile)
A run passes when every rule it exercises passes.

MIT License, Copyright (c) 2026 Abhinav Roy
"""
import json, os, sys
import numpy as np


def verdict(run):
    S = json.load(open(os.path.join(run, 'summary.json')))
    T = json.load(open(os.path.join(run, 'truth_10hz.json')))
    E = [json.loads(l) for l in open(os.path.join(run, 'events.jsonl'))]
    Z = np.load(os.path.join(run, 'truth.npz'))
    t, qd, tau, base, sup = Z['t'], Z['qd'], Z['tau'], Z['base'], Z['support']
    el2t = (lambda el: np.interp(el, [s['el'] for s in T], [s['t'] for s in T])) if T else None
    fsm = lambda el: next((s['fsm'] for s in reversed(T) if s['el'] <= el), None)
    cmds = [e for e in E if e['kind'] == 'command']; t0 = next((e['wall'] for e in E if e['kind'] == 'booted'), None)
    cmd_el = [(e['wall'] - t0, e['data']) for e in cmds] if t0 else []
    vel_ev = [(e['wall'] - t0, e['data']) for e in E if e['kind'] == 'vel'] if t0 else []
    R = {}
    states_before_start = [s['fsm'] for s in T if not cmd_el or s['el'] < cmd_el[0][0]]
    R['boot'] = bool(S['booted']) and 'ESTOP' not in states_before_start and 'IDLE' in (states_before_start or ['?'])
    fallen = lambda s: s['z'] < 0.45 or s['tilt_deg'] > 45
    if T and el2t:
        # hold: STAND while support == 1, after the 2 s position ramp
        st = [s for s in T if s['fsm'] == 'STAND' and s['support'] > 0.99]
        if len(st) > 15:
            a, b = el2t(st[0]['el'] + 2.5), el2t(st[-1]['el'])
            w = (t >= a) & (t <= b)
            if w.sum() > 20:
                R['hold'] = float(np.sqrt((qd[w] ** 2).mean())) < 0.5
                R['_hold_qd_rms'] = round(float(np.sqrt((qd[w] ** 2).mean())), 3)
        rel = [s for s in T if s['fsm'] == 'STAND' and s['support'] < 0.05 and not any(c[1] == 'WALK' and c[0] <= s['el'] for c in cmd_el)]
        if rel:
            R['stand'] = not any(fallen(s) for s in rel) and max(s['tilt_deg'] for s in rel) < 20
        walk = [s for s in T if s['fsm'] == 'WALK' and s['support'] < 0.05]
        walks = [c[0] for c in cmd_el if c[1] == 'WALK']
        if walks:
            R['walk'] = bool(walk) and not any(fallen(s) for s in walk) and 'ESTOP' not in [s['fsm'] for s in T if s['el'] >= walks[0] and s['el'] <= (next((c[0] for c in cmd_el if c[1] == 'STOP'), 1e9))]
            fwd = [(a, v) for a, v in vel_ev if v > 0]
            if fwd and R['walk']:
                a = fwd[0][0] + 1.0; b = next((x for x, v in vel_ev if x > fwd[0][0] and v == 0), None) or next((c[0] for c in cmd_el if c[1] == 'STOP' and c[0] > a), None)
                seg = [s for s in T if a <= s['el'] <= (b or 1e9) and s['fsm'] == 'WALK']
                if len(seg) > 10:
                    vx = (seg[-1]['x'] - seg[0]['x']) / (seg[-1]['el'] - seg[0]['el'])
                    R['_vx'] = round(vx, 3); R['_vx_cmd'] = fwd[0][1]
                    R['walk'] = vx >= 0.5 * fwd[0][1]
        stops = [c[0] for c in cmd_el if c[1] == 'STOP']
        if stops and walks:
            after = [s for s in T if stops[0] <= s['el'] <= stops[0] + 3.0]
            R['stop'] = bool(after) and not any(fallen(s) for s in after)
        if len(walks) >= 2:
            pk = []
            for wv in walks[:2]:
                a, b = el2t(wv), el2t(wv + 0.5); w = (t >= a) & (t <= b)
                pk.append(float(np.abs(tau[w]).max()) if w.any() else np.nan)
            R['_entry_tau_peaks'] = [round(x, 1) for x in pk]
            R['reentry'] = bool(pk[1] <= max(1.5 * pk[0], pk[0] + 10.0))   # 10 Nm floor: small peaks are noise
    # gains: every motor executes the gain the controller commanded (ankle motors: mean of the two joint gains)
    ratios = []
    for s in T:
        ck, ek = s.get('cmd_kp') or {}, s.get('kp_exec') or {}
        for m, kexe in ek.items():
            if kexe <= 0:
                continue
            if m.endswith(('_top', '_bottom')):
                a, b = ck.get(m[0] + '_foot_pitch'), ck.get(m[0] + '_foot_roll')
                kc = 0.5 * (a + b) if a and b else None
            else:
                kc = ck.get(m)
            if kc and kc > 0:
                ratios.append(kexe / kc)
    unified = any('unified:=true' in x for x in S.get('launch_args', []))   # unified_node ignores /joint_commands
    if len(ratios) > 20 and not unified:
        r = np.array(ratios); R['_kp_exec_over_cmd'] = [round(float(np.percentile(r, 5)), 2), round(float(np.percentile(r, 95)), 2)]
        R['gains'] = bool(np.all(np.abs(np.percentile(r, [5, 95]) - 1) < 0.05))
    rules = {k: v for k, v in R.items() if not k.startswith('_')}
    R['pass'] = all(rules.values())
    R['failed'] = [k for k, v in rules.items() if not v]
    json.dump(R, open(os.path.join(run, 'verdict.json'), 'w'), indent=1)
    return R


if __name__ == '__main__':
    for r in sys.argv[1:]:
        try:
            v = verdict(r)
            print(f"{os.path.basename(r.rstrip('/')):28s} {'PASS' if v['pass'] else 'FAIL'}  failed={v['failed']}  "
                  + ' '.join(f'{k}={v[k]}' for k in v if k.startswith('_')))
        except Exception as e:
            print(f'{r}: no verdict ({e})')
