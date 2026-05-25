#!/usr/bin/env python3
"""
Batch Esterel plan generator.

Runs esterel_plan_generator on multiple domain/problem pairs, optionally
in parallel.  Pairs can be specified via a JSON config file, via --pair
flags on the command line, or by scanning a directory tree.

Config file format (JSON):
  {
    "planner":   "timeout TIMEOUT popf DOMAIN PROBLEM",  // default planner
    "timeout":   60,                                     // seconds (replaces TIMEOUT)
    "data_path": "/tmp/esterel_batch",                   // scratch space
    "epsilon":   0.1,                                    // TIL margin
    "pairs": [
      {"name": "rovers",  "domain": "domain.pddl", "problem": "problem.pddl"},
      {"name": "amazon",  "domain": "amazon/domain.pddl", "problem": "amazon/problem.pddl",
       "planner": "timeout TIMEOUT optic-clp DOMAIN PROBLEM"}   // per-pair override
    ]
  }

CLI examples:
  # two pairs, default POPF
  python3 batch_esterel.py \\
      --pair domain1.pddl problem1.pddl \\
      --pair domain2.pddl problem2.pddl \\
      --planner "timeout 30 popf DOMAIN PROBLEM"

  # config file, save JSON results, 4 parallel workers
  python3 batch_esterel.py --config pairs.json --output-dir results/ --jobs 4

  # scan a directory for domain.pddl + problem.pddl pairs
  python3 batch_esterel.py --scan rosplan_planning_system/test/pddl/
"""

import os
import sys
import json
import time
import argparse
import traceback
import threading
from concurrent.futures import ThreadPoolExecutor, as_completed
from dataclasses import dataclass, field
from typing import List, Optional, Dict, Any

import esterel_plan_generator as eg


# ---------------------------------------------------------------------------
# Data structures
# ---------------------------------------------------------------------------

@dataclass
class PairSpec:
    domain:   str
    problem:  str
    name:     str  = ''
    planner:  Optional[str] = None   # overrides global if set


@dataclass
class PairResult:
    spec:       PairSpec
    success:    bool
    elapsed_s:  float
    num_nodes:  int  = 0
    num_edges:  int  = 0
    error:      str  = ''
    plan_dict:  Optional[Dict] = None


# ---------------------------------------------------------------------------
# Domain cache
# ---------------------------------------------------------------------------

class _DomainCache:
    """Thread-safe cache: domain realpath → parsed operators dict.

    parse_domain() is O(domain-size) pure Python.  For a batch where N
    instances share one domain file, caching cuts that cost to 1 parse
    instead of N.
    """

    def __init__(self) -> None:
        self._cache: Dict[str, Dict] = {}
        self._lock  = threading.Lock()

    def get(self, domain_path: str) -> Dict:
        key = os.path.realpath(domain_path)
        if key in self._cache:          # fast path — no lock needed after first write
            return self._cache[key]
        with self._lock:                # slow path — parse once, then cache
            if key not in self._cache:
                self._cache[key] = eg.parse_domain(domain_path)
        return self._cache[key]

    @property
    def size(self) -> int:
        return len(self._cache)


# ---------------------------------------------------------------------------
# Config loading
# ---------------------------------------------------------------------------

def _load_config(path: str) -> Dict[str, Any]:
    with open(path) as fh:
        return json.load(fh)


def _pairs_from_config(cfg: Dict[str, Any]) -> List[PairSpec]:
    pairs = []
    for i, entry in enumerate(cfg.get('pairs', [])):
        name = entry.get('name', f'pair_{i}')
        pairs.append(PairSpec(
            domain=entry['domain'],
            problem=entry['problem'],
            name=name,
            planner=entry.get('planner'),
        ))
    return pairs


def _scan_directory(root: str) -> List[PairSpec]:
    """
    Walk *root* and collect every directory that contains both a
    domain.pddl and a problem.pddl (case-insensitive).
    """
    pairs: List[PairSpec] = []
    for dirpath, _, filenames in os.walk(root):
        lower = {f.lower(): f for f in filenames}
        domain_file  = lower.get('domain.pddl')
        problem_file = lower.get('problem.pddl')
        if domain_file and problem_file:
            rel = os.path.relpath(dirpath, root)
            pairs.append(PairSpec(
                domain=os.path.join(dirpath, domain_file),
                problem=os.path.join(dirpath, problem_file),
                name=rel.replace(os.sep, '/'),
            ))
    pairs.sort(key=lambda p: p.name)
    return pairs


# ---------------------------------------------------------------------------
# Single-pair runner
# ---------------------------------------------------------------------------

def _run_pair(spec: PairSpec,
              global_planner: str,
              data_root: str,
              epsilon: float,
              save_json: bool,
              domain_cache: _DomainCache,
              timeout: int = 60) -> PairResult:
    planner   = spec.planner or global_planner
    # Give each pair its own scratch sub-directory so parallel runs don't clash
    safe_name = spec.name.replace('/', '_').replace(' ', '_') or 'pair'
    data_path = os.path.join(data_root, safe_name)

    t0 = time.perf_counter()
    try:
        # Step 1: run external planner (subprocess — GIL released, truly parallel)
        solved, planner_output = eg.run_planner(
            domain_path=spec.domain,
            problem_path=spec.problem,
            data_path=data_path,
            planner_command=planner,
            timeout=timeout,
        )
        if not solved:
            return PairResult(spec=spec, success=False,
                              elapsed_s=time.perf_counter() - t0,
                              error='Planner found no solution')

        # Step 2: domain parse (cached — at most once per unique domain file)
        operators = domain_cache.get(spec.domain)

        # Step 3: problem TILs + graph build
        tils    = eg.parse_problem_tils(spec.problem)
        builder = eg.EsterelPlanBuilder(epsilon_time=epsilon)
        plan    = builder.build(planner_output, operators, tils)

        elapsed   = time.perf_counter() - t0
        plan_dict = eg.plan_to_dict(plan) if save_json else None
        return PairResult(
            spec=spec,
            success=True,
            elapsed_s=elapsed,
            num_nodes=len(plan.nodes),
            num_edges=len(plan.edges),
            plan_dict=plan_dict,
        )

    except Exception as exc:
        elapsed = time.perf_counter() - t0
        return PairResult(spec=spec, success=False, elapsed_s=elapsed,
                          error=f'{type(exc).__name__}: {exc}\n{traceback.format_exc()}')


# ---------------------------------------------------------------------------
# Output helpers
# ---------------------------------------------------------------------------

def _print_result(r: PairResult, verbose: bool) -> None:
    tag   = '  OK ' if r.success else ' FAIL'
    label = r.spec.name or f'{r.spec.domain} / {r.spec.problem}'
    base  = f'[{tag}] {label:<40s}  {r.elapsed_s:6.2f}s'
    if r.success:
        print(f'{base}  {r.num_nodes} nodes, {r.num_edges} edges')
    else:
        print(f'{base}  {r.error.splitlines()[0]}')
        if verbose and r.error:
            for line in r.error.splitlines()[1:]:
                print(f'          {line}')


def _print_progress(done: int, total: int, n_ok: int,
                    wall_start: float) -> None:
    elapsed = time.perf_counter() - wall_start
    rate    = done / elapsed if elapsed > 0 else 0
    eta_s   = (total - done) / rate if rate > 0 else 0

    def _fmt_time(s: float) -> str:
        s = int(s)
        h, rem = divmod(s, 3600)
        m, sec = divmod(rem, 60)
        return f'{h}h{m:02d}m{sec:02d}s' if h else f'{m}m{sec:02d}s'

    bar_width = 30
    filled    = int(bar_width * done / total) if total else 0
    bar       = '█' * filled + '░' * (bar_width - filled)
    pct       = 100 * done / total if total else 0

    print(
        f'\r  [{bar}] {pct:5.1f}%  {done}/{total}'
        f'  ok={n_ok}  fail={done - n_ok}'
        f'  elapsed={_fmt_time(elapsed)}'
        f'  eta={_fmt_time(eta_s)}   ',
        end='', flush=True,
    )


def _save_json_result(r: PairResult, output_dir: str) -> None:
    os.makedirs(output_dir, exist_ok=True)
    safe = (r.spec.name or 'pair').replace('/', '_').replace(' ', '_')
    path = os.path.join(output_dir, f'{safe}.json')
    payload = {
        'name':       r.spec.name,
        'domain':     r.spec.domain,
        'problem':    r.spec.problem,
        'planner':    r.spec.planner,
        'success':    r.success,
        'elapsed_s':  r.elapsed_s,
        'num_nodes':  r.num_nodes,
        'num_edges':  r.num_edges,
        'error':      r.error or None,
        'plan':       r.plan_dict,
    }
    with open(path, 'w') as fh:
        json.dump(payload, fh, indent=2)


def _print_summary(results: List[PairResult]) -> None:
    ok   = [r for r in results if r.success]
    fail = [r for r in results if not r.success]
    total_t = sum(r.elapsed_s for r in results)
    print()
    print('─' * 60)
    print(f'Summary: {len(ok)}/{len(results)} succeeded  '
          f'(wall time: {total_t:.1f}s)')
    if fail:
        print(f'Failed pairs:')
        for r in fail:
            label = r.spec.name or r.spec.domain
            print(f'  {label}: {r.error.splitlines()[0]}')


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def _build_argparser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(
        description='Batch Esterel plan generation over multiple domain/problem pairs.',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )

    # ---- input sources ----
    ap.add_argument('--config', metavar='FILE',
                    help='JSON config file listing pairs (see module docstring)')
    ap.add_argument('--pair', nargs=2, metavar=('DOMAIN', 'PROBLEM'),
                    action='append', dest='pairs', default=[],
                    help='A domain/problem pair; may be repeated')
    ap.add_argument('--scan', metavar='DIR',
                    help='Scan directory tree for domain.pddl+problem.pddl pairs')

    # ---- planner ----
    ap.add_argument('--planner',
                    default='timeout TIMEOUT popf -n DOMAIN PROBLEM',
                    help='Planner command template '
                         '(DOMAIN, PROBLEM, and TIMEOUT are replaced at runtime). '
                         'Default: %(default)s')
    ap.add_argument('--timeout', type=int, default=60,
                    help='Planner timeout in seconds, substituted for TIMEOUT in '
                         '--planner. Set 0 to disable. (default: 60)')

    # ---- execution ----
    ap.add_argument('--jobs', type=int, default=1, metavar='N',
                    help='Number of parallel workers (default: 1)')
    ap.add_argument('--data-path', default='/tmp/esterel_batch', metavar='DIR',
                    help='Root directory for planner scratch files '
                         '(sub-dirs per pair). Default: %(default)s')
    ap.add_argument('--epsilon', type=float, default=0.1,
                    help='Epsilon time for TIL upper-bound margin (default: 0.1)')

    # ---- output ----
    ap.add_argument('--output-dir', metavar='DIR',
                    help='Save a JSON result file per pair into this directory')
    ap.add_argument('--summary-json', metavar='FILE',
                    help='Write a machine-readable summary JSON to this file')
    ap.add_argument('--verbose', '-v', action='store_true',
                    help='Print full tracebacks for failed pairs')

    return ap


def main() -> int:
    ap = _build_argparser()
    args = ap.parse_args()

    # ---- collect pairs ----
    specs: List[PairSpec] = []

    if args.config:
        cfg = _load_config(args.config)
        specs.extend(_pairs_from_config(cfg))
        # config-level defaults can override CLI only if CLI is still at default
        if cfg.get('planner') and args.planner == ap.get_default('planner'):
            args.planner = cfg['planner']
        if cfg.get('data_path') and args.data_path == ap.get_default('data_path'):
            args.data_path = cfg['data_path']
        if cfg.get('epsilon') is not None and args.epsilon == ap.get_default('epsilon'):
            args.epsilon = cfg['epsilon']
        if cfg.get('timeout') is not None and args.timeout == ap.get_default('timeout'):
            args.timeout = cfg['timeout']

    if args.scan:
        scanned = _scan_directory(args.scan)
        if not scanned:
            print(f'[warn] No domain.pddl+problem.pddl pairs found under {args.scan}',
                  file=sys.stderr)
        specs.extend(scanned)

    for i, (domain, problem) in enumerate(args.pairs):
        specs.append(PairSpec(
            domain=domain, problem=problem,
            name=f'pair_{i:02d}_{os.path.basename(domain)}',
        ))

    if not specs:
        ap.error('No domain/problem pairs specified. '
                 'Use --pair, --config, or --scan.')

    # ---- assign default names where missing ----
    for i, s in enumerate(specs):
        if not s.name:
            s.name = f'pair_{i:02d}'

    total = len(specs)
    print(f'Running {total} pair(s) with {args.jobs} worker(s)...\n')

    # ---- execute ----
    results: List[PairResult] = [None] * total

    save_json  = bool(args.output_dir)
    cache      = _DomainCache()
    wall_start = time.perf_counter()
    done = 0
    n_ok = 0

    def _on_result(r: PairResult, idx: int) -> None:
        nonlocal done, n_ok
        results[idx] = r
        done += 1
        if r.success:
            n_ok += 1
        # Overwrite the progress bar line, then print the pair result above it
        print('\r' + ' ' * 80 + '\r', end='')
        _print_result(r, args.verbose)
        _print_progress(done, total, n_ok, wall_start)

    if args.jobs == 1:
        for i, spec in enumerate(specs):
            r = _run_pair(spec, args.planner, args.data_path, args.epsilon,
                          save_json, cache, args.timeout)
            _on_result(r, i)
            if save_json:
                _save_json_result(r, args.output_dir)
    else:
        futures = {}
        with ThreadPoolExecutor(max_workers=args.jobs) as pool:
            for i, spec in enumerate(specs):
                fut = pool.submit(_run_pair, spec, args.planner,
                                  args.data_path, args.epsilon, save_json, cache,
                                  args.timeout)
                futures[fut] = i
            for fut in as_completed(futures):
                i = futures[fut]
                r = fut.result()
                _on_result(r, i)
                if save_json:
                    _save_json_result(r, args.output_dir)

    print()  # newline after final progress bar
    _print_summary(results)
    print(f'Domain cache: {cache.size} unique domain(s) parsed')

    # ---- optional machine-readable summary ----
    if args.summary_json:
        summary = [
            {
                'name':      r.spec.name,
                'domain':    r.spec.domain,
                'problem':   r.spec.problem,
                'success':   r.success,
                'elapsed_s': r.elapsed_s,
                'num_nodes': r.num_nodes,
                'num_edges': r.num_edges,
                'error':     r.error or None,
            }
            for r in results
        ]
        with open(args.summary_json, 'w') as fh:
            json.dump(summary, fh, indent=2)
        print(f'Summary written to {args.summary_json}')

    n_fail = sum(1 for r in results if not r.success)
    return 0 if n_fail == 0 else 1


if __name__ == '__main__':
    sys.exit(main())
