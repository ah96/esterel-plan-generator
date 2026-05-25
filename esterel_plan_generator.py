#!/usr/bin/env python3
"""
Standalone (non-ROS) Esterel plan generator.

Replicates the logic of:
  - POPFPlannerInterface::runPlanner()
  - PDDLEsterelPlanParser::preparePlan() / createGraph() / makeEdge()

Usage:
  python3 esterel_plan_generator.py <domain.pddl> <problem.pddl> [options]

Or import and call generate_esterel_plan() directly.
"""

import os
import re
import sys
import math
import json
import argparse
import subprocess
from copy import deepcopy
from dataclasses import dataclass, field
from typing import List, Optional, Dict, Tuple

# ---------------------------------------------------------------------------
# Constants (mirrors EsterelPlanNode / EsterelPlanEdge message constants)
# ---------------------------------------------------------------------------

NODE_ACTION_START = 0
NODE_ACTION_END   = 1
NODE_PLAN_START   = 2

EDGE_CONDITION         = 0
EDGE_START_END_ACTION  = 1
EDGE_INTERFERENCE      = 2

_FLOAT_MAX = sys.float_info.max   # 1.7976931348623157e+308 — ROSPlan upper bound for inf

class _FlowList(list):
    """Integer sequences serialised inline (flow style) in YAML."""

# ---------------------------------------------------------------------------
# Data structures (mirror ROS messages without any ROS dependency)
# ---------------------------------------------------------------------------

@dataclass
class KeyValue:
    key:   str
    value: str


@dataclass
class DomainFormula:
    name: str
    typed_parameters: List[KeyValue] = field(default_factory=list)


@dataclass
class KnowledgeItem:
    """Represents a timed initial literal (TIL) from the problem file."""
    attribute_name: str
    values: List[KeyValue] = field(default_factory=list)
    is_negative: bool = False


@dataclass
class DomainOperator:
    formula: DomainFormula = field(default_factory=lambda: DomainFormula(''))
    at_start_add_effects:      List[DomainFormula] = field(default_factory=list)
    at_start_del_effects:      List[DomainFormula] = field(default_factory=list)
    at_end_add_effects:        List[DomainFormula] = field(default_factory=list)
    at_end_del_effects:        List[DomainFormula] = field(default_factory=list)
    at_start_simple_condition: List[DomainFormula] = field(default_factory=list)
    over_all_simple_condition: List[DomainFormula] = field(default_factory=list)
    at_end_simple_condition:   List[DomainFormula] = field(default_factory=list)
    at_start_neg_condition:    List[DomainFormula] = field(default_factory=list)
    over_all_neg_condition:    List[DomainFormula] = field(default_factory=list)
    at_end_neg_condition:      List[DomainFormula] = field(default_factory=list)


@dataclass
class ActionDispatch:
    action_id:     int   = 0
    plan_id:       int   = 0
    name:          str   = ''
    parameters:    List[KeyValue] = field(default_factory=list)
    duration:      float = 0.0
    dispatch_time: float = 0.0


@dataclass
class EsterelPlanNode:
    node_type: int   = NODE_PLAN_START
    node_id:   int   = 0
    name:      str   = ''
    action:    Optional[ActionDispatch] = None
    edges_out: List[int] = field(default_factory=list)
    edges_in:  List[int] = field(default_factory=list)


@dataclass
class EsterelPlanEdge:
    edge_type:            int   = EDGE_CONDITION
    edge_id:              int   = 0
    edge_name:            str   = ''
    signal_type:          int   = 0
    source_ids:           List[int] = field(default_factory=list)
    sink_ids:             List[int] = field(default_factory=list)
    duration_lower_bound: float = 0.001
    duration_upper_bound: float = math.inf


@dataclass
class EsterelPlan:
    nodes: List[EsterelPlanNode] = field(default_factory=list)
    edges: List[EsterelPlanEdge] = field(default_factory=list)


# ---------------------------------------------------------------------------
# PDDL S-expression tokenizer / parser
# ---------------------------------------------------------------------------

def _tokenize(text: str) -> List[str]:
    text = re.sub(r';[^\n]*', '', text)          # strip comments
    return re.findall(r'[()]|[^\s()]+', text.lower())


def _parse_sexp(tokens: List[str], pos: int = 0):
    """Return (sexp, next_pos).  sexp is a list or a string atom."""
    if pos >= len(tokens):
        return None, pos
    if tokens[pos] == '(':
        pos += 1
        items = []
        while pos < len(tokens) and tokens[pos] != ')':
            item, pos = _parse_sexp(tokens, pos)
            items.append(item)
        return items, pos + 1   # consume ')'
    return tokens[pos], pos + 1


# ---------------------------------------------------------------------------
# PDDL domain parser
# ---------------------------------------------------------------------------

def _parse_typed_list(lst: list) -> List[KeyValue]:
    """
    Parse typed parameter list such as [?v - robot ?from ?to - waypoint].
    Multiple parameters before a single '-' share the same type.
    Returns list of KeyValue(key=label, value=type).
    """
    result: List[KeyValue] = []
    batch: List[str] = []
    i = 0
    while i < len(lst):
        token = lst[i]
        if token == '-':
            type_ = lst[i + 1] if i + 1 < len(lst) and isinstance(lst[i + 1], str) else ''
            for label in batch:
                result.append(KeyValue(key=label, value=type_))
            batch = []
            i += 2
        elif isinstance(token, str):
            batch.append(token)
            i += 1
        else:
            i += 1
    # Labels with no type (untyped PDDL)
    for label in batch:
        result.append(KeyValue(key=label, value=''))
    return result


def _parse_atom_formula(sexp: list) -> Optional[DomainFormula]:
    """Parse (pred ?a ?b constant ...) → DomainFormula with key=value=label."""
    if not isinstance(sexp, list) or not sexp or not isinstance(sexp[0], str):
        return None
    name = sexp[0]
    params = [KeyValue(key=p, value=p) for p in sexp[1:] if isinstance(p, str)]
    return DomainFormula(name=name, typed_parameters=params)


def _collect_conditions(sexp, op: DomainOperator, ctx: str = 'at_start', negate: bool = False):
    """
    Recursively walk a :condition expression and classify predicates into
    op.at_start_simple_condition / at_start_neg_condition / over_all_* / at_end_*.
    """
    if not isinstance(sexp, list) or not sexp:
        return
    head = sexp[0].lower() if isinstance(sexp[0], str) else ''

    if head == 'and':
        for sub in sexp[1:]:
            _collect_conditions(sub, op, ctx, negate)

    elif head == 'at' and len(sexp) >= 3 and isinstance(sexp[1], str) and sexp[1].lower() in ('start', 'end'):
        new_ctx = {'start': 'at_start', 'end': 'at_end'}[sexp[1].lower()]
        _collect_conditions(sexp[2], op, new_ctx, negate)

    elif head == 'over' and len(sexp) >= 3 and sexp[1].lower() == 'all':
        _collect_conditions(sexp[2], op, 'over_all', negate)

    elif head == 'not' and len(sexp) >= 2:
        _collect_conditions(sexp[1], op, ctx, not negate)

    elif head in ('>', '<', '>=', '<=', '=', 'forall', 'exists'):
        pass  # skip numeric / quantified conditions

    else:
        f = _parse_atom_formula(sexp)
        if f is None:
            return
        bucket = {
            ('at_start', False): op.at_start_simple_condition,
            ('at_start', True):  op.at_start_neg_condition,
            ('over_all', False): op.over_all_simple_condition,
            ('over_all', True):  op.over_all_neg_condition,
            ('at_end',   False): op.at_end_simple_condition,
            ('at_end',   True):  op.at_end_neg_condition,
        }.get((ctx, negate))
        if bucket is not None:
            bucket.append(f)


def _collect_effects(sexp, op: DomainOperator, ctx: str = 'at_start', negate: bool = False):
    """
    Recursively walk an :effect expression and classify into
    op.at_start_add/del_effects and at_end_add/del_effects.
    """
    if not isinstance(sexp, list) or not sexp:
        return
    head = sexp[0].lower() if isinstance(sexp[0], str) else ''

    if head == 'and':
        for sub in sexp[1:]:
            _collect_effects(sub, op, ctx, negate)

    elif head == 'at' and len(sexp) >= 3 and isinstance(sexp[1], str) and sexp[1].lower() in ('start', 'end'):
        new_ctx = {'start': 'at_start', 'end': 'at_end'}[sexp[1].lower()]
        _collect_effects(sexp[2], op, new_ctx, negate)

    elif head == 'not' and len(sexp) >= 2:
        _collect_effects(sexp[1], op, ctx, not negate)

    elif head in ('assign', 'increase', 'decrease', 'scale-up', 'scale-down', 'forall'):
        pass  # skip numeric / quantified effects

    else:
        f = _parse_atom_formula(sexp)
        if f is None:
            return
        bucket = {
            ('at_start', False): op.at_start_add_effects,
            ('at_start', True):  op.at_start_del_effects,
            ('at_end',   False): op.at_end_add_effects,
            ('at_end',   True):  op.at_end_del_effects,
        }.get((ctx, negate))
        if bucket is not None:
            bucket.append(f)


def _parse_operator(sexp: list, durative: bool) -> DomainOperator:
    """Parse (:durative-action | :action name ...) → DomainOperator."""
    name = sexp[1] if len(sexp) > 1 and isinstance(sexp[1], str) else 'unknown'
    op = DomainOperator(formula=DomainFormula(name=name))

    i = 2
    while i < len(sexp):
        if not isinstance(sexp[i], str):
            i += 1
            continue
        key = sexp[i].lower()
        val = sexp[i + 1] if i + 1 < len(sexp) else None

        if key == ':parameters' and isinstance(val, list):
            op.formula.typed_parameters = _parse_typed_list(val)
        elif key == ':condition' and val is not None:
            _collect_conditions(val, op)
        elif key == ':precondition' and val is not None:
            # Non-durative: treat whole precondition as at_start
            _collect_conditions(val, op, ctx='at_start')
        elif key == ':effect' and val is not None:
            # Non-durative: treat effects as at_end
            ctx = 'at_end' if not durative else 'at_start'
            _collect_effects(val, op, ctx)
        i += 2
    return op


def parse_domain(domain_path: str) -> Dict[str, DomainOperator]:
    """Parse a PDDL domain file. Returns {operator_name: DomainOperator}."""
    with open(domain_path) as fh:
        text = fh.read()
    domain_sexp, _ = _parse_sexp(_tokenize(text))
    if not isinstance(domain_sexp, list):
        return {}

    operators: Dict[str, DomainOperator] = {}
    for item in domain_sexp:
        if not isinstance(item, list) or not item:
            continue
        head = item[0].lower() if isinstance(item[0], str) else ''
        if head == ':durative-action':
            op = _parse_operator(item, durative=True)
            operators[op.formula.name] = op
        elif head == ':action':
            op = _parse_operator(item, durative=False)
            operators[op.formula.name] = op
    return operators


# ---------------------------------------------------------------------------
# Problem file TIL parser
# ---------------------------------------------------------------------------

def parse_problem_tils(problem_path: str) -> List[Tuple[float, KnowledgeItem]]:
    """
    Extract Timed Initial Literals from :init section of a problem file.
    Format:  (at <time> (<predicate> <arg1> ...))
    Returns list sorted ascending by time.
    """
    with open(problem_path) as fh:
        text = fh.read()
    problem_sexp, _ = _parse_sexp(_tokenize(text))
    if not isinstance(problem_sexp, list):
        return []

    tils: List[Tuple[float, KnowledgeItem]] = []
    for item in problem_sexp:
        if not isinstance(item, list) or not item:
            continue
        if item[0].lower() != ':init':
            continue
        for fact in item[1:]:
            if not isinstance(fact, list) or len(fact) < 3:
                continue
            if fact[0].lower() != 'at':
                continue
            try:
                til_time = float(fact[1])
            except (ValueError, TypeError):
                continue
            inner = fact[2]
            if not isinstance(inner, list) or not inner:
                continue
            is_neg = False
            pred = inner
            if inner[0].lower() == 'not' and len(inner) >= 2 and isinstance(inner[1], list):
                is_neg = True
                pred = inner[1]
            if not pred or not isinstance(pred[0], str):
                continue
            ki = KnowledgeItem(
                attribute_name=pred[0],
                is_negative=is_neg,
                values=[KeyValue(key=str(j), value=str(v))
                        for j, v in enumerate(pred[1:]) if isinstance(v, str)]
            )
            tils.append((til_time, ki))

    tils.sort(key=lambda x: x[0])
    return tils


# ---------------------------------------------------------------------------
# Plan text parser + parameter grounding
# ---------------------------------------------------------------------------

def _copy_formulae(formulae: List[DomainFormula]) -> List[DomainFormula]:
    return [DomainFormula(name=f.name,
                          typed_parameters=[KeyValue(kv.key, kv.value) for kv in f.typed_parameters])
            for f in formulae]


def _ground_formulae(formulae: List[DomainFormula],
                     op_params: List[KeyValue],
                     ground_values: List[str]) -> None:
    """Replace parameter labels with ground values in-place."""
    for f in formulae:
        for tp in f.typed_parameters:
            matched = False
            for i, op_p in enumerate(op_params):
                if op_p.key == tp.key and i < len(ground_values):
                    tp.value = ground_values[i]
                    matched = True
                    break
            if not matched:
                tp.value = tp.key   # constant: use label as value


def _ground_operator(op_template: DomainOperator, ground_values: List[str]) -> DomainOperator:
    """Return a deep-copied, fully grounded DomainOperator."""
    op = DomainOperator(
        formula=DomainFormula(
            name=op_template.formula.name,
            typed_parameters=[KeyValue(k.key, k.value) for k in op_template.formula.typed_parameters]
        ),
        at_start_add_effects=_copy_formulae(op_template.at_start_add_effects),
        at_start_del_effects=_copy_formulae(op_template.at_start_del_effects),
        at_end_add_effects=_copy_formulae(op_template.at_end_add_effects),
        at_end_del_effects=_copy_formulae(op_template.at_end_del_effects),
        at_start_simple_condition=_copy_formulae(op_template.at_start_simple_condition),
        over_all_simple_condition=_copy_formulae(op_template.over_all_simple_condition),
        at_end_simple_condition=_copy_formulae(op_template.at_end_simple_condition),
        at_start_neg_condition=_copy_formulae(op_template.at_start_neg_condition),
        over_all_neg_condition=_copy_formulae(op_template.over_all_neg_condition),
        at_end_neg_condition=_copy_formulae(op_template.at_end_neg_condition),
    )
    params = op_template.formula.typed_parameters
    for flist in [
        op.at_start_add_effects, op.at_start_del_effects,
        op.at_end_add_effects,   op.at_end_del_effects,
        op.at_start_simple_condition, op.over_all_simple_condition, op.at_end_simple_condition,
        op.at_start_neg_condition,    op.over_all_neg_condition,    op.at_end_neg_condition,
    ]:
        _ground_formulae(flist, params, ground_values)
    return op


def parse_plan_text(planner_output: str,
                    operators: Dict[str, DomainOperator]
                    ) -> Tuple[List[ActionDispatch], Dict[int, DomainOperator]]:
    """
    Parse POPF/OPTIC plan text.  Each action line looks like:
      0.000: (navigate rover0 waypoint3 waypoint1)  [5.000]

    Returns (action_list, action_details) where action_details maps
    action_id → grounded DomainOperator.
    """
    action_list: List[ActionDispatch] = []
    action_details: Dict[int, DomainOperator] = {}
    action_id = 0

    for line in planner_output.splitlines():
        line = line.strip()
        if len(line) < 2:
            continue
        # Skip rostopic echo -p CSV header
        if line.startswith('%time'):
            continue
        if not all(c in line for c in (':', '(', ')', '[', ']')):
            continue
        try:
            colon = line.index(':')
            time_str = line[:colon].strip()
            # rostopic echo -p prepends a nanosecond timestamp: "1234567890,0.000"
            if ',' in time_str:
                time_str = time_str.rsplit(',', 1)[-1]
            dispatch_time = float(time_str)

            open_p = line.index('(', colon) + 1
            close_p = line.index(')', open_p)
            parts = line[open_p:close_p].split()
            name = parts[0].lower()
            params = [p.lower() for p in parts[1:]]

            open_b = line.index('[', close_p) + 1
            close_b = line.index(']', open_b)
            duration = float(line[open_b:close_b].strip())

        except (ValueError, IndexError):
            continue

        msg = ActionDispatch(
            action_id=action_id, name=name,
            duration=duration, dispatch_time=dispatch_time
        )

        if name in operators:
            grounded_op = _ground_operator(operators[name], params)
            # Store parameters in action message  (key=param_label, value=ground_value)
            for i, op_p in enumerate(operators[name].formula.typed_parameters):
                if i < len(params):
                    msg.parameters.append(KeyValue(key=op_p.key.lstrip('?'), value=params[i]))
            action_details[action_id] = grounded_op
        else:
            action_details[action_id] = DomainOperator()

        action_list.append(msg)
        action_id += 1

    return action_list, action_details


# ---------------------------------------------------------------------------
# Esterel plan builder  (mirrors PDDLEsterelPlanParser)
# ---------------------------------------------------------------------------

class EsterelPlanBuilder:

    EPSILON = 0.001   # minimum lower-bound separation (makeEdge epsilon)

    def __init__(self, epsilon_time: float = 0.1):
        self.epsilon_time = epsilon_time      # TIL upper-bound safety margin
        self.plan = EsterelPlan()
        self.action_details: Dict[int, DomainOperator] = {}
        self.til_list: List[Tuple[float, KnowledgeItem]] = []  # sorted ascending

    # ------------------------------------------------------------------
    # Public entry point
    # ------------------------------------------------------------------

    def build(self,
              planner_output: str,
              operators: Dict[str, DomainOperator],
              tils: Optional[List[Tuple[float, KnowledgeItem]]] = None) -> EsterelPlan:
        """Build and return the Esterel plan graph."""
        self.plan = EsterelPlan()
        self.til_list = sorted(tils or [], key=lambda x: x[0])

        # Node 0: plan start
        self.plan.nodes.append(EsterelPlanNode(
            node_type=NODE_PLAN_START, node_id=0, name='plan_start'))

        action_list, self.action_details = parse_plan_text(planner_output, operators)

        for msg in action_list:
            nid_start = len(self.plan.nodes)
            self.plan.nodes.append(EsterelPlanNode(
                node_type=NODE_ACTION_START,
                node_id=nid_start,
                name=f'{msg.name}_start',
                action=msg))
            self.plan.nodes.append(EsterelPlanNode(
                node_type=NODE_ACTION_END,
                node_id=nid_start + 1,
                name=f'{msg.name}_end',
                action=msg))

        self._create_graph()
        return self.plan

    # ------------------------------------------------------------------
    # Graph creation  (mirrors createGraph)
    # ------------------------------------------------------------------

    def _create_graph(self) -> None:
        # Build time-ordered list of (time, node_id).
        # Round end times to 3 dp (POPF output precision) to avoid floating-point
        # accumulation (e.g. 13.002 + 10.0 = 23.002000000000002) that would push
        # an action_end past a simultaneous action_start in the sort, causing the
        # backward scan to miss it as a causal provider.
        node_times: List[Tuple[float, int]] = []
        for node in self.plan.nodes:
            if node.node_type == NODE_ACTION_START:
                t = node.action.dispatch_time
            elif node.node_type == NODE_ACTION_END:
                t = round(node.action.dispatch_time + node.action.duration, 3)
            else:
                t = 0.0
            node_times.append((t, node.node_id))
        node_times.sort()

        for idx, (t, nid) in enumerate(node_times):
            node = self.plan.nodes[nid]

            # ACTION_END: add start→end duration edge, then fall through
            if node.node_type == NODE_ACTION_END:
                d = node.action.duration
                self._make_edge(nid - 1, nid, d, d, EDGE_START_END_ACTION)

            if node.node_type not in (NODE_ACTION_START, NODE_ACTION_END):
                continue

            op = self.action_details.get(node.action.action_id, DomainOperator())
            edge_created = False

            if node.node_type == NODE_ACTION_START:
                cond_groups = [
                    (op.at_start_simple_condition, False, False),
                    (op.over_all_simple_condition, False, True),
                    (op.at_start_neg_condition,    True,  False),
                    (op.over_all_neg_condition,    True,  True),
                ]
            else:
                cond_groups = [
                    (op.at_end_simple_condition, False, False),
                    (op.at_end_neg_condition,    True,  False),
                ]

            for cond_list, neg, overall in cond_groups:
                for cond in cond_list:
                    if self._add_condition_edge(node_times, idx, cond, neg, overall):
                        edge_created = True

            if self._add_interference_edges(node_times, idx):
                edge_created = True

            if not edge_created:
                self._make_edge(0, nid, 0.0, math.inf, EDGE_CONDITION)

    # ------------------------------------------------------------------
    # Condition edge  (mirrors addConditionEdge)
    # ------------------------------------------------------------------

    def _add_condition_edge(self,
                            node_times: List[Tuple[float, int]],
                            current_idx: int,
                            condition: DomainFormula,
                            negative: bool,
                            overall: bool) -> bool:
        current_time, current_nid = node_times[current_idx]

        # --- Phase 1: TILs AFTER current node that would violate the condition ---
        # Sweep backwards from the latest TIL.
        # tit is an index into self.til_list (sorted ascending → sweep from end).
        tit = len(self.til_list) - 1
        while tit >= 0 and self.til_list[tit][0] > current_time:
            til_time, til = self.til_list[tit]
            if self._satisfies_til(condition, til, not negative):
                target = current_nid + 1 if overall else current_nid
                ub = til_time - self.epsilon_time
                self._make_edge(0, target, 0.0, ub, EDGE_CONDITION)
            tit -= 1

        # tit now points to the latest TIL at or before current_time.

        # --- Phase 2: scan previous nodes/TILs for causal support ---
        for j in range(current_idx - 1, -1, -1):
            prev_time, prev_nid = node_times[j]

            # TILs between prev_time and the current tit position
            while tit >= 0 and self.til_list[tit][0] > prev_time:
                til_time, til = self.til_list[tit]
                if self._satisfies_til(condition, til, negative):
                    self._make_edge(0, current_nid, til_time, math.inf, EDGE_CONDITION)
                    return True
                tit -= 1

            prev_node = self.plan.nodes[prev_nid]
            if self._satisfies_node(condition, prev_node, negative):
                self._make_edge(prev_nid, current_nid, 0.0, math.inf, EDGE_CONDITION)
                return True

        return False

    # ------------------------------------------------------------------
    # Interference edges  (mirrors addInterferenceEdges)
    # ------------------------------------------------------------------

    def _add_interference_edges(self,
                                 node_times: List[Tuple[float, int]],
                                 current_idx: int) -> bool:
        _, current_nid = node_times[current_idx]
        current_node = self.plan.nodes[current_nid]
        edge_added = False

        for j in range(current_idx - 1, -1, -1):
            _, prev_nid = node_times[j]
            prev_node = self.plan.nodes[prev_nid]

            if prev_node.node_type not in (NODE_ACTION_START, NODE_ACTION_END):
                continue

            op = self.action_details.get(prev_node.action.action_id, DomainOperator())

            # For each formula in prev_node, check if current_node's effects interfere.
            # satisfies_node(f, current_node, neg=True)  → current_node DELETES f
            # satisfies_node(f, current_node, neg=False) → current_node ADDS   f
            if prev_node.node_type == NODE_ACTION_START:
                checks = [
                    (op.at_start_simple_condition, True),   # current deletes pos cond
                    (op.at_start_neg_condition,    False),  # current adds neg-cond formula
                    (op.at_start_add_effects,      True),   # current deletes prev add-eff
                    (op.at_start_del_effects,      False),  # current adds prev del-eff
                ]
            else:
                checks = [
                    (op.at_end_simple_condition,   True),
                    (op.at_end_neg_condition,      False),
                    (op.over_all_simple_condition, True),
                    (op.over_all_neg_condition,    False),
                    (op.at_end_add_effects,        True),
                    (op.at_end_del_effects,        False),
                ]

            interferes = False
            for formulae, neg in checks:
                for f in formulae:
                    if self._satisfies_node(f, current_node, neg):
                        interferes = True
                        break
                if interferes:
                    break

            if interferes:
                self._make_edge(prev_nid, current_nid, 0.0, math.inf, EDGE_INTERFERENCE)
                edge_added = True

        return edge_added

    # ------------------------------------------------------------------
    # Predicate matching helpers  (mirrors satisfiesPrecondition / domainFormulaMatches)
    # ------------------------------------------------------------------

    @staticmethod
    def _formula_matches(a: DomainFormula, b: DomainFormula) -> bool:
        if a.name != b.name or len(a.typed_parameters) != len(b.typed_parameters):
            return False
        return all(pa.value == pb.value
                   for pa, pb in zip(a.typed_parameters, b.typed_parameters))

    def _satisfies_node(self, condition: DomainFormula,
                        node: EsterelPlanNode, negative: bool) -> bool:
        """Does node's add-effects (negative=False) or del-effects (negative=True) match condition?"""
        if node.action is None:
            return False
        op = self.action_details.get(node.action.action_id)
        if op is None:
            return False
        if not negative:
            effs = (op.at_start_add_effects if node.node_type == NODE_ACTION_START
                    else op.at_end_add_effects)
        else:
            effs = (op.at_start_del_effects if node.node_type == NODE_ACTION_START
                    else op.at_end_del_effects)
        return any(self._formula_matches(condition, e) for e in effs)

    def _satisfies_til(self, condition: DomainFormula,
                       til: KnowledgeItem, negative: bool) -> bool:
        """Does the TIL satisfy (or negate) condition?"""
        eff = DomainFormula(
            name=til.attribute_name,
            typed_parameters=[KeyValue(kv.key, kv.value) for kv in til.values]
        )
        return (negative == til.is_negative) and self._formula_matches(condition, eff)

    # ------------------------------------------------------------------
    # Edge construction  (mirrors makeEdge)
    # ------------------------------------------------------------------

    def _make_edge(self, source_id: int, sink_id: int,
                   lower: float, upper: float, edge_type: int) -> None:
        """Create or update an edge.  Applies minimum epsilon to zero bounds."""
        if lower == 0.0:
            lower = self.EPSILON
        if upper == 0.0:
            upper = self.EPSILON

        # Check for existing edge between same source/sink
        for eid in self.plan.nodes[source_id].edges_out:
            if 0 <= eid < len(self.plan.edges):
                e = self.plan.edges[eid]
                if sink_id in e.sink_ids:
                    if lower > e.duration_lower_bound:
                        e.duration_lower_bound = lower
                    if upper < e.duration_upper_bound:
                        e.duration_upper_bound = upper
                    return

        edge = EsterelPlanEdge(
            edge_type=edge_type,
            edge_id=len(self.plan.edges),
            edge_name=f'edge_{len(self.plan.edges)}',
            signal_type=0,
            source_ids=[source_id],
            sink_ids=[sink_id],
            duration_lower_bound=lower,
            duration_upper_bound=upper,
        )
        self.plan.edges.append(edge)
        self.plan.nodes[source_id].edges_out.append(edge.edge_id)
        self.plan.nodes[sink_id].edges_in.append(edge.edge_id)


# ---------------------------------------------------------------------------
# Planner runner  (mirrors POPFPlannerInterface::runPlanner)
# ---------------------------------------------------------------------------

def run_planner(domain_path: str,
                problem_path: str,
                data_path: str,
                planner_command: str = 'timeout TIMEOUT popf -n DOMAIN PROBLEM',
                timeout: int = 60,
                ) -> Tuple[bool, str]:
    """
    Run the external planner and return (solved, plan_text).
    plan_text contains only the action lines (no header).

    DOMAIN, PROBLEM, and TIMEOUT in planner_command are substituted at runtime.
    """
    os.makedirs(data_path, exist_ok=True)
    if not data_path.endswith('/'):
        data_path += '/'

    cmd = (planner_command
           .replace('TIMEOUT', str(timeout))
           .replace('DOMAIN', domain_path)
           .replace('PROBLEM', problem_path))
    plan_file = data_path + 'plan.pddl'

    with open(plan_file, 'w') as _pf:
        subprocess.run(cmd, shell=True, stdout=_pf, stderr=_pf)

    solved = False
    planner_output = ''

    try:
        with open(plan_file) as fh:
            lines = fh.readlines()
    except FileNotFoundError:
        return False, ''

    i = 0
    while i < len(lines):
        line = lines[i]
        if '; Plan found' in line or ';;;; Solution Found' in line:
            solved = True
        if '; Time' in line:
            j = i + 1
            block: List[str] = []
            while j < len(lines) and len(lines[j].strip()) >= 2:
                block.append(lines[j])
                j += 1
            planner_output = ''.join(block)   # keep last plan if multiple
        i += 1

    return solved, planner_output


# ---------------------------------------------------------------------------
# Serialisation helpers
# ---------------------------------------------------------------------------

def _kv_to_dict(kv: KeyValue) -> dict:
    return {'key': kv.key, 'value': kv.value}


def _action_to_dict(a: ActionDispatch) -> dict:
    return {
        'action_id':     a.action_id,
        'name':          a.name,
        'parameters':    [_kv_to_dict(p) for p in a.parameters],
        'duration':      a.duration,
        'dispatch_time': a.dispatch_time,
    }


def plan_to_dict(plan: EsterelPlan) -> dict:
    """Serialise EsterelPlan to a plain dict (JSON-serialisable).

    node_type and edge_type are stored as integers to match the ROSPlan
    message constants (NODE_ACTION_START=0, NODE_ACTION_END=1,
    NODE_PLAN_START=2, EDGE_CONDITION=0, EDGE_START_END_ACTION=1,
    EDGE_INTERFERENCE=2).
    """
    nodes = []
    for n in plan.nodes:
        nd = {
            'node_id':   n.node_id,
            'node_type': n.node_type,
            'name':      n.name,
            'edges_in':  n.edges_in,
            'edges_out': n.edges_out,
        }
        if n.action is not None:
            nd['action'] = _action_to_dict(n.action)
        nodes.append(nd)

    edges = []
    for e in plan.edges:
        edges.append({
            'edge_id':              e.edge_id,
            'edge_name':            e.edge_name,
            'edge_type':            e.edge_type,
            'source_ids':           e.source_ids,
            'sink_ids':             e.sink_ids,
            'duration_lower_bound': e.duration_lower_bound,
            'duration_upper_bound': e.duration_upper_bound if e.duration_upper_bound != math.inf else None,
        })
    return {'nodes': nodes, 'edges': edges}


# ---------------------------------------------------------------------------
# ROSPlan-compatible YAML serialisation
# ---------------------------------------------------------------------------

def _action_to_rosplan_dict(a: ActionDispatch) -> dict:
    params = ([{'key': p.key.lstrip('?'), 'value': p.value} for p in a.parameters]
              if a.parameters else _FlowList())
    return {
        'action_id':     a.action_id,
        'plan_id':       0,
        'name':          a.name,
        'parameters':    params,
        'duration':      float(a.duration),
        'dispatch_time': float(a.dispatch_time),
    }


def plan_to_rosplan_dict(plan: EsterelPlan) -> dict:
    """Build a dict matching the ROSPlan YAML message format exactly.

    Matches `rostopic echo /rosplan_parsing_interface/complete_plan -n 1`.
    Differences from plan_to_dict: plan_start has a zeroed action field,
    action includes plan_id, edge includes signal_type, infinity is
    sys.float_info.max, integer lists use _FlowList for inline YAML style.
    """
    nodes = []
    for n in plan.nodes:
        if n.node_type == NODE_PLAN_START:
            action_d = {
                'action_id':     0,
                'plan_id':       0,
                'name':          '',
                'parameters':    _FlowList(),
                'duration':      0.0,
                'dispatch_time': 0.0,
            }
        else:
            action_d = _action_to_rosplan_dict(n.action)
        nodes.append({
            'node_type': n.node_type,
            'node_id':   n.node_id,
            'name':      n.name,
            'action':    action_d,
            'edges_out': _FlowList(n.edges_out),
            'edges_in':  _FlowList(n.edges_in),
        })

    edges = []
    for e in plan.edges:
        ub = e.duration_upper_bound
        if math.isinf(ub) or ub >= _FLOAT_MAX:
            ub = _FLOAT_MAX
        edges.append({
            'edge_type':            e.edge_type,
            'edge_id':              e.edge_id,
            'edge_name':            e.edge_name,
            'signal_type':          e.signal_type,
            'source_ids':           _FlowList(e.source_ids),
            'sink_ids':             _FlowList(e.sink_ids),
            'duration_lower_bound': e.duration_lower_bound,
            'duration_upper_bound': ub,
        })
    return {'nodes': nodes, 'edges': edges}


def _dump_rosplan_yaml(data: dict, fh) -> None:
    """Write an EsterelPlan dict to *fh* in ROSPlan's YAML style.

    Hand-written serialiser — matches the exact formatting of
    `rostopic echo /rosplan_parsing_interface/complete_plan -n 1`.
    """
    def _str_val(s: str) -> str:
        return "''" if s == '' else f'"{s}"'

    def _float_val(f: float) -> str:
        if math.isinf(f) or f >= _FLOAT_MAX:
            return repr(_FLOAT_MAX)
        r = repr(f)
        if '.' not in r and 'e' not in r.lower():
            r += '.0'
        return r

    def _scalar(v) -> str:
        if isinstance(v, str):   return _str_val(v)
        if isinstance(v, float): return _float_val(v)
        if isinstance(v, bool):  return 'true' if v else 'false'
        return str(v)

    def _write_val(k: str, v, indent: int) -> None:
        pad = '  ' * indent
        if isinstance(v, _FlowList):
            fh.write(f'{pad}{k}: [{", ".join(str(x) for x in v)}]\n')
        elif isinstance(v, list):
            if not v:
                fh.write(f'{pad}{k}: []\n')
            else:
                fh.write(f'{pad}{k}: \n')
                for item in v:
                    if isinstance(item, dict):
                        fh.write(f'{pad}  - \n')
                        for ik, iv in item.items():
                            _write_val(ik, iv, indent + 2)
                    else:
                        fh.write(f'{pad}  - {_scalar(item)}\n')
        elif isinstance(v, dict):
            fh.write(f'{pad}{k}: \n')
            for dk, dv in v.items():
                _write_val(dk, dv, indent + 1)
        else:
            fh.write(f'{pad}{k}: {_scalar(v)}\n')

    fh.write('nodes: \n')
    for node in data['nodes']:
        fh.write('  - \n')
        for k, v in node.items():
            _write_val(k, v, indent=2)
    fh.write('edges: \n')
    for edge in data['edges']:
        fh.write('  - \n')
        for k, v in edge.items():
            _write_val(k, v, indent=2)


# ---------------------------------------------------------------------------
# Top-level convenience function
# ---------------------------------------------------------------------------

def generate_esterel_plan(
        domain_path: str,
        problem_path: str,
        planner_command: str = 'timeout TIMEOUT popf -n DOMAIN PROBLEM',
        data_path: str = '/tmp/rosplan_esterel',
        epsilon_time: float = 0.1,
        timeout: int = 60,
) -> Optional[EsterelPlan]:
    """
    Full pipeline:
      1. Run the PDDL planner (default: POPF).
      2. Parse the domain to extract operator pre/effects.
      3. Parse TILs from the problem file.
      4. Build and return the Esterel plan graph.

    Returns None if planning failed.
    """
    solved, planner_output = run_planner(
        domain_path, problem_path, data_path, planner_command, timeout)
    if not solved:
        print('[esterel] Planning failed: no solution found.', file=sys.stderr)
        return None

    operators = parse_domain(domain_path)
    tils = parse_problem_tils(problem_path)

    builder = EsterelPlanBuilder(epsilon_time=epsilon_time)
    return builder.build(planner_output, operators, tils)


def generate_esterel_plan_from_text(
        plan_text: str,
        domain_path: str,
        problem_path: str,
        epsilon_time: float = 0.1,
) -> EsterelPlan:
    """
    Build an Esterel plan from an already-computed plan text string
    (skips the planner step).
    """
    operators = parse_domain(domain_path)
    tils = parse_problem_tils(problem_path)
    builder = EsterelPlanBuilder(epsilon_time=epsilon_time)
    return builder.build(plan_text, operators, tils)


# ---------------------------------------------------------------------------
# CLI entry point
# ---------------------------------------------------------------------------

if __name__ == '__main__':
    ap = argparse.ArgumentParser(
        description='Generate an Esterel plan graph from PDDL domain + problem files.')
    ap.add_argument('domain',  help='Path to PDDL domain file')
    ap.add_argument('problem', help='Path to PDDL problem file')
    ap.add_argument('--planner',
                    default='timeout TIMEOUT popf -n DOMAIN PROBLEM',
                    help='Planner command template (DOMAIN, PROBLEM, and TIMEOUT are '
                         'substituted at runtime). Default: %(default)s')
    ap.add_argument('--timeout', type=int, default=60,
                    help='Planner timeout in seconds, substituted for TIMEOUT in '
                         '--planner. Set 0 to disable. (default: 60)')
    ap.add_argument('--data-path', default='/tmp/rosplan_esterel',
                    help='Temporary directory for planner output (default: /tmp/rosplan_esterel)')
    ap.add_argument('--epsilon', type=float, default=0.1,
                    help='Epsilon time for TIL upper-bound margin (default: 0.1)')
    ap.add_argument('--output', choices=['summary', 'json'], default='summary',
                    help='Output format for stdout (default: summary)')
    ap.add_argument('--output-dir', metavar='DIR', default=None,
                    help='If set, write esterel_plan.json, esterel_plan.yaml, and plan.txt '
                         'into this directory (yaml/plan.txt match ROSPlan format)')
    ap.add_argument('--plan-file', metavar='FILE', default=None,
                    help='Use an existing plan file instead of running the planner. '
                         'Accepts raw POPF output (plan.pddl) or rostopic echo -p '
                         'format (plan.txt).  Skips --planner and --data-path.')
    args = ap.parse_args()

    if args.plan_file:
        with open(args.plan_file) as fh:
            planner_output = fh.read()
    else:
        # Run pipeline steps directly so planner_output is available for plan.txt
        solved, planner_output = run_planner(
            domain_path=args.domain,
            problem_path=args.problem,
            data_path=args.data_path,
            planner_command=args.planner,
            timeout=args.timeout,
        )
        if not solved:
            print('[esterel] Planning failed: no solution found.', file=sys.stderr)
            sys.exit(1)

    operators = parse_domain(args.domain)
    tils      = parse_problem_tils(args.problem)
    builder   = EsterelPlanBuilder(epsilon_time=args.epsilon)
    plan      = builder.build(planner_output, operators, tils)

    if args.output == 'json':
        print(json.dumps(plan_to_dict(plan), indent=2))
    else:
        node_type_names = {NODE_ACTION_START: 'ACTION_START',
                           NODE_ACTION_END:   'ACTION_END',
                           NODE_PLAN_START:   'PLAN_START'}
        edge_type_names = {EDGE_CONDITION:        'CONDITION',
                           EDGE_START_END_ACTION: 'START_END_ACTION',
                           EDGE_INTERFERENCE:     'INTERFERENCE'}
        print(f'Esterel plan: {len(plan.nodes)} nodes, {len(plan.edges)} edges\n')
        for n in plan.nodes:
            nt = node_type_names.get(n.node_type, str(n.node_type))
            print(f'  Node {n.node_id:3d} [{nt:16s}] {n.name}')
        print()
        for e in plan.edges:
            et = edge_type_names.get(e.edge_type, str(e.edge_type))
            ub = f'{e.duration_upper_bound:.3f}' if e.duration_upper_bound != math.inf else 'inf'
            print(f'  Edge {e.edge_id:3d} [{et:18s}] '
                  f'{e.source_ids} → {e.sink_ids}  '
                  f'[{e.duration_lower_bound:.3f}, {ub}]')

    if args.output_dir:
        import time as _time_mod
        out = os.path.realpath(args.output_dir)
        os.makedirs(out, exist_ok=True)

        # esterel_plan.json  (portable; None for infinity)
        json_path = os.path.join(out, 'esterel_plan.json')
        with open(json_path, 'w') as fh:
            json.dump(plan_to_dict(plan), fh, indent=2)

        # esterel_plan.yaml  (matches ROSPlan message format exactly)
        yaml_path = os.path.join(out, 'esterel_plan.yaml')
        with open(yaml_path, 'w') as fh:
            _dump_rosplan_yaml(plan_to_rosplan_dict(plan), fh)

        # plan.txt  (matches `rostopic echo /rosplan_planner_interface/planner_output -p -n 1`)
        # Extract only action lines so raw POPF output (with search stats) is handled cleanly.
        ns = _time_mod.time_ns()
        action_lines = []
        for raw in planner_output.splitlines():
            s = raw.strip()
            if s.startswith('%time') or not all(c in s for c in (':', '(', ')', '[', ']')):
                continue
            # Strip rostopic echo timestamp prefix "1234567890,0.000: ..." → "0.000: ..."
            colon = s.index(':')
            if ',' in s[:colon]:
                s = s[s.index(',') + 1:]
            action_lines.append(s)
        plan_txt_path = os.path.join(out, 'plan.txt')
        with open(plan_txt_path, 'w') as fh:
            fh.write('%time,field.data\n')
            for i, line in enumerate(action_lines):
                fh.write(f'{ns},{line}\n' if i == 0 else f'{line}\n')
            fh.write('\n')

        print(f'\nSaved to {out}/')
