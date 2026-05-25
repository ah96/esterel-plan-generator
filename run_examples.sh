#!/usr/bin/env bash
# run_examples.sh — quick smoke-test / demo for esterel-plan-generator
#
# Runs the generator on every example domain using POPF (10-second timeout)
# and writes results to results/<domain>-<instance>/.
#
# Usage:
#   chmod +x planners/popf planners/lpg-td run_examples.sh
#   ./run_examples.sh              # run all examples with POPF
#   ./run_examples.sh lpg          # run all examples with LPG-td
#   ./run_examples.sh batch        # run all examples in parallel via batch_esterel.py

set -euo pipefail

PLANNER_MODE="${1:-popf}"
TIMEOUT=10
RESULTS_DIR="results"

POPF_CMD="timeout TIMEOUT planners/popf -n DOMAIN PROBLEM"
LPG_CMD="timeout TIMEOUT planners/lpg-td -o DOMAIN -f PROBLEM -n 1"
OPTIC_CMD="timeout TIMEOUT planners/optic-clp DOMAIN PROBLEM"

case "$PLANNER_MODE" in
  lpg)   PLANNER_CMD="$LPG_CMD"   ;;
  optic) PLANNER_CMD="$OPTIC_CMD" ;;
  popf)  PLANNER_CMD="$POPF_CMD"  ;;
  batch) ;;
  *)
    echo "Usage: $0 [popf|lpg|optic|batch]"
    exit 1
    ;;
esac

chmod +x planners/popf planners/lpg-td 2>/dev/null || true

echo "============================================================"
echo "  Esterel Plan Generator — Example Run"
echo "  Mode   : $PLANNER_MODE  (choices: popf | lpg | optic | batch)"
echo "  Timeout: ${TIMEOUT}s"
echo "============================================================"
echo ""

if [ "$PLANNER_MODE" = "batch" ]; then
  echo "Running all examples in parallel via batch_esterel.py ..."
  python3 batch_esterel.py \
      --scan examples/ \
      --planner "$POPF_CMD" \
      --timeout "$TIMEOUT" \
      --jobs "$(nproc)" \
      --output-dir "${RESULTS_DIR}/" \
      --summary-json "${RESULTS_DIR}/summary.json" \
      --verbose
  echo ""
  echo "Summary JSON: ${RESULTS_DIR}/summary.json"
  exit 0
fi

DOMAINS=(depots driverlog rovers satellite zenotravel)
INSTANCES=(instance-01 instance-02)

for domain in "${DOMAINS[@]}"; do
  for inst in "${INSTANCES[@]}"; do
    label="${domain}/${inst}"
    out_dir="${RESULTS_DIR}/${domain}-${inst/instance-/}"
    echo "--- $label ---"
    python3 esterel_plan_generator.py \
        "examples/${domain}/${inst}/domain.pddl" \
        "examples/${domain}/${inst}/problem.pddl" \
        --planner "$PLANNER_CMD" \
        --timeout "$TIMEOUT" \
        --output summary \
        --output-dir "$out_dir"
    echo "    Saved to: $out_dir"
    echo ""
  done
done

echo "============================================================"
echo "  All examples complete. Results in: ${RESULTS_DIR}/"
echo "============================================================"
