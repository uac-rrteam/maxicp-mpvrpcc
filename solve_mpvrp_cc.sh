#!/usr/bin/env bash

# Do not stop after a failed instance: the remaining instances must still run.
# Undefined variables and pipeline failures are still treated as errors.
set -uo pipefail

if (( $# < 1 || $# > 2 )); then
    echo "Usage: $0 {hybrid-opt|hybrid-nn|exact|opt|nn} [time-limit-seconds]" >&2
    exit 1
fi

time_limit_seconds=${2:-30}
if [[ ! "$time_limit_seconds" =~ ^[1-9][0-9]*$ ]]; then
    echo "Error: time limit must be a positive integer number of seconds." >&2
    exit 1
fi

case "$1" in
    nn|hybrid-nn)
        mode="hybrid-nn"
        main_class="org.maxicp.cp.examples.raw.mpvrp.MPVRPSeqVar"
        ordering="nearest-neighbor"
        solver_kind="hybrid"
        ;;
    opt|hybrid|hybrid-opt)
        mode="hybrid-opt"
        main_class="org.maxicp.cp.examples.raw.mpvrp.MPVRPSeqVar"
        ordering="2-opt"
        solver_kind="hybrid"
        ;;
    exact)
        mode="exact"
        main_class="org.maxicp.cp.examples.raw.mpvrp.MPVRP"
        ordering=""
        solver_kind="exact"
        ;;
    *)
        echo "Error: unknown solver mode '$1'." >&2
        echo "Usage: $0 {hybrid-opt|hybrid-nn|exact|opt|nn} [time-limit-seconds]" >&2
        exit 1
        ;;
esac

readonly TIME_LIMIT_SECONDS=$time_limit_seconds
readonly MAIN_CLASS=$main_class
readonly MODE=$mode
readonly ORDERING=$ordering
readonly SOLVER_KIND=$solver_kind

# Resolve every path from the repository root, regardless of the current directory.
script_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
instance_dir="$script_dir/data/MPVRP_CC"
solution_dir="$script_dir/sols_mpvrp_cc/$MODE"
class_dir="$script_dir/target/classes"

# Check the commands required to compile and run the solver.
if ! command -v mvn >/dev/null 2>&1; then
    echo "Error: Maven (mvn) was not found." >&2
    exit 1
fi

if ! command -v java >/dev/null 2>&1; then
    echo "Error: Java was not found." >&2
    exit 1
fi

if ! command -v timeout >/dev/null 2>&1; then
    echo "Error: the GNU timeout command was not found." >&2
    exit 1
fi

if [[ ! -d "$instance_dir" ]]; then
    echo "Error: instance directory not found: $instance_dir" >&2
    exit 1
fi

# Compile once before starting the batch instead of compiling for every instance.
echo "Compiling the project..."
if ! (cd "$script_dir" && mvn -q -DskipTests compile); then
    echo "Error: project compilation failed." >&2
    exit 1
fi

if ! mkdir -p "$solution_dir"; then
    echo "Error: could not create $solution_dir." >&2
    exit 1
fi

# Each run writes to a temporary file first. This prevents an unsuccessful run
# from deleting or replacing a valid solution produced by an earlier execution.
if ! temporary_dir=$(mktemp -d "${TMPDIR:-/tmp}/maxicp-mpvrp-XXXXXX"); then
    echo "Error: could not create the temporary directory." >&2
    exit 1
fi
trap 'rm -rf -- "$temporary_dir"' EXIT

# Bash globbing keeps the canonical lexicographic order of the instance names.
shopt -s nullglob
instances=("$instance_dir"/MPVRP_*.dat)
shopt -u nullglob

if (( ${#instances[@]} == 0 )); then
    echo "No MPVRP_*.dat instance found in $instance_dir." >&2
    exit 1
fi

solved=0
without_solution=0
errors=0
certified=0
total=${#instances[@]}

echo "$total instance(s) to solve, with ${TIME_LIMIT_SECONDS}s per instance."
echo "Solver mode: $MODE"
if [[ "$SOLVER_KIND" == "hybrid" ]]; then
    echo "Trip ordering: $ORDERING"
fi
echo "Solution directory: $solution_dir"

for index in "${!instances[@]}"; do
    instance=${instances[$index]}
    instance_name=$(basename -- "$instance")
    solution_name="Sol_${instance_name#MPVRP_}"
    final_solution="$solution_dir/$solution_name"
    temporary_solution="$temporary_dir/$solution_name"
    temporary_log="$temporary_dir/${solution_name}.log"

    printf '\n[%d/%d] %s\n' "$((index + 1))" "$total" "$instance_name"

    if [[ "$SOLVER_KIND" == "exact" ]]; then
        # MPVRP handles its limit internally so it can print whether the search
        # completed. The external limit is only a five-second JVM safeguard.
        watchdog_seconds=$((TIME_LIMIT_SECONDS + 5))
        solver_command=(java -cp "$class_dir" "$MAIN_CLASS" \
            "$instance" "$temporary_solution" "$TIME_LIMIT_SECONDS")
    else
        watchdog_seconds=$TIME_LIMIT_SECONDS
        solver_command=(java -cp "$class_dir" "$MAIN_CLASS" \
            "$instance" "$temporary_solution" "$ORDERING")
    fi

    timeout --signal=TERM --kill-after=5s "${watchdog_seconds}s" \
        "${solver_command[@]}" 2>&1 | tee "$temporary_log"
    exit_code=${PIPESTATUS[0]}

    search_completed=false
    if grep -Eq 'completed[[:space:]]*:[[:space:]]*true' "$temporary_log"; then
        search_completed=true
    fi
    time_limit_reached=false
    if (( exit_code == 124 || exit_code == 137 )) \
            || grep -q 'Time limit reached' "$temporary_log"; then
        time_limit_reached=true
    fi

    # MPVRPSeqVar writes every improving solution immediately. Therefore, a
    # solution found before the timeout remains available after interruption.
    if [[ -s "$temporary_solution" ]]; then
        mv -f -- "$temporary_solution" "$final_solution"
        solved=$((solved + 1))
        if [[ "$search_completed" == true ]]; then
            certified=$((certified + 1))
            if [[ "$SOLVER_KIND" == "exact" ]]; then
                echo "Exact optimum certified and saved: $solution_name"
            else
                echo "Restricted trip-model optimum certified and saved: $solution_name"
            fi
        elif [[ "$time_limit_reached" == true ]]; then
            echo "Time limit reached; best solution found was saved: $solution_name"
        else
            echo "Search stopped with exit code $exit_code; solution found was saved: $solution_name"
        fi
    else
        without_solution=$((without_solution + 1))
        if [[ "$search_completed" == true ]]; then
            certified=$((certified + 1))
            echo "Search completed without a feasible solution; infeasibility is certified for this model."
        elif [[ "$time_limit_reached" == true ]]; then
            echo "Time limit reached without a solution; continuing with the next instance."
        else
            errors=$((errors + 1))
            echo "No solution produced (exit code $exit_code); continuing with the next instance." >&2
        fi
    fi
done

# Print a short batch summary after all instances have been processed.
printf '\nDone.\n'
echo "Instances processed: $total"
echo "Solutions produced: $solved"
echo "Searches completed with an optimality/infeasibility certificate: $certified"
echo "Instances without a new solution: $without_solution"
echo "Failed executions: $errors"
echo "Solution directory: $solution_dir"
