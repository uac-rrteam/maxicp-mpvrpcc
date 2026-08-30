#!/usr/bin/env bash

# Do not stop after a failed instance: the remaining instances must still run.
# Undefined variables and pipeline failures are still treated as errors.
set -uo pipefail

readonly TIME_LIMIT_SECONDS=30
readonly MAIN_CLASS="org.maxicp.cp.examples.raw.mpvrp.MPVRPSeqVar"

if (( $# != 1 )); then
    echo "Usage: $0 {nn|opt}" >&2
    exit 1
fi

case "$1" in
    nn)
        readonly ORDERING="nearest-neighbor"
        ;;
    opt)
        readonly ORDERING="2-opt"
        ;;
    *)
        echo "Error: unknown trip ordering '$1'. Expected 'nn' or 'opt'." >&2
        echo "Usage: $0 {nn|opt}" >&2
        exit 1
        ;;
esac

# Resolve every path from the repository root, regardless of the current directory.
script_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
instance_dir="$script_dir/data/MPVRP_CC"
solution_dir="$script_dir/sols_mpvrp_cc/$1"
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
total=${#instances[@]}

echo "$total instance(s) to solve, with ${TIME_LIMIT_SECONDS}s per instance."
echo "Trip ordering: $ORDERING"
echo "Solution directory: $solution_dir"

for index in "${!instances[@]}"; do
    instance=${instances[$index]}
    instance_name=$(basename -- "$instance")
    solution_name="Sol_${instance_name#MPVRP_}"
    final_solution="$solution_dir/$solution_name"
    temporary_solution="$temporary_dir/$solution_name"

    printf '\n[%d/%d] %s\n' "$((index + 1))" "$total" "$instance_name"

    # TERM normally stops the JVM immediately. KILL is used five seconds later
    # only as a safeguard if the process does not react to TERM.
    timeout --signal=TERM --kill-after=5s "${TIME_LIMIT_SECONDS}s" \
        java -cp "$class_dir" "$MAIN_CLASS" "$instance" "$temporary_solution" "$ORDERING"
    exit_code=$?

    # MPVRPSeqVar writes every improving solution immediately. Therefore, a
    # solution found before the timeout remains available after interruption.
    if [[ -s "$temporary_solution" ]]; then
        mv -f -- "$temporary_solution" "$final_solution"
        solved=$((solved + 1))
        if (( exit_code == 124 || exit_code == 137 )); then
            echo "Time limit reached; best solution found was saved: $solution_name"
        elif (( exit_code == 0 )); then
            echo "Search completed; solution saved: $solution_name"
        else
            echo "Search stopped with exit code $exit_code; solution found was saved: $solution_name"
        fi
    else
        without_solution=$((without_solution + 1))
        if (( exit_code == 124 || exit_code == 137 )); then
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
echo "Instances without a new solution: $without_solution"
echo "Failed executions: $errors"
echo "Solution directory: $solution_dir"
