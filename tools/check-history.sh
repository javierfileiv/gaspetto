#!/usr/bin/env bash

# Walk the commit history and replay the CI checks on every single commit.
#
# usage: tools/check-history.sh <start-sha> [options]
#
# Every commit between <start-sha> and <end> (HEAD when the script launches)
# is checked out in a throwaway worktree and runs the same commands the
# GitHub Actions workflow runs: the result tells which commit broke the CI,
# not just that "the branch is broken". Your own working tree is never
# touched, and the build directories live in the worktree so every commit
# rebuilds only what it changed.

set -uo pipefail

RED=$'\033[0;31m'
GREEN=$'\033[0;32m'
YELLOW=$'\033[1;33m'
BLUE=$'\033[0;34m'
NC=$'\033[0m'

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" &>/dev/null && pwd)"
REPO="$(dirname "$SCRIPT_DIR")"

# Same gate as the coverage job in .github/workflows/ci.yml
COVERAGE_MIN="${COVERAGE_MIN:-95}"

# pc        build-gaspetto-pc.sh      (release+debug car/box + smoke runs)
# utest     build-gaspetto-utest.sh   (utest_car + utest_box)
# coverage  coverage-gaspetto-utest   (gcovr + the COVERAGE_MIN gate)
# pio       build-gaspetto-pio.sh     (the 8 PlatformIO projects)
# commitlint the commit message rules of commitlint.config.mjs
ALL_CHECKS="pc,utest,coverage,pio,commitlint"
CHECKS_CSV="$ALL_CHECKS"

END_SHA=""
START_SHA=""
WORKTREE=""
STOP_ON_FAIL=0
KEEP_WORKTREE=0

usage() {
    cat <<EOF
Usage: $0 <start-sha> [options]

Walks history from <start-sha> to <end> (default: HEAD at launch) and runs
the CI checks on each commit individually.

Options:
  --end <sha>         last commit to check (default: HEAD when launched)
  --checks <list>     comma list of: ${ALL_CHECKS}
                      (default: all of them)
  --fast              shortcut for --checks utest
  --stop-on-fail      stop at the first commit that fails
  --worktree <dir>    where to create the throwaway worktree
                      (default: a temp directory under /tmp/gaspetto)
  --keep              keep the worktree and the logs after the run
  -h, --help          this help

Exit code: 0 when every commit passes every check, 1 otherwise.
COVERAGE_MIN=${COVERAGE_MIN} (same as CI).
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --end)          END_SHA="${2:-}"; shift 2 ;;
        --checks)       CHECKS_CSV="${2:-}"; shift 2 ;;
        --fast)         CHECKS_CSV="utest"; shift ;;
        --stop-on-fail) STOP_ON_FAIL=1; shift ;;
        --worktree)     WORKTREE="${2:-}"; shift 2 ;;
        --keep)         KEEP_WORKTREE=1; shift ;;
        -h|--help)      usage; exit 0 ;;
        -*)             echo "Unknown option: $1" >&2; usage >&2; exit 2 ;;
        *)              if [[ -z "$START_SHA" ]]; then
                            START_SHA="$1"; shift
                        else
                            echo "Unexpected argument: $1" >&2; usage >&2; exit 2
                        fi ;;
    esac
done

if [[ -z "$START_SHA" ]]; then
    usage >&2
    exit 2
fi

# Resolve both ends before touching anything
START_SHA="$(git -C "$REPO" rev-parse --verify "${START_SHA}^{commit}" 2>/dev/null)" || {
    echo "Not a commit: $START_SHA" >&2
    exit 2
}
if [[ -z "$END_SHA" ]]; then
    END_SHA="$(git -C "$REPO" rev-parse --verify HEAD)"
else
    END_SHA="$(git -C "$REPO" rev-parse --verify "${END_SHA}^{commit}" 2>/dev/null)" || {
        echo "Not a commit: $END_SHA" >&2
        exit 2
    }
fi

if ! git -C "$REPO" merge-base --is-ancestor "$START_SHA" "$END_SHA"; then
    echo "The start commit is not an ancestor of the end commit:" >&2
    echo "  start: $(git -C "$REPO" log -1 --oneline "$START_SHA")" >&2
    echo "  end  : $(git -C "$REPO" log -1 --oneline "$END_SHA")" >&2
    echo "Pick a start commit that is in the history of the end one." >&2
    exit 2
fi

IFS=',' read -r -a CHECKS <<< "$CHECKS_CSV"
for check in "${CHECKS[@]}"; do
    case "$check" in
        pc|utest|coverage|pio|commitlint) ;;
        *) echo "Unknown check: $check (valid: ${ALL_CHECKS//,/ })" >&2; exit 2 ;;
    esac
done

# The range: START^..END keeps the start commit in the list
if git -C "$REPO" rev-parse --verify "${START_SHA}^" &>/dev/null; then
    RANGE_START="${START_SHA}^.."
else
    RANGE_START=""
fi
mapfile -t SHAS < <(git -C "$REPO" rev-list --reverse "${RANGE_START}${END_SHA}")
COUNT="${#SHAS[@]}"

WORK_DIR="${WORKTREE:-/tmp/gaspetto/gaspetto-history-$$}"
TREE="$WORK_DIR/tree"
LOG_DIR="$WORK_DIR/logs"

echo -e "${BLUE}Gaspetto history check${NC}"
echo "  range    : $(git -C "$REPO" log -1 --oneline "$START_SHA")"
echo "             -> $(git -C "$REPO" log -1 --oneline "$END_SHA")"
echo "  commits  : $COUNT"
echo "  checks   : ${CHECKS[*]}"
echo "  worktree : $TREE"
echo "  logs     : $LOG_DIR"
echo "  coverage : min ${COVERAGE_MIN}%"
echo

cleanup() {
    if [[ "$KEEP_WORKTREE" -eq 1 ]]; then
        echo -e "${YELLOW}Worktree and logs kept in $WORK_DIR${NC}"
        return
    fi
    git -C "$REPO" worktree remove --force "$TREE" &>/dev/null || true
    git -C "$REPO" worktree prune &>/dev/null || true
    rm -rf "$WORK_DIR"
}
trap cleanup EXIT

mkdir -p "$LOG_DIR"
if ! git -C "$REPO" worktree add --quiet --detach "$TREE" "$END_SHA"; then
    echo "Could not create the worktree at $TREE" >&2
    exit 2
fi

# commitlint is what CI runs (defaultIgnores: false, so fixup/squash are
# rejected there); the local tools/check-commit-msg is more permissive and
# is only a fallback, announced once below
COMMITLINT_CMD=""
if command -v commitlint &>/dev/null; then
    COMMITLINT_CMD="commitlint"
elif command -v npx &>/dev/null && npx --no-install commitlint --version &>/dev/null; then
    COMMITLINT_CMD="npx --no-install commitlint"
fi

# One check, run inside the worktree, output captured in its own log
run_check() {
    local check="$1" sha="$2"
    case "$check" in
        pc)
            "$TREE/tools/build-gaspetto-pc.sh"
            ;;
        utest)
            "$TREE/tools/build-gaspetto-utest.sh"
            ;;
        coverage)
            "$TREE/tools/coverage-gaspetto-utest" || return 1
            local cov="$TREE/soft/emu-pc/build-tests-coverage/coverage/coverage.txt"
            [[ -f "$cov" ]] || { echo "no coverage.txt produced" >&2; return 1; }
            local total
            total="$(grep '^TOTAL' "$cov" | tr -s ' ' | awk '{print $4}' | tr -d '%')"
            [[ -n "$total" ]] || { echo "no TOTAL line in coverage.txt" >&2; return 1; }
            if awk -v t="$total" -v m="$COVERAGE_MIN" 'BEGIN { exit !(t + 0 < m + 0) }'; then
                echo "coverage gate failed: ${total}% < ${COVERAGE_MIN}%" >&2
                return 1
            fi
            echo "coverage ${total}% (min ${COVERAGE_MIN}%)"
            ;;
        pio)
            "$TREE/tools/build-gaspetto-pio.sh"
            ;;
        commitlint)
            local msg_file="$LOG_DIR/.msg.$RANDOM"
            git -C "$REPO" log -1 --format=%B "$sha" > "$msg_file"
            if [[ -n "$COMMITLINT_CMD" ]]; then
                (cd "$TREE" && $COMMITLINT_CMD < "$msg_file")
            else
                "$TREE/tools/check-commit-msg" "$msg_file"
            fi
            local rc=$?
            rm -f "$msg_file"
            return $rc
            ;;
    esac
}

if [[ " ${CHECKS[*]} " == *" commitlint "* && -z "$COMMITLINT_CMD" ]]; then
    echo -e "${YELLOW}commitlint not installed: falling back to tools/check-commit-msg${NC}"
    echo -e "${YELLOW}which is more permissive than CI (it accepts fixup/squash commits).${NC}"
    echo
fi

declare -A RESULTS
FAILURES=()
START_TS=$(date +%s)
INDEX=0

for sha in "${SHAS[@]}"; do
    INDEX=$((INDEX + 1))
    short="$(git -C "$REPO" rev-parse --short "$sha")"
    subject="$(git -C "$REPO" log -1 --format=%s "$sha")"
    printf '%s[%d/%d]%s %s %s\n' "$BLUE" "$INDEX" "$COUNT" "$NC" "$short" "$subject"

    if ! git -C "$TREE" checkout --quiet --detach -f "$sha"; then
        echo -e "  ${RED}checkout FAILED${NC}"
        RESULTS[$short]="checkout-FAIL"
        FAILURES+=("$short|$subject|checkout")
        [[ "$STOP_ON_FAIL" -eq 1 ]] && break
        continue
    fi

    commit_failed=0
    for check in "${CHECKS[@]}"; do
        log="$LOG_DIR/$short-$check.log"
        t0=$(date +%s)
        if (cd "$TREE" && run_check "$check" "$sha") >"$log" 2>&1; then
            printf '  %-10s %sOK%s\n' "$check" "$GREEN" "$NC"
            RESULTS[$short]+=" $check:OK"
        else
            dt=$(( $(date +%s) - t0 ))
            printf '  %-10s %sFAIL%s (%ss, log: %s)\n' "$check" "$RED" "$NC" "$dt" "$log"
            if [[ -n "$(tail -n 3 "$log" 2>/dev/null)" ]]; then
                tail -n 3 "$log" 2>/dev/null | sed 's/^/      | /'
            fi
            RESULTS[$short]+=" $check:FAIL"
            FAILURES+=("$short|$subject|$check")
            commit_failed=1
        fi
    done

    if [[ "$commit_failed" -eq 1 && "$STOP_ON_FAIL" -eq 1 ]]; then
        echo -e "${YELLOW}stopping at the first failing commit (--stop-on-fail)${NC}"
        break
    fi
    echo
done

ELAPSED=$(( $(date +%s) - START_TS ))
FAIL_COUNT="${#FAILURES[@]}"

echo -e "${BLUE}=== SUMMARY ===${NC}"
echo "  checked : $INDEX/$COUNT commit(s), ${CHECKS[*]}"
echo "  duration: ${ELAPSED}s"

if [[ "$FAIL_COUNT" -eq 0 ]]; then
    echo -e "  result  : ${GREEN}every commit passes every check${NC}"
    exit 0
fi

echo -e "  result  : ${RED}${FAIL_COUNT} failing check(s)${NC}"
echo
echo -e "${RED}Failing commits:${NC}"
for entry in "${FAILURES[@]}"; do
    IFS='|' read -r s sub check <<< "$entry"
    printf '  %s  %-10s %s\n' "$s" "$check" "$sub"
done
echo
echo "Logs per commit and check: $LOG_DIR"
KEEP_WORKTREE=1
exit 1
