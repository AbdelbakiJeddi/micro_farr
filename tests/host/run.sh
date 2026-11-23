#!/usr/bin/env bash
# Build and run the host unit tests for the pure (ESP-IDF free) modules.
# Each tests/host/test_*.c lists the sources (and extra flags) it needs on its
# first line:  // SRCS: main/params.c
set -u
cd "$(dirname "$0")/../.."

out=build_host
mkdir -p "$out"
fail=0

for test in tests/host/test_*.c; do
    name=$(basename "$test" .c)
    srcs=$(sed -n '1s|^// SRCS:||p' "$test")
    # shellcheck disable=SC2086  # srcs is a word list on purpose
    if ! gcc -std=c11 -Wall -Werror -I main -I main/app -I main/tasks/motion -I tests/host \
            -o "$out/$name" "$test" $srcs -lm; then
        echo "BUILD FAILED: $name"
        fail=1
        continue
    fi
    if ! "$out/$name"; then
        echo "FAILED: $name"
        fail=1
    fi
done

if [ "$fail" -eq 0 ]; then
    echo "ALL HOST TESTS PASSED"
fi
exit "$fail"
