#!/usr/bin/env bash
set -euo pipefail
build_dir="$(realpath -m "$1")"
project_name="$2"
sdkconfig="$(realpath "$3")"
source "$4"
target="$5"
[[ "$MAX_FIRMWARE_BYTES" =~ ^(0|[1-9][0-9]*)$ ]]
report="$build_dir/reports/memory"
mkdir -p "$report"
test -s "$build_dir/$project_name.map"
for command in size size-components size-files; do
    echo "===== $command ====="
    idf.py -B "$build_dir" -D "SDKCONFIG=$sdkconfig" \
        -D "IDF_TARGET=$target" "$command" | tee "$report/$command.txt"
done
test -s "$build_dir/$project_name.bin"
bytes=$(stat -c %s "$build_dir/$project_name.bin")
status=PASS
if (( MAX_FIRMWARE_BYTES == 0 )); then
    status=DISABLED
elif (( bytes > MAX_FIRMWARE_BYTES )); then
    status=FAIL
fi
printf 'Firmware bytes: %s\nBudget bytes: %s\nResult: %s\n' \
    "$bytes" "$MAX_FIRMWARE_BYTES" "$status" | tee "$report/memory-check.txt"
[[ "$status" != FAIL ]]
