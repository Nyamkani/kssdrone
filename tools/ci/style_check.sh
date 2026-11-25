#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/../.."
source config/ci/analysis.env
mkdir -p ci-reports/style
clang-format --version | tee ci-reports/style/tool-version.txt
: > ci-reports/style/format.txt
count=0
while IFS= read -r -d '' file; do
    if ! clang-format --dry-run --Werror --style="file:$PWD/config/ci/clang-format.yml" "$file" \
        >> ci-reports/style/format.txt 2>&1; then
        count=$((count + 1))
    fi
done < <(find main -type f \( -name '*.cpp' -o -name '*.c' -o -name '*.h' \) -print0)
printf 'Files with formatting diagnostics: %s\n' "$count" | tee ci-reports/style/summary.txt
if (( count > 0 )); then
    echo "::warning::Formatting diagnostics found; see report"
    if (( STYLE_STRICT != 0 )); then exit 1; fi
fi
