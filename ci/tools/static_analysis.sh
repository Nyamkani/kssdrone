#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/../.."
source ci/config/analysis.env
mkdir -p ci-reports/static
cppcheck --version | tee ci-reports/static/tool-version.txt
# Source-only baseline: SDK headers and full target compile flags are not imported.
rc=0
cppcheck --enable=warning,performance,portability --std=c++17 \
    --language=c++ -I main --suppress=missingIncludeSystem \
    --error-exitcode=2 --xml --xml-version=2 main \
    > ci-reports/static/stdout.txt 2> ci-reports/static/cppcheck.xml || rc=$?
python3 - <<'PY'
import xml.etree.ElementTree as ET
from collections import Counter
from pathlib import Path
root = ET.parse("ci-reports/static/cppcheck.xml").getroot()
issues = root.findall("./errors/error")
counts = Counter(e.get("severity", "unknown") for e in issues)
out = "\n".join([f"Total findings: {len(issues)}"] + [f"{k}: {v}" for k,v in sorted(counts.items())])
print(out)
Path("ci-reports/static/summary.txt").write_text(out + "\n")
PY
if (( rc == 2 && STATIC_STRICT == 0 )); then
    echo "::warning::Cppcheck baseline findings; report-only mode"
elif (( rc != 0 )); then
    exit "$rc"
fi
