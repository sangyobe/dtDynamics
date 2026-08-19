#!/usr/bin/env bash
set -uo pipefail

mkdir -p reports

SEARCH_DIRS=()
for dir in include src; do
    if [[ -d "${dir}" ]]; then
        SEARCH_DIRS+=("${dir}")
    fi
done

if [[ ${#SEARCH_DIRS[@]} -eq 0 ]]; then
    echo "ERROR: clang-format 검사 대상 디렉터리(include/src)를 찾을 수 없습니다."
    exit 1
fi

echo "=== clang-format check ==="
echo "대상: ${SEARCH_DIRS[*]}"

# clang-format 오류는 exit code로 반환하고,
# .gitlab-ci.yml의 allow_failure 정책이 Pipeline 진행 여부를 결정한다.
find "${SEARCH_DIRS[@]}" \
    -type f \
    \( -name "*.cpp" -o -name "*.h" \) \
    -print0 \
| xargs -0 -r clang-format --dry-run --Werror 2>&1 \
| tee reports/clang-format.txt

FORMAT_EXIT=${PIPESTATUS[1]}

if [[ ${FORMAT_EXIT} -ne 0 ]]; then
    echo "clang-format 검사에서 오류가 발견되었습니다."
else
    echo "clang-format 검사 완료: 오류 없음"
fi

exit "${FORMAT_EXIT}"
