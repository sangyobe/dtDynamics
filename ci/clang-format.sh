#!/usr/bin/env bash
# clang-format 로컬 사전 점검 스크립트
#
# 사용법:
#   bash ci/clang-format.sh          # 검사만 (CI와 동일, dry-run)
#   bash ci/clang-format.sh --fix    # 위반 파일 자동 수정

set -euo pipefail

FIX_MODE=0
if [ "${1:-}" = "--fix" ]; then
  FIX_MODE=1
fi

# CI lint-job과 동일한 파일 목록 수집
REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$REPO_ROOT"

FILES=$(find include src \
  \( -name "*.cpp" -o -name "*.h" \) \
  | sort)

FILE_COUNT=$(echo "$FILES" | grep -c . || true)
echo "검사 대상: ${FILE_COUNT}개 파일"

if [ "$FIX_MODE" -eq 1 ]; then
  echo "$FILES" | xargs clang-format -i
  echo ""
  echo "자동 수정 완료. git diff 로 변경 내용을 확인하세요."
else
  # dry-run: 위반 파일 목록 수집
  VIOLATIONS=$(echo "$FILES" | xargs clang-format --dry-run --Werror 2>&1 || true)

  REPORT_DIR="$REPO_ROOT/ci/reports"
  REPORT_FILE="$REPORT_DIR/clang-format-local.txt"
  mkdir -p "$REPORT_DIR"

  if [ -z "$VIOLATIONS" ]; then
    echo "OK: clang-format 검사 통과"
    rm -f "$REPORT_FILE"
    exit 0
  else
    echo "$VIOLATIONS" > "$REPORT_FILE"
    echo ""
    echo "FAIL: clang-format 규칙에 맞지 않는 파일이 있습니다. (터미널 줄 제한으로 전체 목록은 파일 참고)"
    echo "전체 목록: $REPORT_FILE"
    echo ""
    echo "자동 수정하려면: bash ci/clang-format.sh --fix"
    exit 1
  fi
fi
