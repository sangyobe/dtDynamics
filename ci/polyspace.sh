#!/usr/bin/env bash
set -uo pipefail

: "${POLYSPACE_PROJECT_NAME:?POLYSPACE_PROJECT_NAME is not set}"
: "${POLYSPACE_ACCESS_PARENT:?POLYSPACE_ACCESS_PARENT is not set}"
: "${POLYSPACE_BIN:?POLYSPACE_BIN is not set}"
: "${POLYSPACE_TEMPLATES:?POLYSPACE_TEMPLATES is not set}"
: "${POLYSPACE_ACCESS_HOST:?POLYSPACE_ACCESS_HOST is not set}"
: "${POLYSPACE_ACCESS_PORT:?POLYSPACE_ACCESS_PORT is not set}"

export PATH="${POLYSPACE_BIN}:${PATH}"

# GitLab Runner locale 차이로 인한 로그/grep 문제 방지
export LANG=en_US.UTF-8
export LC_ALL=en_US.UTF-8

BUILD_DIR="build"
PROG="${POLYSPACE_PROJECT_NAME}"

REPORT_ROOT="reports/polyspace"
RESULTS_DIR="${REPORT_ROOT}/results"
POLYSPACE_BUILD_DIR="build-polyspace"
CONFIG_OPTS="${POLYSPACE_BUILD_DIR}/configure.opts"
REPORT="${REPORT_ROOT}/polyspace/result_list.txt"
ACCESS_OUT="${REPORT_ROOT}/access-upload.txt"

ACCESS_PARENT="${POLYSPACE_ACCESS_PARENT}"
ACCESS_PROJECT="${POLYSPACE_PROJECT_NAME}_${CI_COMMIT_REF_SLUG}"

RUN_LABEL="${CI_COMMIT_SHORT_SHA} | pipeline #${CI_PIPELINE_IID} | $(date '+%Y-%m-%d %H:%M')"

echo "=================================================="
echo "Branch               : ${CI_COMMIT_REF_NAME}"
echo "Commit               : ${CI_COMMIT_SHA}"
echo "Pipeline             : #${CI_PIPELINE_IID}"
echo "Access parent        : ${ACCESS_PARENT}"
echo "Access project       : ${ACCESS_PROJECT}"
echo "Access run label     : ${RUN_LABEL}"
echo "=================================================="

# 바이너리/라이선스 서버 연결을 초반에 확인
if ! polyspace-bug-finder-server -version; then
    echo "ERROR: Polyspace Bug Finder Server is unavailable"
    exit 1
fi

if ! mkdir -p "${REPORT_ROOT}" "${POLYSPACE_BUILD_DIR}"; then
    echo "ERROR: cannot create Polyspace report directories"
    exit 1
fi

# cache(policy: pull-push, key가 브랜치 단위)가 직전 실행의 -results-dir를 그대로 복원해오므로,
# 여기서 비우지 않으면 "풀스캔"이어도 이전 실행 결과가 섞인 상태로 재검증되어 실행마다 결함 집합이 달라진다
if ! rm -rf "${RESULTS_DIR}"; then
    echo "ERROR: cannot clear prior Polyspace results"
    exit 1
fi
if ! mkdir -p "${RESULTS_DIR}"; then
    echo "ERROR: cannot create fresh Polyspace results directory"
    exit 1
fi

# compile_commands.json → Polyspace 분석 옵션 추출
if ! rm -f "${CONFIG_OPTS}"; then
    echo "ERROR: cannot clear prior Polyspace configuration"
    exit 1
fi
if ! polyspace-configure \
    -compilation-database "${BUILD_DIR}/compile_commands.json" \
    -output-options-file "${CONFIG_OPTS}" \
    -allow-overwrite \
    -prog "${PROG}"
then
    echo "ERROR: Polyspace configuration failed"
    exit 1
fi
test -s "${CONFIG_OPTS}" || {
    echo "ERROR: Polyspace configuration output is missing or empty"
    exit 1
}

ANALYSIS_EXIT=0
polyspace-bug-finder-server \
    -options-file polyspace/polyspace.opts \
    -options-file "${CONFIG_OPTS}" \
    -checkers-activation-file polyspace/checkers.xml \
    -results-dir "${RESULTS_DIR}" \
    -max-processes 4 || ANALYSIS_EXIT=$?

test -s "${RESULTS_DIR}/ps_results.psbf" || {
    echo "ERROR: Polyspace analysis failed: ps_results.psbf is missing or empty"
    exit 1
}
if [[ ${ANALYSIS_EXIT} -ne 0 ]]; then
    echo "ERROR: Polyspace analysis command failed (exit ${ANALYSIS_EXIT})"
    exit "${ANALYSIS_EXIT}"
fi

# CSV 결과 내보내기
if ! rm -f "${REPORT}"; then
    echo "ERROR: cannot clear prior Polyspace result report"
    exit 1
fi
if ! polyspace-results-export \
    -format csv \
    -output-name "${REPORT_ROOT}/polyspace" \
    -results-dir "${RESULTS_DIR}"
then
    echo "ERROR: Polyspace result export failed"
    exit 1
fi

# PDF 리포트 생성
polyspace-report-generator \
    -template "${POLYSPACE_TEMPLATES}/Developer.rpt" \
    -format PDF \
    -output-name "${REPORT_ROOT}/polyspace-report" \
    -results-dir "${RESULTS_DIR}" \
    -set-language-english || true

# 라이선스 오류 감지
PSLOG="$(ls "${RESULTS_DIR}"/Polyspace_*.log 2>/dev/null | head -1 || true)"
if [[ -n "${PSLOG}" ]] && grep -q "License error\|Licensing Error" "${PSLOG}"; then
    echo "ERROR: Polyspace 라이선스 오류 — 라이선스 서버(FlexLM) 연결 실패"
    grep "Licensing Error\|Error Code\|Feature:" "${PSLOG}" | head -5
    exit 1
fi

# CI passes only when every result is Green or carries a narrow Polyspace
# justification. Compilation failures always block the static-analysis gate.
POLYSPACE_EXIT=0
if [[ -f "${REPORT}" ]]; then
    echo "=============================="
    echo "Polyspace 결과 요약"
    python3 tools/polyspace_result_gate.py --input "${REPORT}" || POLYSPACE_EXIT=1
    echo "  PDF: ${REPORT_ROOT}/polyspace-report.pdf"
    echo "=============================="
else
    echo "ERROR: exported Polyspace result is missing: ${REPORT}"
    POLYSPACE_EXIT=1
fi

# Polyspace Access 에 결과 업로드 (변수 미설정 시 건너뜀)
if [[ -n "${POLYSPACE_ACCESS_LOGIN:-}" && -n "${POLYSPACE_ACCESS_ENCRYPTED_PASSWORD:-}" ]]; then
    if polyspace-access \
        -protocol https \
        -host "${POLYSPACE_ACCESS_HOST}" \
        -port "${POLYSPACE_ACCESS_PORT}" \
        -login "${POLYSPACE_ACCESS_LOGIN}" \
        -encrypted-password "${POLYSPACE_ACCESS_ENCRYPTED_PASSWORD}" \
        -upload "${RESULTS_DIR}" \
        -parent-project "${ACCESS_PARENT}" \
        -project "${ACCESS_PROJECT}" \
        -output "${ACCESS_OUT}"
    then
        RUN_ID="$(grep -oP '(?<=RUN_ID )\d+' "${ACCESS_OUT}" | tail -1 || true)"

        if [[ -n "${RUN_ID}" ]]; then
            polyspace-access \
                -protocol https \
                -host "${POLYSPACE_ACCESS_HOST}" \
                -port "${POLYSPACE_ACCESS_PORT}" \
                -login "${POLYSPACE_ACCESS_LOGIN}" \
                -encrypted-password "${POLYSPACE_ACCESS_ENCRYPTED_PASSWORD}" \
                -add-label "${RUN_LABEL}" \
                -run-id "${RUN_ID}"

            echo "업로드 완료: Run ${RUN_ID} / ${RUN_LABEL}"
        else
            echo "Polyspace Access RUN_ID 확인 실패"
        fi
    else
        echo "Polyspace Access 업로드 실패 (계속 진행)"
    fi
else
    echo "POLYSPACE_ACCESS_LOGIN/POLYSPACE_ACCESS_ENCRYPTED_PASSWORD 미설정 — 업로드 건너뜀"
fi

exit "${POLYSPACE_EXIT}"
