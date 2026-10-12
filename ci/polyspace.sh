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
polyspace-bug-finder-server -version

mkdir -p "${REPORT_ROOT}" "${POLYSPACE_BUILD_DIR}"

# cache(policy: pull-push, key가 브랜치 단위)가 직전 실행의 -results-dir를 그대로 복원해오므로,
# 여기서 비우지 않으면 "풀스캔"이어도 이전 실행 결과가 섞인 상태로 재검증되어 실행마다 결함 집합이 달라진다
rm -rf "${RESULTS_DIR}" || {
    echo "ERROR: cannot clear prior Polyspace results"
    exit 1
}
mkdir -p "${RESULTS_DIR}"

# compile_commands.json → Polyspace 분석 옵션 추출
polyspace-configure \
    -compilation-database "${BUILD_DIR}/compile_commands.json" \
    -output-options-file "${CONFIG_OPTS}" \
    -allow-overwrite \
    -prog "${PROG}" || {
    echo "ERROR: Polyspace configuration failed"
    exit 1
}

polyspace-bug-finder-server \
    -options-file polyspace/polyspace.opts \
    -options-file "${CONFIG_OPTS}" \
    -checkers-activation-file polyspace/checkers.xml \
    -results-dir "${RESULTS_DIR}" \
    -max-processes 4 || {
    echo "ERROR: Polyspace analysis command failed"
    exit 1
}

test -f "${RESULTS_DIR}/ps_results.psbf" || {
    echo "ERROR: Polyspace 분석 실패 - ps_results.psbf 결과가 없습니다."
    exit 1
}

# CSV 결과 내보내기 — 성공 exit만으로는 새 REPORT 생성을 보장하지 않는다.
# 직전 cache의 report를 release gate가 재사용하지 못하도록 export 전에 지우고,
# 새 canonical result_list.txt가 없으면 fail closed 한다.
rm -f "${REPORT}" || {
    echo "ERROR: cannot clear prior Polyspace result list"
    exit 1
}
polyspace-results-export \
    -format csv \
    -output-name "${REPORT_ROOT}/polyspace" \
    -results-dir "${RESULTS_DIR}" || {
    echo "ERROR: Polyspace result export failed"
    exit 1
}
test -s "${REPORT}" || {
    echo "ERROR: Polyspace export did not produce canonical result_list.txt"
    exit 1
}

# PDF report is advisory rather than a release decision source. Remove any cached
# copy first so a generator failure cannot publish stale human-readable evidence.
rm -f "${REPORT_ROOT}/polyspace-report.pdf" || {
    echo "ERROR: cannot clear prior Polyspace PDF report"
    exit 1
}
if ! polyspace-report-generator \
    -template "${POLYSPACE_TEMPLATES}/Developer.rpt" \
    -format PDF \
    -output-name "${REPORT_ROOT}/polyspace-report" \
    -results-dir "${RESULTS_DIR}" \
    -set-language-english
then
    echo "WARNING: Polyspace PDF report generation failed; canonical result_list.txt remains the gate evidence"
fi

# 라이선스 오류 감지
PSLOG="$(ls "${RESULTS_DIR}"/Polyspace_*.log 2>/dev/null | head -1 || true)"
if [[ -n "${PSLOG}" ]] && grep -q "License error\|Licensing Error" "${PSLOG}"; then
    echo "ERROR: Polyspace 라이선스 오류 — 라이선스 서버(FlexLM) 연결 실패"
    grep "Licensing Error\|Error Code\|Feature:" "${PSLOG}" | head -5
    exit 1
fi

# Canonical raw-defect-zero gate. It is the only release decision source: every row
# must be Green with a blank or Unreviewed status. The parser also fails closed on
# malformed TSV, missing values, and "File does not compile" rows.
python3 tools/polyspace_result_gate.py --input "${REPORT}"
POLYSPACE_EXIT=$?

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
