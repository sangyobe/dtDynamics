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

echo "=== clang-format auto-fix ==="
echo "대상: ${SEARCH_DIRS[*]}"

find "${SEARCH_DIRS[@]}" \
    -type f \
    \( -name "*.cpp" -o -name "*.h" \) \
    -print0 \
| xargs -0 -r clang-format -i

# CI 러너의 checkout은 Job 종료 후 버려지므로, 수정 결과를 patch로 남겨
# artifacts로 내려받아 로컬 저장소에 반영한다.
git diff -- "${SEARCH_DIRS[@]}" > reports/clang-format.patch
git diff --name-only -- "${SEARCH_DIRS[@]}" > reports/clang-format-changed-files.txt

if [[ -s reports/clang-format.patch ]]; then
    echo "clang-format 자동 수정으로 변경된 파일:"
    cat reports/clang-format-changed-files.txt
    echo ""

    # 프로젝트 전용 봇 계정 토큰(GIT_PUSH_TOKEN)이 있으면 수정 커밋을 직접 push하고,
    # 없거나(non-protected 브랜치 등) push에 실패하면 patch artifact를 생성하여 다운받아 반영한다.
    if [[ -n "${GIT_PUSH_TOKEN:-}" && -n "${CI_SERVER_HOST:-}" && -n "${CI_PROJECT_PATH:-}" && -n "${CI_COMMIT_REF_NAME:-}" ]]; then
        COMMIT_AUTHOR_EMAIL=$(git log -1 --pretty=%ae)
        if [[ "${COMMIT_AUTHOR_EMAIL}" == "ci-clang-format-bot@noreply.hmc.co.kr" ]]; then
            echo "봇이 만든 커밋에서도 포맷 위반이 남아있습니다 — 자동 재귀 방지를 위해 자동 push/trigger를 건너뜁니다."
        else
            git config user.name "ci-clang-format-bot"
            git config user.email "ci-clang-format-bot@noreply.hmc.co.kr"
            git add -- "${SEARCH_DIRS[@]}"
            git commit -m "chore(lint): clang-format 자동 수정."

            if git push "https://oauth2:${GIT_PUSH_TOKEN}@${CI_SERVER_HOST}/${CI_PROJECT_PATH}.git" "HEAD:${CI_COMMIT_REF_NAME}"; then
                echo "자동 수정 커밋을 ${CI_COMMIT_REF_NAME} 브랜치에 push했습니다."

                # push된 커밋을 build/polyspace까지 검사받도록 새 파이프라인을 API로 트리거한다.
                # GIT_PUSH_TOKEN은 write_repository + api scope를 모두 가진 Project Access Token이어야 한다.
                TRIGGER_HTTP_CODE=$(curl --silent --output /dev/null --write-out "%{http_code}" \
                    --request POST \
                    --header "PRIVATE-TOKEN: ${GIT_PUSH_TOKEN}" \
                    --form "ref=${CI_COMMIT_REF_NAME}" \
                    "https://${CI_SERVER_HOST}/api/v4/projects/${CI_PROJECT_ID}/pipeline")

                if [[ "${TRIGGER_HTTP_CODE}" == "201" ]]; then
                    echo "수정된 커밋으로 새 파이프라인을 트리거했습니다."
                else
                    echo "파이프라인 자동 트리거에 실패했습니다 (HTTP ${TRIGGER_HTTP_CODE}) — 필요 시 수동으로 새 Pipeline을 실행하세요."
                fi
            else
                echo "push에 실패했습니다 (non-fast-forward 등) — reports/clang-format.patch 를 artifacts에서 내려받아 'git apply reports/clang-format.patch'로 반영하세요."
            fi
        fi
    else
        echo "GIT_PUSH_TOKEN 또는 GitLab CI 환경변수가 없습니다 — reports/clang-format.patch 를 artifacts에서 내려받아 'git apply reports/clang-format.patch'로 반영하세요."
    fi
else
    echo "clang-format 검사 완료: 수정할 내용 없음"
fi

exit 0
