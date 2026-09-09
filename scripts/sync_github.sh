#!/usr/bin/env bash
set -euo pipefail

usage() {
    cat <<'EOF'
Usage:
  ./scripts/sync_github.sh [options] "brief change description"

Options:
  -y, --yes       Commit and push without confirmation
  -a, --auto      Generate a description, then commit and push automatically
  -n, --dry-run   Only display changes; do not modify Git state
  -h, --help      Show this help

Examples:
  ./scripts/sync_github.sh "改进有限时域截击与目标偏航控制"
  ./scripts/sync_github.sh -y "补充相机与激光雷达接口文档"
  ./scripts/sync_github.sh --auto
EOF
}

confirm=true
dry_run=false
automatic_description=false
summary_parts=()

while (($# > 0)); do
    case "$1" in
        -y|--yes)
            confirm=false
            ;;
        -a|--auto)
            confirm=false
            automatic_description=true
            ;;
        -n|--dry-run)
            dry_run=true
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        --)
            shift
            summary_parts+=("$@")
            break
            ;;
        -*)
            echo "Unknown option: $1" >&2
            usage >&2
            exit 2
            ;;
        *)
            summary_parts+=("$1")
            ;;
    esac
    shift
done

script_dir="$(
    cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
    pwd
)"
workspace_root="$(git -C "${script_dir}/.." rev-parse --show-toplevel)"
cd "${workspace_root}"

remote_name="${UAV_USV_GIT_REMOTE:-github}"
if ! remote_url="$(git remote get-url "${remote_name}" 2>/dev/null)"; then
    echo "Git remote '${remote_name}' is not configured." >&2
    echo "Configure it with:" >&2
    echo "  git remote add ${remote_name} git@github.com:Qinjianheng/uav_usv.git" >&2
    exit 1
fi

branch="$(git branch --show-current)"
if [[ -z "${branch}" ]]; then
    echo "Detached HEAD is not supported. Switch to a branch first." >&2
    exit 1
fi

if [[ -z "$(git status --porcelain)" ]]; then
    echo "No changes to commit. ${remote_name}/${branch} remains unchanged."
    exit 0
fi

commit_summary="${summary_parts[*]:-}"
if [[ -z "${commit_summary//[[:space:]]/}" \
    && "${automatic_description}" == true ]]; then
    pending_file_count="$(git status --porcelain | wc -l)"
    pending_file_count="${pending_file_count//[[:space:]]/}"
    commit_summary="自动同步：更新 ${pending_file_count} 个文件 ($(date '+%Y-%m-%d %H:%M'))"
fi

if [[ -z "${commit_summary//[[:space:]]/}" ]]; then
    if [[ -t 0 ]]; then
        read -r -p "Briefly describe this change: " commit_summary
    else
        echo "A brief change description is required." >&2
        echo "Example: ./scripts/sync_github.sh \"改进截击规划\"" >&2
        exit 2
    fi
fi

if [[ -z "${commit_summary//[[:space:]]/}" ]]; then
    echo "The change description cannot be empty." >&2
    exit 2
fi

echo "Repository : ${workspace_root}"
echo "Destination: ${remote_name}/${branch} (${remote_url})"
echo "Commit     : ${commit_summary}"
echo
echo "Changes to synchronize:"
git status --short

if [[ "${dry_run}" == true ]]; then
    echo
    echo "Dry run complete. No files were staged, committed, or pushed."
    exit 0
fi

if [[ "${confirm}" == true && -t 0 ]]; then
    echo
    read -r -p "Commit and push all changes above? [Y/n] " answer
    case "${answer:-Y}" in
        Y|y|YES|Yes|yes)
            ;;
        *)
            echo "Synchronization cancelled."
            exit 0
            ;;
    esac
fi

git add -A

if git diff --cached --quiet; then
    echo "No staged changes remain after applying .gitignore rules."
    exit 0
fi

change_statistics="$(git diff --cached --shortstat)"
changed_files="$(git diff --cached --name-status | sed -n '1,20p')"
changed_file_count="$(git diff --cached --name-only | wc -l)"

commit_body="Change statistics: ${change_statistics}

Changed files (up to 20 of ${changed_file_count}):
${changed_files}"

git commit -m "${commit_summary}" -m "${commit_body}"

echo
echo "Pushing ${branch} to ${remote_name}..."
if ! git push --set-upstream "${remote_name}" "${branch}"; then
    echo >&2
    echo "Push failed, but the commit is safely stored locally." >&2
    echo "If the remote branch is ahead, review and run:" >&2
    echo "  git pull --rebase ${remote_name} ${branch}" >&2
    echo "  git push ${remote_name} ${branch}" >&2
    exit 1
fi

echo
echo "Synchronization complete:"
git show --stat --oneline --summary HEAD

if [[ -n "$(git status --porcelain)" ]]; then
    echo
    echo "Warning: the working tree still contains changes:"
    git status --short
fi
