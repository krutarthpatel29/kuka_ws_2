#!/usr/bin/env bash
set -euo pipefail

echo "repositories:"
# find every nested git repo under src/
while IFS= read -r -d '' gitdir; do
  repo_dir="$(dirname "$gitdir")"
  # name key in YAML: path relative to workspace root
  key="${repo_dir}"

  # try to get origin URL, branch, and pinned commit
  url="$(git -C "$repo_dir" config --get remote.origin.url || true)"
  branch="$(git -C "$repo_dir" rev-parse --abbrev-ref HEAD || true)"
  commit="$(git -C "$repo_dir" rev-parse HEAD || true)"

  # if no origin, skip (probably not a remote repo)
  if [ -z "$url" ]; then
    continue
  fi

  echo "  ${key}:"
  echo "    type: git"
  echo "    url: ${url}"
  # Use a pinned commit for reproducibility; comment the branch for readability
  echo "    version: ${commit}  # branch: ${branch}"
done < <(find src -type d -name .git -print0 | sort -z)
