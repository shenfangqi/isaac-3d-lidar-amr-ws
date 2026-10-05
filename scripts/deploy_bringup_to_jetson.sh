#!/usr/bin/env bash
# Sync isaac_3d_lidar_bringup and its Jetson container script to the Jetson
# workspace, rebuild it in the navigation image, and verify the deployed
# version.  The default is a read-only preview; nothing changes without
# --apply.  This script never starts navigation or publishes velocity.

set -Eeuo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
workspace="$(cd -- "${script_dir}/.." && pwd)"

jetson_host="${CARBOT_JETSON_HOST:-isaac-jetson}"
jetson_fallback_ip="${CARBOT_JETSON_FALLBACK_IP:-192.168.1.109}"
jetson_identity_file="${CARBOT_JETSON_IDENTITY_FILE:-${HOME}/.ssh/id_ed25519_isaac_jetson}"
jetson_workspace="${CARBOT_JETSON_WORKSPACE:-/home/shenfq/Projects/isaac_ros-dev}"
livox_workspace="/home/shenfq/Projects/lidar-mid360/ws_livox"
image_name="carbot-isaac-ros-nvblox:3.3-lio"
container_name="carbot-nvblox"
package="isaac_3d_lidar_bringup"
package_dir="src/${package}"
record_path="${jetson_workspace}/.carbot_deploy/${package}.json"

mode=preview
allow_dirty=false
overwrite_local_edits=false
skip_blocked=false

usage() {
  cat <<EOF
Usage: $0 [--check|--apply] [--allow-dirty] [--skip-blocked|--overwrite-local-edits]

Deploy ${package_dir} and scripts/jetson_nvblox_container.sh to
${jetson_workspace} on the Jetson and rebuild ${package}.

  (default)      Preview: identity, layout, install mode, container state and
                 a per-file hash comparison. Changes nothing.
  --check        Version check only. Exit 0 when the Jetson source and install
                 match this checkout, 3 when they differ.
  --apply        Sync, rebuild in ${image_name}, then verify and record.
                 Refused while ${container_name} is running.
  --allow-dirty  Permit --apply with uncommitted changes in the deployed paths;
                 the record then marks the deployment as dirty.
  --skip-blocked Deploy only BEHIND/NEW files and leave every blocked
                 (OTHER_BRANCH/LOCAL_EDIT) Jetson file untouched; verification
                 then excludes them and the record lists them.
  --overwrite-local-edits
                 Permit --apply to replace blocked Jetson files as well.

Every differing Jetson file is classified against git history: BEHIND
(matches an ancestor of HEAD, safe to update), OTHER_BRANCH (matches a commit
that HEAD does not contain, so deploying would revert that branch's work) or
LOCAL_EDIT (matches no commit).  OTHER_BRANCH and LOCAL_EDIT block --apply.

Environment: CARBOT_JETSON_HOST, CARBOT_JETSON_FALLBACK_IP,
CARBOT_JETSON_IDENTITY_FILE, CARBOT_JETSON_WORKSPACE.
EOF
}

while (( $# > 0 )); do
  case "$1" in
    --check) mode=check ;;
    --apply) mode=apply ;;
    --allow-dirty) allow_dirty=true ;;
    --overwrite-local-edits) overwrite_local_edits=true ;;
    --skip-blocked) skip_blocked=true ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown option: $1" >&2; usage >&2; exit 2 ;;
  esac
  shift
done

require_command() {
  command -v "$1" >/dev/null 2>&1 || {
    echo "Required command not found: $1" >&2
    exit 1
  }
}

require_command git
require_command ssh
require_command sha256sum
if [[ "${mode}" == "apply" ]]; then
  require_command tar
  require_command flock
fi

ssh_options=(-o BatchMode=yes -o ConnectTimeout=5)
if [[ -r "${jetson_identity_file}" ]]; then
  ssh_options+=(-i "${jetson_identity_file}")
fi

remote() {
  local command=""
  local argument quoted
  for argument in "$@"; do
    printf -v quoted '%q' "${argument}"
    command+="${quoted} "
  done
  ssh "${ssh_options[@]}" "${jetson_host}" "${command% }"
}

# --- Local state -----------------------------------------------------------

cd "${workspace}"
mapfile -t files < <(
  git ls-files -- "${package_dir}" scripts/jetson_nvblox_container.sh | sort)
if (( ${#files[@]} == 0 )); then
  echo "No tracked files found under ${package_dir}." >&2
  exit 1
fi
local_commit="$(git rev-parse HEAD)"
local_branch="$(git rev-parse --abbrev-ref HEAD)"
dirty_status="$(git status --porcelain -- "${package_dir}" scripts/jetson_nvblox_container.sh)"
dirty=false
if [[ -n "${dirty_status}" ]]; then
  dirty=true
fi

declare -A local_hash=()
while read -r digest path; do
  local_hash["${path}"]="${digest}"
done < <(sha256sum -- "${files[@]}")

# Installed counterparts that must match the source after a build.
installed_pairs=()
for path in "${files[@]}"; do
  relative="${path#"${package_dir}"/}"
  case "${relative}" in
    "${package}"/*.py)
      installed_pairs+=("${path}|install/${package}/lib/python3*/site-packages/${relative}")
      ;;
    launch/*.launch.py|config/*/*.yaml)
      installed_pairs+=("${path}|install/${package}/share/${package}/${relative}")
      ;;
  esac
done

# --- Jetson connection -----------------------------------------------------

if ! remote true >/dev/null 2>&1; then
  if [[ -n "${CARBOT_JETSON_HOST+x}" ]]; then
    echo "Cannot reach explicitly configured Jetson host: ${jetson_host}" >&2
    exit 1
  fi
  [[ -r "${jetson_identity_file}" ]] || {
    echo "Jetson mDNS failed and fallback key is unreadable: ${jetson_identity_file}" >&2
    exit 1
  }
  echo "Warning: ${jetson_host} is unreachable; trying verified fallback ${jetson_fallback_ip}." >&2
  jetson_host="${jetson_fallback_ip}"
fi
identity="$(remote bash -lc 'printf "%s|%s" "$(hostname)" "$(whoami)"')"
if [[ "${identity}" != "ubuntu|shenfq" ]]; then
  echo "Unexpected Jetson identity: ${identity}" >&2
  exit 1
fi
remote test -d "${jetson_workspace}/${package_dir}" || {
  echo "Jetson has no ${jetson_workspace}/${package_dir}; refusing to guess the layout." >&2
  exit 1
}
remote test -d "${jetson_workspace}/scripts" || {
  echo "Jetson has no ${jetson_workspace}/scripts." >&2
  exit 1
}

# Hash remote files listed on stdin (relative to the Jetson workspace).
# Missing files print "MISSING path".  Globs are expanded remotely.
remote_hashes() {
  ssh "${ssh_options[@]}" "${jetson_host}" bash -c "$(printf '%q' '
    cd "$1" || exit 1
    while IFS= read -r pattern; do
      matches=( $pattern )
      if [[ -e "${matches[0]}" ]]; then
        sha256sum -- "${matches[0]}"
      else
        printf "MISSING %s\n" "${pattern}"
      fi
    done')" _ "${jetson_workspace}"
}

compare_source() {
  source_diff=()
  declare -gA remote_source=()
  local digest path
  while read -r digest path; do
    remote_source["${path}"]="${digest}"
  done < <(printf '%s\n' "${files[@]}" | remote_hashes)
  for path in "${files[@]}"; do
    if [[ "${remote_source[${path}]:-MISSING}" != "${local_hash[${path}]}" ]]; then
      source_diff+=("${path}")
    fi
  done
}

compare_install() {
  install_diff=()
  local pair source target digest path
  declare -A remote_install=()
  while read -r digest path; do
    remote_install["${path}"]="${digest}"
  done < <(for pair in "${installed_pairs[@]}"; do
             printf '%s\n' "${pair#*|}"
           done | remote_hashes)
  for pair in "${installed_pairs[@]}"; do
    source="${pair%%|*}"
    target="${pair#*|}"
    # Remote sha256sum prints the expanded path; match by suffix.
    local found=MISSING key
    for key in "${!remote_install[@]}"; do
      if [[ "${key}" == ${target} ]]; then
        found="${remote_install[${key}]}"
      fi
    done
    if [[ "${found}" != "${local_hash[${source}]}" ]]; then
      install_diff+=("${target}")
    fi
  done
}

# Classify each differing source file by searching its git history for the
# exact content now on the Jetson.  Sets classification[path] and
# local_edits (paths that match no committed version).
classify_source_diff() {
  declare -gA classification=()
  local_edits=()
  local path remote_digest commit digest found
  for path in "${source_diff[@]}"; do
    remote_digest="${remote_source[${path}]:-MISSING}"
    if [[ "${remote_digest}" == "MISSING" ]]; then
      classification["${path}"]="NEW (absent on the Jetson)"
      continue
    fi
    found=""
    while read -r commit; do
      digest="$(git show "${commit}:${path}" 2>/dev/null | sha256sum | cut -d' ' -f1)"
      if [[ "${digest}" == "${remote_digest}" ]]; then
        found="${commit}"
        break
      fi
    done < <(git rev-list --all -- "${path}")
    if [[ -n "${found}" ]] && git merge-base --is-ancestor "${found}" HEAD; then
      classification["${path}"]="BEHIND (matches $(git log -1 --format='%h %cs %s' "${found}" | cut -c1-70))"
    elif [[ -n "${found}" ]]; then
      local branches
      branches="$(git branch -a --contains "${found}" --format='%(refname:short)' | head -3 | paste -sd, -)"
      classification["${path}"]="OTHER_BRANCH (matches $(git log -1 --format='%h' "${found}") on ${branches:-no branch}; not in HEAD)"
      local_edits+=("${path}")
    else
      classification["${path}"]="LOCAL_EDIT (matches no committed version)"
      local_edits+=("${path}")
    fi
  done
}

install_mode() {
  if remote bash -c "test -L ${jetson_workspace}/install/${package}/lib/python3*/site-packages/${package}/automatic_localization_manager.py" 2>/dev/null; then
    echo symlink
  elif remote bash -c "test -e ${jetson_workspace}/install/${package}/lib/python3*/site-packages/${package}/automatic_localization_manager.py" 2>/dev/null; then
    echo copy
  else
    echo absent
  fi
}

container_running() {
  [[ "$(remote docker inspect -f '{{.State.Running}}' "${container_name}" 2>/dev/null || true)" == "true" ]]
}

report() {
  echo "Local:     ${local_branch} ${local_commit:0:12}$( [[ "${dirty}" == true ]] && echo ' (uncommitted changes in deployed paths)')"
  echo "Jetson:    ${identity} (${jetson_host}) ${jetson_workspace}"
  echo "Install:   ${current_install_mode}"
  echo "Container: ${container_name} $(container_running && echo running || echo stopped)"
  if remote test -f "${record_path}" 2>/dev/null; then
    echo "Last deploy record: $(remote cat "${record_path}" | tr -d '\n')"
  else
    echo "Last deploy record: none"
  fi
  echo "Source files differing: ${#source_diff[@]} of ${#files[@]}"
  local path
  for path in "${source_diff[@]}"; do
    printf '  %-80s %s\n' "${path}" "${classification[${path}]:-}"
  done
  if (( ${#local_edits[@]} > 0 )); then
    echo "WARNING: ${#local_edits[@]} Jetson file(s) contain work that HEAD does not have."
    echo "  Inspect them before deploying, e.g.:"
    echo "  ssh ${jetson_host} cat ${jetson_workspace}/${local_edits[0]} | diff -u ${local_edits[0]} -"
  fi
  echo "Installed files differing: ${#install_diff[@]} of ${#installed_pairs[@]}"
  if (( ${#install_diff[@]} > 0 )); then
    printf '  %s\n' "${install_diff[@]}"
  fi
}

current_install_mode="$(install_mode)"
compare_source
classify_source_diff
compare_install

if [[ "${mode}" != "apply" ]]; then
  report
  if (( ${#source_diff[@]} == 0 && ${#install_diff[@]} == 0 )); then
    echo "IN_SYNC: the Jetson runs this checkout of ${package}."
    exit 0
  fi
  echo "OUT_OF_SYNC: run '$0 --apply' after stopping navigation."
  [[ "${mode}" == "check" ]] && exit 3
  exit 0
fi

# --- Apply -----------------------------------------------------------------

if [[ "${skip_blocked}" == "true" && "${overwrite_local_edits}" == "true" ]]; then
  echo "--skip-blocked and --overwrite-local-edits are mutually exclusive." >&2
  exit 2
fi
blocked=()
if [[ "${skip_blocked}" == "true" ]]; then
  blocked=("${local_edits[@]}")
fi
if (( ${#local_edits[@]} > 0 )) && [[ "${overwrite_local_edits}" != "true" \
    && "${skip_blocked}" != "true" ]]; then
  echo "Refusing to overwrite Jetson work that HEAD does not contain:" >&2
  printf '  %s\n' "${local_edits[@]}" >&2
  echo "Bring it into this branch first, or pass --overwrite-local-edits." >&2
  exit 1
fi
if [[ "${dirty}" == "true" && "${allow_dirty}" != "true" ]]; then
  echo "Uncommitted changes in deployed paths; commit them or pass --allow-dirty:" >&2
  printf '%s\n' "${dirty_status}" >&2
  exit 1
fi
# Share the navigation startup lock so a deploy can never race a startup.
navigation_lock="${XDG_RUNTIME_DIR:-/tmp}/carbot-real-navigation-start.lock"
exec {navigation_lock_fd}>"${navigation_lock}"
if ! flock -n "${navigation_lock_fd}"; then
  echo "A real-navigation startup is running; finish or stop it first." >&2
  exit 1
fi
if container_running; then
  echo "${container_name} is running. Stop navigation first: ./stop_real_nav.sh" >&2
  exit 1
fi

is_blocked() {
  local item
  for item in "${blocked[@]}"; do
    [[ "${item}" == "$1" ]] && return 0
  done
  return 1
}
to_sync=()
for path in "${source_diff[@]}"; do
  is_blocked "${path}" || to_sync+=("${path}")
done
if (( ${#blocked[@]} > 0 )); then
  echo "Leaving ${#blocked[@]} blocked Jetson file(s) untouched:"
  printf '  %s\n' "${blocked[@]}"
fi
echo "[1/3] Syncing ${#to_sync[@]} changed tracked files..."
if (( ${#to_sync[@]} > 0 )); then
  printf '  %s\n' "${to_sync[@]}"
  # Only tracked files whose hash differs; nothing on the Jetson is deleted.
  tar -cf - -- "${to_sync[@]}" \
    | remote tar -xf - -C "${jetson_workspace}"
fi

build_flags=""
if [[ "${current_install_mode}" == "symlink" ]]; then
  build_flags="--symlink-install"
fi
echo "[2/3] Building ${package} in ${image_name} (${current_install_mode} install)..."
# Same user and entrypoint as jetson_nvblox_container.sh, so the build never
# leaves root-owned files in the shared workspace.
remote docker run --rm --network host \
  --user 1000:1000 \
  -e HOME=/tmp/carbot-deploy-home \
  -v "${jetson_workspace}:/workspaces/isaac_ros-dev" \
  -v "${livox_workspace}:${livox_workspace}:ro" \
  -v /tmp:/tmp \
  --entrypoint /bin/bash \
  "${image_name}" -lc "
    set -e
    mkdir -p \"\${HOME}\"
    source /opt/ros/humble/setup.bash
    source ${livox_workspace}/install/setup.bash
    cd /workspaces/isaac_ros-dev
    colcon build ${build_flags} --packages-select ${package}"

echo "[3/3] Verifying the deployed version..."
current_install_mode="$(install_mode)"
compare_source
classify_source_diff
compare_install
report
# Blocked files (and their installed copies) were deliberately left alone.
remaining=()
for path in "${source_diff[@]}"; do
  is_blocked "${path}" || remaining+=("${path}")
done
for pair in "${installed_pairs[@]}"; do
  if is_blocked "${pair%%|*}"; then
    continue
  fi
  for target in "${install_diff[@]}"; do
    [[ "${target}" == "${pair#*|}" ]] && remaining+=("${target}")
  done
done
if (( ${#remaining[@]} != 0 )); then
  echo "DEPLOY_FAILED: these Jetson files do not match this checkout after the build:" >&2
  printf '  %s\n' "${remaining[@]}" >&2
  exit 1
fi
skipped_json=""
if (( ${#blocked[@]} > 0 )); then
  skipped_json="$(printf '"%s",' "${blocked[@]}")"
fi
record="$(printf '{"package":"%s","commit":"%s","branch":"%s","dirty":%s,"deployed_unix":%s,"install":"%s","files":%s,"skipped_blocked":[%s]}' \
  "${package}" "${local_commit}" "${local_branch}" "${dirty}" "$(date +%s)" \
  "${current_install_mode}" "${#files[@]}" "${skipped_json%,}")"
remote bash -c "mkdir -p '$(dirname "${record_path}")' && cat > '${record_path}'" <<<"${record}"
echo "DEPLOYED: ${package} ${local_commit:0:12}$( [[ "${dirty}" == true ]] && echo ' (dirty)') is built and verified on the Jetson$( (( ${#blocked[@]} > 0 )) && echo ", except ${#blocked[@]} blocked file(s) left untouched")."
