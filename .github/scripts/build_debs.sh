#!/usr/bin/env bash
set -euo pipefail

OS_VERSION="${1:-jammy}"
ROS_DISTRO="${2:-noetic}"

export ROSDISTRO_INDEX_URL="${ROSDISTRO_INDEX_URL:-https://github.com/tianbot/rosdistro/raw/master/index-v4.yaml}"
export DEBIAN_FRONTEND="${DEBIAN_FRONTEND:-noninteractive}"
export ROS_DISTRO="$ROS_DISTRO"

mkdir -p artifacts
export DEB_BUILD_OPTIONS="parallel=$(nproc) nocheck"

# 收集在逐个包安装阶段失败的 deb，最后统一尝试安装
FAILED_DEBS=()

rosdep update

chmod +x generate.sh
./generate.sh "$OS_VERSION" "$ROS_DISTRO"

echo "正在更新软件包列表..."
apt-get update

echo "正在安装系统依赖..."
PKG_PATHS=$(find . -maxdepth 4 -name "package.xml" | grep -vE "/\.|/(rosdep|rosdistro|rospkg|test|tests)/" | xargs -n1 dirname | sort -u)

rosdep install --from-paths $PKG_PATHS --ignore-src -r -y \
  --os=ubuntu:"$OS_VERSION" \
  --rosdistro "$ROS_DISTRO" \
  --skip-keys "python3-rospkg python3-catkin-pkg python3-rosdep python3-rosdistro opencv2" \
  || echo "部分系统依赖可能无法安装，尝试继续..."

ORDERED_PATHS=$(python3 -c "
import os
from catkin_pkg.packages import find_packages
from catkin_pkg.topological_order import topological_order_packages

try:
    pkgs = find_packages('.', exclude_paths=['rosdep', 'rosdistro', 'rospkg', 'test', 'tests'])
    ordered = topological_order_packages(pkgs)
    for path, pkg in ordered:
        if path != '.':
            print(path)
except Exception as e:
    import sys
    print(f'Error during topological sort: {e}', file=sys.stderr)
    sys.exit(1)
")

if [ $? -ne 0 ]; then
  echo "拓扑排序失败，请检查 package.xml 是否完整或路径是否正确。"
  exit 1
fi

BOOTSTRAP_LIST=$(cat .github/bootstrap_packages.txt)

is_bootstrap() {
  echo "$BOOTSTRAP_LIST" | grep -qx "$1"
}

build_one() {
  local pkg_path="$1"
  local phase="$2"
  local pkg_name
  local parent_dir
  pkg_name=$(basename "$pkg_path")
  parent_dir=$(dirname "$pkg_path")

  printf "%s 正在构建 %-30s ... " "$phase" "$pkg_name"
  local before_deb_list
  before_deb_list=$(find "$parent_dir" -maxdepth 1 -name "*.deb" -type f -printf "%f\n" | sort)
  local expected_version
  local expected_arch
  local expected_pkgs
  local expected_debs
  expected_version=$(cd "$pkg_path" && dpkg-parsechangelog -S Version 2>/dev/null || true)
  expected_arch=$(dpkg-architecture -qDEB_BUILD_ARCH 2>/dev/null || true)
  expected_pkgs=$(awk '/^Package: /{print $2}' "$pkg_path/debian/control" 2>/dev/null || true)
  expected_debs=""
  if [ -n "$expected_version" ] && [ -n "$expected_arch" ] && [ -n "$expected_pkgs" ]; then
    while read -r p; do
      if [ -n "$p" ]; then
        expected_debs="$expected_debs $p"_"$expected_version"_"$expected_arch".deb
      fi
    done <<< "$expected_pkgs"
  fi

  if (cd "$pkg_path" && dpkg-buildpackage -us -uc -b -d -jauto > build.log 2>&1); then
    echo "✅ 成功"
    echo "::group::查看构建日志：$pkg_name"
    cat "$pkg_path/build.log"
    echo "::endgroup::"

    local after_deb_list
    local new_debs
    after_deb_list=$(find "$parent_dir" -maxdepth 1 -name "*.deb" -type f -printf "%f\n" | sort)
    new_debs=$(comm -13 <(echo "$before_deb_list") <(echo "$after_deb_list"))
    if [ -z "$new_debs" ]; then
      BUILD_ONE_NEW_DEBS=""
      if [ -n "$expected_debs" ]; then
        echo "::warning title=Deb Not Generated::构建 $pkg_name 完成但未发现新增 deb。期望：$expected_debs"
      else
        echo "::warning title=Deb Not Generated::构建 $pkg_name 完成但未发现新增 deb。"
      fi
    else
      BUILD_ONE_NEW_DEBS="$new_debs"
      echo "$new_debs" | while read -r deb_name; do
        if [ -n "$deb_name" ]; then
          cp "$parent_dir/$deb_name" artifacts/
        fi
      done
    fi
    return 0
  else
    BUILD_ONE_NEW_DEBS=""
    echo "❌ 失败"
    echo "--------------------------------------------------"
    echo "构建失败日志: $pkg_name"
    cat "$pkg_path/build.log"
    echo "--------------------------------------------------"
    echo "::warning title=Build Failed::构建 $pkg_name 失败，尝试继续下一个包。"
    return 1
  fi
}

echo "开始构建并安装基础包..."
echo "$ORDERED_PATHS" | while read -r pkg_path; do
  if [ -z "$pkg_path" ]; then continue; fi
  if echo "$pkg_path" | grep -qE "/\.|/(rosdep|rosdistro|rospkg|test|tests)/"; then continue; fi
  if ! is_bootstrap "$pkg_path"; then continue; fi
  if [ -d "$pkg_path/debian" ]; then
    build_one "$pkg_path" "[bootstrap]"
  fi
done

if compgen -G "artifacts/*.deb" > /dev/null; then
  echo "基础包构建完成，开始统一安装 bootstrap deb 包..."
  if ! apt-get install -y ./artifacts/*.deb; then
    echo "::warning title=Install Failed::bootstrap 统一安装失败，稍后继续构建并在最终阶段重试。"
  fi
else
  echo "::warning title=No Debs Collected::未收集到任何 bootstrap deb 包，跳过安装。"
fi

if [ -f /opt/ros/noetic/setup.bash ]; then
  # shellcheck disable=SC1091
  source /opt/ros/noetic/setup.bash
else
  echo "::warning title=Setup Not Found::/opt/ros/noetic/setup.bash 不存在，环境未刷新。"
fi

echo "$ORDERED_PATHS" | while read -r pkg_path; do
  if [ -z "$pkg_path" ]; then continue; fi
  if echo "$pkg_path" | grep -qE "/\.|/(rosdep|rosdistro|rospkg|test|tests)/"; then continue; fi
  if is_bootstrap "$pkg_path"; then continue; fi
  if [ -d "$pkg_path/debian" ]; then
    build_one "$pkg_path" ""
  fi
done

all_debs=()
if compgen -G "artifacts/*.deb" > /dev/null; then
  for f in artifacts/*.deb; do
    all_debs+=("./$f")
  done
fi
if [ ${#FAILED_DEBS[@]} -gt 0 ]; then
  all_debs+=("${FAILED_DEBS[@]}")
fi

if [ ${#all_debs[@]} -gt 0 ]; then
  echo "开始安装收集到的 deb 包（包含先前失败的）..."
  if ! apt-get install -y "${all_debs[@]}"; then
    echo "::warning title=Install Failed::统一安装 deb 失败，尝试回退策略：先用 dpkg 安装本地 deb，再尝试修复依赖。失败的 deb 列表: ${FAILED_DEBS[*]:-none}"
    # 回退策略：先用 dpkg -i 安装本地 deb（可能产生未满足依赖），再用 apt-get -f install 试图从仓库拉取缺失依赖
    if dpkg -i "${all_debs[@]}"; then
      echo "部分本地 deb 已通过 dpkg 安装（或标记为半安装状态）。尝试修复依赖..."
    else
      echo "dpkg 安装本地 deb 过程中出现错误，继续尝试修复依赖..."
    fi
    if apt-get install -f -y; then
      echo "依赖修复完成。"
    else
      echo "::warning title=Fix Failed::自动修复依赖失败。建议：检查缺失包并添加相应 apt 源或构建缺失包。"
      echo "尝试显示模拟修复（apt-get -s install -f）以帮助诊断："
      apt-get -s install -f | sed -n '1,200p'
    fi
  fi
else
  echo "::warning title=No Debs Collected::未收集到任何 deb 包，跳过安装。"
fi

find . -name "*.deb" -type f -not -path "./artifacts/*" -exec cp {} artifacts/ \;
