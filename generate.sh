#!/bin/bash

# 获取系统版本参数，默认为 jammy
OS_VERSION=${1:-jammy}
ROS_DISTRO=${2:-noetic}

# 导出 ROSDISTRO 索引 URL 以确保 bloom/rosdep 使用正确的源
export ROSDISTRO_INDEX_URL=https://github.com/tianbot/rosdistro/raw/master/index-v4.yaml

# 遍历目录查找 package.xml，排除隐藏目录（如 .git, .pc）
find . -type d \( -path "*/.*" \) -prune -o -type f -name 'package.xml' -print | while read -r package_xml; do
    # 获取包含 package.xml 的目录路径
    dir=$(dirname "$package_xml")
    
    echo "在目录 $dir 中发现了 package.xml，执行 bloom-generate 命令 (OS: $OS_VERSION, Distro: $ROS_DISTRO)..."
    
    # 执行 bloom-generate 命令
    (cd "$dir" && bloom-generate rosdebian --os-name ubuntu --os-version "$OS_VERSION" --ros-distro "$ROS_DISTRO")
done
