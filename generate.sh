#!/bin/bash

# 获取系统版本参数，默认为 jammy
OS_VERSION=${1:-jammy}
ROS_DISTRO=${2:-noetic}

# 导出 ROSDISTRO 索引 URL
export ROSDISTRO_INDEX_URL=https://github.com/tianbot/rosdistro/raw/master/index-v4.yaml

echo "正在生成本地 rosdep 映射以解决内部依赖解析问题..."
# 创建本地 rosdep yaml 文件
LOCAL_ROSDEP_YAML="/tmp/local_rosdep.yaml"
rm -f "$LOCAL_ROSDEP_YAML"
touch "$LOCAL_ROSDEP_YAML"

# 遍历所有 package.xml 生成映射： pkg_name -> ros-noetic-pkg-name
find . -type d \( -path "*/.*" -o -path "./rosdep" -o -path "./rosdistro" -o -path "./rospkg" -o -name "test" -o -name "tests" \) -prune -o -type f -name 'package.xml' -print | while read -r pkg_xml; do
    pkg_name=$(grep -oPm1 "(?<=<name>)[^<]+" "$pkg_xml")
    deb_name="ros-noetic-$(echo "$pkg_name" | tr '_' '-')"
    echo "$pkg_name:" >> "$LOCAL_ROSDEP_YAML"
    echo "  ubuntu: [$deb_name]" >> "$LOCAL_ROSDEP_YAML"
done

# 将本地源添加到 rosdep
sudo sh -c "echo 'yaml file://$LOCAL_ROSDEP_YAML' > /etc/ros/rosdep/sources.list.d/10-local.list"
rosdep update

# 遍历目录执行 bloom-generate
find . -type d \( -path "*/.*" -o -path "./rosdep" -o -path "./rosdistro" -o -path "./rospkg" -o -name "test" -o -name "tests" \) -prune -o -type f -name 'package.xml' -print | while read -r package_xml; do
    dir=$(dirname "$package_xml")
    echo "在目录 $dir 中发现了 package.xml，执行 bloom-generate..."
    (cd "$dir" && bloom-generate rosdebian --os-name ubuntu --os-version "$OS_VERSION" --ros-distro "$ROS_DISTRO") || echo "警告: $dir 上的 bloom-generate 失败"
done
