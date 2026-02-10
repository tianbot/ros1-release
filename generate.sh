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
# 预先注入一些已知在 Ubuntu 22.04/24.04 上有问题的映射
cat <<EOF > "$LOCAL_ROSDEP_YAML"
# Local ROS packages and compatibility mappings
opencv2:
  ubuntu: [libopencv-dev]
python3-rospkg:
  ubuntu: [python3-pip]
python3-catkin-pkg:
  ubuntu: [python3-pip]
eigen:
  ubuntu: [libeigen3-dev]
libqt5-svg-dev:
  ubuntu: [libqt5svg5-dev]
libqt5-websockets-dev:
  ubuntu: [libqt5websockets5-dev]
libqt5-opengl-dev:
  ubuntu: [libqt5opengl5-dev]
lua-dev:
  ubuntu: [liblua5.2-dev]
fmt:
  ubuntu: [libfmt-dev]
nlohmann-json-dev:
  ubuntu: [nlohmann-json3-dev]
EOF

# 遍历所有 package.xml 生成映射： pkg_name -> ros-noetic-pkg-name
# 排除 .git, .pc 等目录
find . -maxdepth 4 \( -name ".*" -o -path "./rosdep" -o -path "./rosdistro" -o -path "./rospkg" -o -name "test" -o -name "tests" \) -prune -o -type f -name 'package.xml' -print | while read -r pkg_xml; do
    pkg_name=$(grep -oPm1 "(?<=<name>)[^<]+" "$pkg_xml")
    if [ ! -z "$pkg_name" ]; then
        deb_name="ros-noetic-$(echo "$pkg_name" | tr '_' '-')"
        echo "$pkg_name:" >> "$LOCAL_ROSDEP_YAML"
        echo "  ubuntu: [$deb_name]" >> "$LOCAL_ROSDEP_YAML"
    fi
done

# 如果文件只有一行注释，添加一个空字典符号以防万一
if [ $(wc -l < "$LOCAL_ROSDEP_YAML") -le 1 ]; then
    echo "{} " >> "$LOCAL_ROSDEP_YAML"
fi

# 将本地源添加到 rosdep
sudo sh -c "echo 'yaml file://$LOCAL_ROSDEP_YAML' > /etc/ros/rosdep/sources.list.d/10-local.list"
rosdep update

# 遍历目录执行 bloom-generate
find . -maxdepth 4 \( -name ".*" -o -path "./rosdep" -o -path "./rosdistro" -o -path "./rospkg" -o -name "test" -o -name "tests" \) -prune -o -type f -name 'package.xml' -print | while read -r package_xml; do
    dir=$(dirname "$package_xml")
    pkg_name=$(grep -oPm1 "(?<=<name>)[^<]+" "$package_xml")
    printf "正在为 %-30s 生成元数据 ... " "$pkg_name"
    
    if (cd "$dir" && bloom-generate rosdebian --os-name ubuntu --os-version "$OS_VERSION" --ros-distro "$ROS_DISTRO" > bloom.log 2>&1); then
        echo "✅ 成功"
        echo "::group::查看生成日志: $pkg_name"
        cat "$dir/bloom.log"
        echo "::endgroup::"
    else
        echo "❌ 失败"
        echo "--------------------------------------------------"
        echo "生成元数据失败日志: $pkg_name"
        cat "$dir/bloom.log"
        echo "--------------------------------------------------"
        echo "::warning title=Bloom Generate Failed::目录 $dir 上的 bloom-generate 失败，跳过该包。"
    fi
done
