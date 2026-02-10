#!/usr/bin/env bash
set -euo pipefail

OS_VERSION="${1:-jammy}"

export DEBIAN_FRONTEND="${DEBIAN_FRONTEND:-noninteractive}"

apt-get update
apt-get install -y software-properties-common
add-apt-repository universe
add-apt-repository multiverse

apt-get update
apt-get install -y \
  python3-pip \
  python3-setuptools \
  python3-catkin-pkg \
  python3-wheel \
  fakeroot \
  dpkg-dev \
  debhelper \
  build-essential \
  cmake \
  git \
  sudo \
  curl \
  gnupg2 \
  lsb-release \
  nasm \
  python3-gnupg \
  python3-pycryptodome \
  bridge-utils \
  libassuan-dev \
  libcaca-dev \
  libfltk1.3-dev \
  libgpg-error-dev \
  libgpgme-dev \
  libpoco-dev \
  libconsole-bridge-dev \
  libboost-all-dev \
  uuid-dev \
  libtinyxml-dev \
  libtinyxml2-dev \
  libsdl2-dev \
  libslang2-dev \
  liburdfdom-dev \
  liburdfdom-headers-dev \
  libv4l-dev \
  libzzip-dev \
  ninja-build \
  libgflags-dev \
  libgoogle-glog-dev \
  liblua5.2-dev \
  libprotobuf-dev \
  libsuitesparse-dev \
  libwebp-dev \
  protobuf-compiler \
  python3-sphinx \
  libatlas-base-dev \
  stow \
  libceres-dev \
  libabsl-dev \
  libfmt-dev \
  nlohmann-json3-dev \
  libzmq3-dev \
  liblz4-dev \
  libzstd-dev \
  libqt5svg5-dev \
  libqt5websockets5-dev \
  libqt5x11extras5-dev \
  libqt5opengl5-dev

if [ "$OS_VERSION" = "noble" ]; then
  apt-get install -y libogre-1.12-dev
  pip3 install --no-cache-dir bloom rosdep rosinstall-generator vcstool --break-system-packages
else
  apt-get install -y libogre-1.9-dev libsdl1.2-dev libsdl-image1.2-dev libfreeimage-dev libgpgmepp-dev
  pip3 install --no-cache-dir bloom rosdep rosinstall-generator vcstool catkin-tools wstool
fi

mkdir -p /etc/ros/rosdep/sources.list.d/

# 尝试从 tianbot 镜像下载，增加重试和错误检测
echo "Setting up rosdep sources..."
if ! curl -sSL --retry 5 --retry-delay 5 --retry-connrefused --fail "https://github.com/tianbot/rosdistro/raw/master/rosdep/sources.list.d/20-default.list" -o /etc/ros/rosdep/sources.list.d/20-default.list; then
    echo "⚠️ Failed to download from tianbot, falling back to official rosdep init..."
    rm -f /etc/ros/rosdep/sources.list.d/20-default.list
    rosdep init
fi

# 再次检查文件内容是否合规（防止某些代理返回 200 OK 的 HTML 错误页）
if grep -q "<html" /etc/ros/rosdep/sources.list.d/20-default.list; then
    echo "⚠️ Detected HTML content in sources list (likely an error page). Falling back to official rosdep init..."
    rm -f /etc/ros/rosdep/sources.list.d/20-default.list
    rosdep init
fi
