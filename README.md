# ROS 1 Release (Ubuntu 22.04 & 24.04 打包自动化)

本项目致力于在官方不支持的环境（如 Ubuntu 22.04 Jammy 和 Ubuntu 24.04 Noble）中通过源码自动化打包生成 ROS 1 Noetic 的 `.deb` 安装包。

## 修改摘要与原因

### 1. `generate.sh` 脚本优化
*   **修改内容**：支持通过参数传递 OS 版本和 ROS 发行版；引入动态 `rosdep` 映射逻辑，排除非包目录（如工具、测试目录及隐藏目录）。
*   **原因**：为了兼容不同版本的 Ubuntu 原生系统依赖名，并解决 `bloom` 无法识别工作空间内尚未打包的内部依赖问题。排除测试目录是为了防止无效或故意的错误 XML 文件干扰解析。

### 2. GitHub Action CI 构建流程
*   **修改内容**：创建了 `.github/workflows/build_deb.yml`，支持 Matrix 策略。
*   **原因**：实现自动化构建产出，并验证在不同 Ubuntu 容器镜像中的兼容性。

### 3. 环境与依赖管理
*   **修改内容**：
    1.  **工具链**：将 `bloom`、`rosdep` 等工具从 `apt` 安装切换为 `pip3`（针对 24.04 使用了 `--break-system-packages`）。
    2.  **系统库**：手动添加了 `nasm`、`libpoco-dev`、`libfltk1.3-dev`、`libogre-1.9-dev`、`libsdl-dev` 等几十个底层依赖。
    3.  **仓库源**：移除了官方 ROS 1 apt 源（避免 404），改用 `tianbot/rosdistro` 自定义源。
*   **原因**：22.04+ 官方不提供 ROS 1 包，因此必须完全依赖源码构建；特定底层库是编译 `rviz`、`image_pipeline` 等核心包所必需的。

### 4. 打包策略：拓扑排序与即时安装
*   **修改内容**：引入 `catkin_tools` 对所有包进行拓扑排序。在 CI 循环中，每打好一个 `.deb` 包就执行 `dpkg -i` 立即安装，并配合 `dpkg-buildpackage -d` 跳过不必要的依赖检查。
*   **原因**：
    *   **拓扑排序**：确保基础包（如 `catkin`）先于上层依赖包构建。
    *   **即时安装**：上层包（如 `roscpp`）在编译时需要底层包提供的 CMake 文件和头文件。通过即时安装，可以实现“一边打包一边满足后续依赖”的滚雪球式构建。
    *   **跳过检查 (`-d`)**：应对复杂的循环依赖和系统尚未索引新包的情况。

## 产物获取
构建完成后，成品 `.deb` 文件将作为 **Artifacts** 存储在 GitHub Action 运行记录中。
