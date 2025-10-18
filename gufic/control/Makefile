# Jaka Robot Controller Makefile
# 用于简化构建和运行过程

.PHONY: help setup test run clean install check-deps

# 默认目标
help:
	@echo "Jaka Robot Controller - Available commands:"
	@echo ""
	@echo "  setup       - 设置环境和依赖"
	@echo "  check-deps  - 检查依赖项"
	@echo "  test        - 运行基本功能测试"
	@echo "  run         - 启动控制器"
	@echo "  install     - 安装Python依赖"
	@echo "  build-ros2  - 构建ROS2工作空间"
	@echo "  clean       - 清理生成的文件"
	@echo "  help        - 显示此帮助信息"

# 设置环境
setup: install build-ros2
	@echo "环境设置完成"

# 检查依赖项
check-deps:
	@echo "检查Python依赖..."
	@python3 -c "import numpy, scipy, rclpy; print('✓ Python依赖检查通过')" || echo "✗ 缺少Python依赖，请运行 make install"
	@echo "检查Jaka SDK..."
	@test -f python/out/shared/libjakaAPI.so && echo "✓ Jaka SDK库文件存在" || echo "✗ Jaka SDK库文件不存在"
	@test -f python/out/python3/jkrc.so && echo "✓ Jaka Python模块存在" || echo "✗ Jaka Python模块不存在"
	@echo "检查ROS2工作空间..."
	@test -d ../../ros2_ws/install && echo "✓ ROS2工作空间已构建" || echo "✗ ROS2工作空间未构建，请运行 make build-ros2"

# 安装Python依赖
install:
	@echo "安装Python依赖..."
	pip3 install numpy scipy matplotlib
	@echo "✓ Python依赖安装完成"

# 构建ROS2工作空间
build-ros2:
	@echo "构建ROS2工作空间..."
	cd ../../ros2_ws && colcon build
	@echo "✓ ROS2工作空间构建完成"

# 运行基本测试
test: check-deps
	@echo "运行Jaka机械臂基本功能测试..."
	python3 test_jaka_basic.py

# 启动控制器
run: check-deps
	@echo "启动Jaka机械臂控制器..."
	python3 start_jaka_controller.py

# 只启动控制器节点（不启动力传感器）
run-controller-only: check-deps
	@echo "启动Jaka控制器节点..."
	python3 jaka_publisher.py

# 只启动力传感器节点
run-force-sensor:
	@echo "启动力传感器节点..."
	cd ../../ros2_ws && source install/setup.bash && ros2 run ros2_force_sensor force_sensor_node

# 启动力传感器（使用启动文件）
run-force-sensor-launch:
	@echo "使用启动文件启动力传感器..."
	cd ../../ros2_ws && source install/setup.bash && ros2 launch launch/force_sensor.launch.py

# 监控ROS2话题
monitor-topics:
	@echo "监控ROS2话题..."
	@echo "关节状态: ros2 topic echo /joint_states"
	@echo "末端力扭矩: ros2 topic echo /ee_wrench"  
	@echo "力传感器数据: ros2 topic echo /force_sensor_data"

# 查看日志
view-logs:
	@echo "查看最新日志文件..."
	@test -d ./log && ls -la ./log/ || echo "日志目录不存在"

# 清理生成的文件
clean:
	@echo "清理生成的文件..."
	rm -rf ./log/*.csv
	rm -rf __pycache__
	rm -rf .pytest_cache
	find . -name "*.pyc" -delete
	@echo "✓ 清理完成"

# 完全清理（包括ROS2构建文件）
clean-all: clean
	@echo "完全清理..."
	cd ../../ros2_ws && rm -rf build install log
	@echo "✓ 完全清理完成"

# 检查网络连接
check-network:
	@echo "检查与机械臂的网络连接..."
	@ping -c 3 192.168.125.3 && echo "✓ 网络连接正常" || echo "✗ 无法连接到机械臂"

# 显示系统信息
info:
	@echo "=== 系统信息 ==="
	@echo "Python版本: $$(python3 --version)"
	@echo "ROS2版本: $$(ros2 --version 2>/dev/null || echo '未安装')"
	@echo "当前目录: $$(pwd)"
	@echo "磁盘使用: $$(df -h . | tail -1 | awk '{print $$4 " available"}')"

# 创建日志目录
create-log-dir:
	@mkdir -p ./log
	@echo "✓ 日志目录已创建"

# 备份配置
backup-config:
	@echo "备份配置文件..."
	@cp jaka_config.ini jaka_config.ini.backup.$$(date +%Y%m%d_%H%M%S)
	@echo "✓ 配置文件已备份"

# 恢复配置
restore-config:
	@echo "可用的配置备份文件:"
	@ls -la jaka_config.ini.backup.* 2>/dev/null || echo "无备份文件"

# 开发者工具
dev-setup: setup
	@echo "安装开发工具..."
	pip3 install pytest black flake8
	@echo "✓ 开发环境设置完成"

# 代码格式化
format:
	@echo "格式化Python代码..."
	black *.py
	@echo "✓ 代码格式化完成"

# 代码检查
lint:
	@echo "检查代码质量..."
	flake8 *.py --max-line-length=100
	@echo "✓ 代码检查完成"

# 运行单元测试
unittest:
	@echo "运行单元测试..."
	python3 -m pytest test_*.py -v
	@echo "✓ 单元测试完成"

# 性能测试
benchmark:
	@echo "运行性能测试..."
	@python3 -c "import time; print('Performance test placeholder')"

# 显示当前配置
show-config:
	@echo "当前配置 (jaka_config.ini):"
	@cat jaka_config.ini

# 生成文档
doc:
	@echo "生成文档..."
	@echo "README.md 已存在"
	@echo "API文档可通过 pydoc 生成"

# 快速开始
quickstart: setup test
	@echo ""
	@echo "=== 快速开始指南 ==="
	@echo "1. 确保机械臂已连接并处于就绪状态"
	@echo "2. 运行: make run"
	@echo "3. 监控数据: make monitor-topics"
	@echo "4. 停止控制器: Ctrl+C"
	@echo ""

# 故障排除
troubleshoot:
	@echo "=== 故障排除 ==="
	@echo "检查常见问题..."
	@make check-deps
	@make check-network
	@make info