SHELL := /bin/bash

ROS_DISTRO    ?= jazzy
BUILD_TYPE    ?= Release
DOCKER_IMAGE  ?= mowgli-ros2
DOCKER_TAG    ?= latest
ROBOT_HOST    ?= mowgli.local
ROBOT_USER    ?= pi

# Fields2Cover cmake hint (devcontainer installs to /usr/local)
F2C_DIR       ?= /usr/local/cmake/fields2cover

.PHONY: help build build-debug build-pkg test clean \
        sim e2e-test \
        docker docker-sim \
        lint format format-check \
        deploy backup-maps

# ─── Help ──────────────────────────────────────────────────────────────────────

help:
	@echo "Mowgli ROS2 Build System (devcontainer)"
	@echo ""
	@echo "Build:"
	@echo "  build              Build all packages (Release)"
	@echo "  build-debug        Build with Debug symbols"
	@echo "  build-pkg PKG=x    Build a single package"
	@echo "  test               Run all unit tests"
	@echo "  clean              Remove build/ install/ log/"
	@echo ""
	@echo "Simulation & testing:"
	@echo "  sim                Launch headless simulation (Foxglove ws://localhost:8765)"
	@echo "  e2e-test           Run E2E validation (simulation must be running)"
	@echo ""
	@echo "Docker images (CI / production):"
	@echo "  docker             Build production image (runtime stage)"
	@echo "  docker-sim         Build simulation image"
	@echo ""
	@echo "Code quality:"
	@echo "  lint               Run cppcheck + cpplint"
	@echo "  format             Apply clang-format in-place"
	@echo "  format-check       Verify formatting (dry run)"
	@echo ""
	@echo "Deployment:"
	@echo "  deploy             Sync install tree to robot and restart"
	@echo "  backup-maps        Pull SLAM maps from robot"

# ─── Build ─────────────────────────────────────────────────────────────────────

build:
	source /opt/ros/$(ROS_DISTRO)/setup.bash && \
	colcon build \
	  --cmake-args \
	    -DCMAKE_BUILD_TYPE=$(BUILD_TYPE) \
	    -DFields2Cover_DIR=$(F2C_DIR) \
	  --parallel-workers $$(nproc) \
	  --symlink-install \
	  --event-handlers console_cohesion+

build-debug:
	$(MAKE) build BUILD_TYPE=Debug

build-pkg:
	source /opt/ros/$(ROS_DISTRO)/setup.bash && \
	source install/setup.bash 2>/dev/null; \
	colcon build \
	  --packages-select $(PKG) \
	  --cmake-args \
	    -DCMAKE_BUILD_TYPE=$(BUILD_TYPE) \
	    -DFields2Cover_DIR=$(F2C_DIR) \
	  --parallel-workers $$(nproc) \
	  --symlink-install

test:
	source /opt/ros/$(ROS_DISTRO)/setup.bash && \
	source install/setup.bash && \
	colcon test --return-code-on-test-failure && \
	colcon test-result --verbose

clean:
	rm -rf build/ install/ log/

# ─── Simulation & Testing ─────────────────────────────────────────────────────

sim:
	@echo "Starting headless simulation..."
	@echo "  Foxglove: ws://localhost:8765"
	@echo ""
	source /opt/ros/$(ROS_DISTRO)/setup.bash && \
	source install/setup.bash && \
	ros2 launch mowgli_bringup sim_full_system.launch.py \
	  headless:=true use_rviz:=false

e2e-test:
	@echo "Running E2E test (simulation must already be running)..."
	source /opt/ros/$(ROS_DISTRO)/setup.bash && \
	source install/setup.bash && \
	python3 src/e2e_test.py

# ─── Docker Images (CI / production) ──────────────────────────────────────────

docker:
	docker build --target runtime \
	  --build-arg BUILD_TYPE=$(BUILD_TYPE) \
	  -t $(DOCKER_IMAGE):$(DOCKER_TAG) .

docker-sim:
	docker build --target simulation \
	  --build-arg BUILD_TYPE=$(BUILD_TYPE) \
	  -t $(DOCKER_IMAGE)-sim:$(DOCKER_TAG) .

# ─── Code Quality ──────────────────────────────────────────────────────────────

lint:
	@echo "--- cppcheck ---"
	find src/ \( -name "*.cpp" -o -name "*.hpp" \) \
	  | xargs cppcheck \
	      --enable=all \
	      --suppress=missingInclude \
	      --suppress=unmatchedSuppression \
	      --error-exitcode=1 \
	      --quiet
	@echo "--- cpplint ---"
	find src/ \( -name "*.cpp" -o -name "*.hpp" \) \
	  | xargs cpplint --quiet

format:
	find src/ \( -name "*.cpp" -o -name "*.hpp" -o -name "*.h" \) \
	  | xargs clang-format -i -style=file

format-check:
	find src/ \( -name "*.cpp" -o -name "*.hpp" -o -name "*.h" \) \
	  | xargs clang-format --dry-run --Werror -style=file

# ─── Deployment ────────────────────────────────────────────────────────────────

deploy:
	@echo "Deploying to $(ROBOT_USER)@$(ROBOT_HOST)..."
	rsync -avz --delete install/ \
	  $(ROBOT_USER)@$(ROBOT_HOST):/opt/mowgli_ros2/install/
	ssh $(ROBOT_USER)@$(ROBOT_HOST) "sudo systemctl restart mowgli"
	@echo "Deployment complete."

backup-maps:
	mkdir -p maps_backup/$$(date +%Y%m%d_%H%M%S)
	scp $(ROBOT_USER)@$(ROBOT_HOST):/opt/mowgli_ros2/maps/* \
	  maps_backup/$$(date +%Y%m%d_%H%M%S)/
	@echo "Maps backed up to maps_backup/$$(date +%Y%m%d_%H%M%S)/"
