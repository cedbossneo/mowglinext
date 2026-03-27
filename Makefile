SHELL := /bin/bash

ROS_DISTRO    ?= jazzy
BUILD_TYPE    ?= Release
DOCKER_IMAGE  ?= mowgli-ros2
DOCKER_TAG    ?= latest
ROBOT_HOST    ?= mowgli.local
ROBOT_USER    ?= pi

# GitHub Container Registry image name (set GITHUB_REPOSITORY in CI env)
GHCR_IMAGE    ?= ghcr.io/$(GITHUB_REPOSITORY)/$(DOCKER_IMAGE)

.PHONY: help build build-debug test clean \
        docker docker-build docker-sim docker-dev \
        run-sim run-sim-gui run-hardware foxglove \
        lint format format-check \
        deploy backup-maps

# ─── Help ──────────────────────────────────────────────────────────────────────

help:
	@echo "Mowgli ROS2 Build System"
	@echo ""
	@echo "Build targets:"
	@echo "  build          Build all packages with colcon (Release)"
	@echo "  build-debug    Build with Debug symbols"
	@echo "  test           Run all unit tests with colcon"
	@echo "  clean          Remove build/ install/ log/ directories"
	@echo ""
	@echo "Docker targets:"
	@echo "  docker         Build production Docker image (runtime stage)"
	@echo "  docker-sim     Build simulation Docker image"
	@echo "  docker-dev     Build development Docker image"
	@echo "  run-sim        Run headless simulation (connect Foxglove to ws://localhost:8765)"
	@echo "  run-sim-gui    Run simulation with Gazebo GUI via noVNC (http://localhost:6080/vnc.html)"
	@echo "  run-hardware   Run hardware stack via docker compose"
	@echo "  foxglove       Open Foxglove Studio with the Mowgli layout"
	@echo ""
	@echo "Code quality:"
	@echo "  lint           Run cppcheck + cpplint on all C++ sources"
	@echo "  format         Apply clang-format to all C++ files in-place"
	@echo "  format-check   Verify formatting without modifying files"
	@echo ""
	@echo "Deployment:"
	@echo "  deploy         Sync install tree to Raspberry Pi and restart service"
	@echo "  backup-maps    Pull SLAM maps from robot into maps_backup/"
	@echo ""
	@echo "Overridable variables:"
	@echo "  ROS_DISTRO     ROS 2 distribution (default: humble)"
	@echo "  BUILD_TYPE     CMake build type (default: Release)"
	@echo "  DOCKER_IMAGE   Base image name (default: mowgli-ros2)"
	@echo "  DOCKER_TAG     Image tag (default: latest)"
	@echo "  ROBOT_HOST     Target hostname for deploy (default: mowgli.local)"
	@echo "  ROBOT_USER     SSH user for deploy (default: pi)"

# ─── Build ─────────────────────────────────────────────────────────────────────

build:
	source /opt/ros/$(ROS_DISTRO)/setup.bash && \
	colcon build \
	  --cmake-args -DCMAKE_BUILD_TYPE=$(BUILD_TYPE) \
	  --parallel-workers $$(nproc) \
	  --event-handlers console_cohesion+

build-debug:
	$(MAKE) build BUILD_TYPE=Debug

test:
	source /opt/ros/$(ROS_DISTRO)/setup.bash && \
	source install/setup.bash && \
	colcon test --return-code-on-test-failure && \
	colcon test-result --verbose

clean:
	rm -rf build/ install/ log/

# ─── Docker ────────────────────────────────────────────────────────────────────

docker-build:
	docker build --target build \
	  --build-arg BUILD_TYPE=$(BUILD_TYPE) \
	  --progress=plain \
	  -t $(DOCKER_IMAGE)-build:$(DOCKER_TAG) .

docker:
	docker build --target runtime \
	  --build-arg BUILD_TYPE=$(BUILD_TYPE) \
	  -t $(DOCKER_IMAGE):$(DOCKER_TAG) .

docker-sim:
	docker build --target simulation \
	  --build-arg BUILD_TYPE=$(BUILD_TYPE) \
	  -t $(DOCKER_IMAGE)-sim:$(DOCKER_TAG) .

docker-dev:
	docker build -f Dockerfile.dev \
	  -t $(DOCKER_IMAGE)-dev:$(DOCKER_TAG) .

sim-build:
	@echo "Compiling workspace inside simulation container..."
	docker compose exec simulation bash -c "\
	  source /opt/ros/$(ROS_DISTRO)/setup.bash && \
	  cd /ros2_ws && \
	  colcon build \
	    --cmake-args -DCMAKE_BUILD_TYPE=Release \
	    --parallel-workers \$$(nproc) \
	    --event-handlers console_cohesion+"

sim-build-pkg:
	@echo "Compiling $(PKG) inside simulation container..."
	docker compose exec simulation bash -c "\
	  source /opt/ros/$(ROS_DISTRO)/setup.bash && \
	  source /ros2_ws/install/setup.bash && \
	  cd /ros2_ws && \
	  colcon build \
	    --packages-select $(PKG) \
	    --cmake-args -DCMAKE_BUILD_TYPE=Release \
	    --parallel-workers \$$(nproc)"

sim-restart:
	docker compose restart simulation

run-sim:
	@echo "Starting headless simulation..."
	@echo "  Foxglove Studio: ws://localhost:8765"
	@echo "  Import layout:   foxglove/mowgli_sim.json"
	@echo ""
	docker compose up simulation

run-sim-gui:
	@echo "Starting simulation with Gazebo GUI..."
	@echo "  Gazebo GUI:      http://localhost:6080/vnc.html"
	@echo "  Foxglove Studio: ws://localhost:8765"
	@echo ""
	docker compose up simulation-gui

run-hardware:
	docker compose up mowgli

foxglove:
	@echo "Opening Foxglove Studio..."
	@echo "Make sure to connect to ws://localhost:8765 (Foxglove Bridge)"
	@echo "Import the layout from foxglove/mowgli_sim.json"
	@open -a "Foxglove Studio" 2>/dev/null || \
	  (echo "Foxglove Studio not found. Install from https://foxglove.dev/download" && \
	   echo "Or use the web version: https://app.foxglove.dev" && \
	   open "https://app.foxglove.dev/?ds=foxglove-websocket&ds.url=ws%3A%2F%2Flocalhost%3A8765")

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
