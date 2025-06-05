# Makefile for Upkie agents
#
# SPDX-License-Identifier: Apache-2.0

CURDIR_NAME = $(shell basename $(CURDIR))

# Help snippet adapted from:
# http://marmelab.com/blog/2016/02/29/auto-documented-makefile.html
help:
	@echo "Host targets:\n"
	@grep -P '^[a-zA-Z0-9_-]+:.*? ## .*$$' $(MAKEFILE_LIST) | sort | awk 'BEGIN {FS = ":.*?## "}; {printf "    \033[36m%-24s\033[0m %s\n", $$1, $$2}'
	@echo "\nRobot targets:\n"
	@grep -P '^[a-zA-Z0-9_-]+:.*?### .*$$' $(MAKEFILE_LIST) | sort | awk 'BEGIN {FS = ":.*?### "}; {printf "    \033[36m%-24s\033[0m %s\n", $$1, $$2}'
	@echo ""

check_upkie_name:
	@ if [ -z "${UPKIE_NAME}" ]; then \
		echo "ERROR: Environment variable UPKIE_NAME is not set.\n"; \
		echo "This variable should contain the robot's hostname or IP address for SSH. "; \
		echo "You can define it inline for a one-time use:\n"; \
		echo "    make some_target UPKIE_NAME=your_robot_hostname\n"; \
		echo "Or add the following line to your shell configuration:\n"; \
		echo "    export UPKIE_NAME=your_robot_hostname\n"; \
		exit 1; \
	fi

upload: check_upkie_name  ## upload built targets to the Raspberry Pi
	ssh ${UPKIE_NAME} mkdir -p $(CURDIR_NAME)
	ssh ${UPKIE_NAME} sudo find $(CURDIR_NAME) -type d -name __pycache__ -user root -exec chmod go+wx {} "\;"
	rsync -Lrtu --delete-after \
		--exclude .pixi \
		--exclude activate.sh \
		--exclude env/ \
		--progress \
		$(CURDIR)/ ${UPKIE_NAME}:$(CURDIR_NAME)/

pack_pixi_env:  ## pack Python environment to be deployed on your Upkie
	@pixi run pack-to-upkie || { \
		echo "Error: pixi not found"; \
		echo "See https://pixi.sh/latest/#installation"; \
		exit 1; \
	}

unpack_pixi_env:  ### unpack Python environment
	@pixi-pack unpack environment.tar || { \
		echo "Error: pixi-pack not found"; \
		echo "You can download `pixi-pack-aarch64-unknown-linux-musl` from https://github.com/Quantco/pixi-pack/releases"; \
		exit 1; \
	}

run_agent:  ### run agent
	@if [ -f $(CURDIR)/activate.sh ]; then \
		echo "Running agent from packed environment..."; \
		. $(CURDIR)/activate.sh && python run_agent.py; \
	else \
		echo "Running agent directly..."; \
		python run_agent.py; \
	fi
