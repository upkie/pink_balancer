# Makefile for upkie targets
#
# Copyright 2022 Stéphane Caron
# Copyright 2023 Inria
# SPDX-License-Identifier: Apache-2.0

# Hostname or IP address of the Raspberry Pi
# Uses the value from the ROBOT environment variable, if defined.
# Valid usage: ``make upload ROBOT=foobar``
REMOTE = ${ROBOT}

PROJECT_NAME = pink_balancer

BAZEL = $(CURDIR)/tools/bazelisk
CURDATE = $(shell date --iso=seconds)
CURDIR_NAME = $(shell basename $(CURDIR))
RASPUNZEL = $(CURDIR)/tools/raspunzel

# Help snippet adapted from:
# http://marmelab.com/blog/2016/02/29/auto-documented-makefile.html
.PHONY: help
help:
	@echo "Host targets:\n"
	@grep -P '^[a-zA-Z0-9_-]+:.*? ## .*$$' $(MAKEFILE_LIST) | sort | awk 'BEGIN {FS = ":.*?## "}; {printf "    \033[36m%-24s\033[0m %s\n", $$1, $$2}'
	@echo "\nRaspberry Pi targets:\n"
	@grep -P '^[a-zA-Z0-9_-]+:.*?### .*$$' $(MAKEFILE_LIST) | sort | awk 'BEGIN {FS = ":.*?### "}; {printf "    \033[36m%-24s\033[0m %s\n", $$1, $$2}'

# HOST TARGETS
# ============

.PHONY: check-robot
check-robot:
	@ if [ -z "${ROBOT}" ]; then \
		echo "ERROR: Environment variable ROBOT is not set.\n"; \
		echo "This variable should contain the robot's hostname or IP address for SSH. You"; \
		echo "can define it inline for a one-time use:\n"; \
		echo "    make some_target ROBOT=your_robot_hostname\n"; \
		echo "Or add the following line to your shell configuration:\n"; \
		echo "    export ROBOT=your_robot_hostname\n"; \
		exit 1; \
	fi

.DEFAULT_GOAL := help
.PHONY: clean_broken_links
clean_broken_links:
	find -L $(CURDIR) -type l ! -exec test -e {} \; -delete

.PHONY: build
build: clean_broken_links  ## build Raspberry Pi targets
	$(BAZEL) build --config=pi64 //pink_balancer
	$(BAZEL) build --config=pi64 @upkie//spines:mock_spine
	$(BAZEL) build --config=pi64 @upkie//spines:pi3hat_spine

.PHONY: clean
clean:  ## clean all local build and intermediate files
	$(BAZEL) clean --expunge

.PHONY: upload
upload: check-robot build  ## upload built targets to the Raspberry Pi
	ssh $(REMOTE) sudo date -s "$(CURDATE)"
	ssh $(REMOTE) mkdir -p $(PROJECT_NAME)
	ssh $(REMOTE) sudo find $(PROJECT_NAME) -type d -name __pycache__ -user root -exec chmod go+wx {} "\;"
	rsync -Lrtu --delete-after --delete-excluded --exclude bazel-out/ --exclude bazel-testlogs/ --exclude bazel-$(CURDIR_NAME) --exclude bazel-$(PROJECT_NAME)/ --progress $(CURDIR)/ $(REMOTE):$(PROJECT_NAME)/

# REMOTE TARGETS
# ==============

run_mock_spine:  ### run the mock spine on the Raspberry Pi
	$(RASPUNZEL) run -s @upkie//spines:mock_spine

run_pi3hat_spine:  ### run the pi3hat spine on the Raspberry Pi
	$(RASPUNZEL) run -s @upkie//spines:pi3hat_spine

run_pink_balancer:  ### run the agent
	$(RASPUNZEL) run -v -s //pink_balancer:pink_balancer -- --config pi3hat
