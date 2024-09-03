# Makefile for Upkie agents
#
# SPDX-License-Identifier: Apache-2.0

# Hostname or IP address of the Raspberry Pi Uses the value from the
# UPKIE_NAME environment variable, if defined. Valid usage: ``make upload
# UPKIE_NAME=foo``
REMOTE = ${UPKIE_NAME}

CURDATE = $(shell date -Iseconds)
CURDIR_NAME = $(shell basename $(CURDIR))

# Help snippet adapted from:
# http://marmelab.com/blog/2016/02/29/auto-documented-makefile.html
.PHONY: help
help:
	@echo "Available targets:\n"
	@grep -P '^[a-zA-Z0-9_-]+:.*? ## .*$$' $(MAKEFILE_LIST) | sort | awk 'BEGIN {FS = ":.*?## "}; {printf "    \033[36m%-24s\033[0m %s\n", $$1, $$2}'
.DEFAULT_GOAL := help

.PHONY: check_upkie_name
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

.PHONY: upload
upload: check_upkie_name ## upload built targets to the Raspberry Pi
	ssh $(REMOTE) sudo date -s "$(CURDATE)"
	ssh $(REMOTE) mkdir -p $(CURDIR_NAME)
	ssh $(REMOTE) sudo find $(CURDIR_NAME) -type d -name __pycache__ -user root -exec chmod go+wx {} "\;"
	rsync -Lrtu --delete-after --delete-excluded --progress $(CURDIR)/ $(REMOTE):$(CURDIR_NAME)/
