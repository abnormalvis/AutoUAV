#!/usr/bin/env bash
set -euo pipefail
RM_STAGE=roscore
. "$(dirname "$(readlink -f "$0")")/common.sh"
exec roscore
