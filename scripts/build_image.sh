#!/usr/bin/env bash
# Build the Lotse development image (idempotent).
set -euo pipefail

source "$(dirname "${BASH_SOURCE[0]}")/common.sh"

lotse::build_image
echo "development image ready: ${LOTSE_DEV_IMAGE}"
