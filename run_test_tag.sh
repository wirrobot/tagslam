#!/usr/bin/env bash
# ============================================================================
# Run AprilTag detection + pose estimation tests.
#
# Usage:
#   ./run_test_tag.sh           # run all tag detection tests
#   ./run_test_tag.sh -k pose   # filter by test name
#   ./run_test_tag.sh -sv       # verbose + no capture (show prints)
#   ./run_test_tag.sh -h        # show help
#
# Requires: test.jpg in tools/tests/
# ============================================================================

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
TEST_DIR="${SCRIPT_DIR}/tools/tests"
TOOLS_SRC="${SCRIPT_DIR}/tools/src"
TEST_IMAGE="${TEST_DIR}/test.jpg"
CONFIG_DIR="${SCRIPT_DIR}/config"

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

# ---------------------------------------------------------------------------
# Check prerequisites
# ---------------------------------------------------------------------------

check_prereqs() {
    echo -e "${CYAN}Checking prerequisites...${NC}"

    if ! python3 -c "import pytest" &>/dev/null; then
        echo -e "${RED}pytest not found. Install it:${NC}"
        echo "  pip3 install pytest"
        exit 1
    fi

    if ! python3 -c "import cv2" &>/dev/null; then
        echo -e "${RED}opencv-python not found. Install it:${NC}"
        echo "  pip3 install opencv-python"
        exit 1
    fi

    if ! python3 -c "from pupil_apriltags import Detector" &>/dev/null; then
        echo -e "${RED}pupil_apriltags not found. Install it:${NC}"
        echo "  pip3 install pupil-apriltags"
        exit 1
    fi

    if ! python3 -c "from yaml import safe_load" &>/dev/null; then
        echo -e "${RED}PyYAML not found. Install it:${NC}"
        echo "  pip3 install pyyaml"
        exit 1
    fi

    if [ ! -f "$TEST_IMAGE" ]; then
        echo -e "${YELLOW}Warning: test.jpg not found at $TEST_IMAGE${NC}"
        echo -e "${YELLOW}Place an image with AprilTags there and re-run.${NC}"
        echo -e "${YELLOW}Tests will be skipped.${NC}"
    fi

    if [ ! -f "$CONFIG_DIR/cameras.yaml" ]; then
        echo -e "${YELLOW}Warning: config/cameras.yaml not found${NC}"
    fi

    if [ ! -f "$CONFIG_DIR/tagslam.yaml" ]; then
        echo -e "${YELLOW}Warning: config/tagslam.yaml not found${NC}"
    fi

    echo -e "${GREEN}Prerequisites OK.${NC}"
    echo ""
}

# ---------------------------------------------------------------------------
# Usage
# ---------------------------------------------------------------------------

usage() {
    echo "Usage: $0 [PYTEST_ARGS...]"
    echo ""
    echo "Run AprilTag detection + pose estimation tests."
    echo "All extra arguments are passed directly to pytest."
    echo ""
    echo "Examples:"
    echo "  $0                    # run all tag tests"
    echo "  $0 -sv                # show print() output"
    echo "  $0 -k pose            # run only pose-related tests"
    echo "  $0 -k test_detect     # run only tag detection test"
    echo ""
    echo "Required: test.jpg placed in tools/tests/"
}

# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

if [ "${1:-}" = "-h" ] || [ "${1:-}" = "--help" ]; then
    usage
    exit 0
fi

check_prereqs

echo -e "${CYAN}Running tag detection tests...${NC}"
echo ""

cd "$SCRIPT_DIR"

PYTHONPATH="${TOOLS_SRC}:${TEST_DIR}:${PYTHONPATH:-}" \
TAGSLAM_CONFIG_DIR="$CONFIG_DIR" \
python3 -m pytest "$TEST_DIR/test_tag_detection.py" \
    --ignore="${TEST_DIR}/test_cli.py" \
    -v --tb=short --color=yes \
    "$@"

exit_code=$?

echo ""
if [ $exit_code -eq 0 ]; then
    echo -e "${GREEN}All tag detection tests passed.${NC}"
else
    echo -e "${RED}Some tests failed (exit code: $exit_code).${NC}"
fi

exit $exit_code
