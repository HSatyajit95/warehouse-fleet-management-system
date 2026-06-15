#!/usr/bin/env bash
# generate_grpc_stubs.sh — Regenerate the Python gRPC stubs (fleet_pb2.py,
# fleet_pb2_grpc.py) used by load_test.py from proto/fleet.proto.
#
# The generated files are NOT checked into git (they're build artifacts of
# the .proto definition and depend on the local grpcio-tools version), so
# this must be run once before using load_test.py, and again any time
# proto/fleet.proto changes.
#
# Usage:
#   ./generate_grpc_stubs.sh [output_dir]
#
# output_dir defaults to this script's own directory, so load_test.py can
# import fleet_pb2 / fleet_pb2_grpc directly when run from here.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROTO_DIR="${SCRIPT_DIR}/../proto"
OUT_DIR="${1:-${SCRIPT_DIR}}"

mkdir -p "${OUT_DIR}"

python3 -m grpc_tools.protoc \
    -I "${PROTO_DIR}" \
    --python_out="${OUT_DIR}" \
    --grpc_python_out="${OUT_DIR}" \
    "${PROTO_DIR}/fleet.proto"

echo "Generated fleet_pb2.py and fleet_pb2_grpc.py in ${OUT_DIR}"
