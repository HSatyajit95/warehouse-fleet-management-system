#!/bin/bash
# Regenerate fms_api/generated/*_pb2*.py from fms_fleet_server/proto/fleet.proto.
# Run after any change to fleet.proto.
set -euo pipefail
cd "$(dirname "$0")"

mkdir -p fms_api/generated
touch fms_api/generated/__init__.py

python3 -m grpc_tools.protoc \
  -I ../fms_fleet_server/proto \
  --python_out=fms_api/generated \
  --grpc_python_out=fms_api/generated \
  ../fms_fleet_server/proto/fleet.proto

# protoc emits an absolute "import fleet_pb2" in the _grpc file; fix to a
# package-relative import so it works when fms_api is imported as a module.
sed -i 's/^import fleet_pb2 as fleet__pb2$/from . import fleet_pb2 as fleet__pb2/' \
  fms_api/generated/fleet_pb2_grpc.py

echo "Regenerated fms_api/generated/fleet_pb2.py and fleet_pb2_grpc.py"
