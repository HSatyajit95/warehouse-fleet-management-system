# Build context is the repo root (see docker-compose.yml) so this can COPY
# both fms_api and fms_fleet_server's proto/ (needed to generate gRPC stubs
# at build time — same approach as fms_api/generate_grpc.sh).

FROM python:3.11-slim

WORKDIR /app

COPY src/fms_api/requirements.txt ./requirements.txt
RUN pip install --no-cache-dir -r requirements.txt

COPY src/fms_fleet_server/proto ./proto
COPY src/fms_api/fms_api ./fms_api

RUN mkdir -p fms_api/generated \
    && touch fms_api/generated/__init__.py \
    && python -m grpc_tools.protoc \
         -I./proto \
         --python_out=fms_api/generated \
         --grpc_python_out=fms_api/generated \
         ./proto/fleet.proto \
    && sed -i 's/^import fleet_pb2 as fleet__pb2$/from . import fleet_pb2 as fleet__pb2/' \
         fms_api/generated/fleet_pb2_grpc.py

ENV FLEET_SERVER_ADDR=localhost:50051
EXPOSE 8000

CMD ["uvicorn", "fms_api.main:app", "--host", "0.0.0.0", "--port", "8000"]
