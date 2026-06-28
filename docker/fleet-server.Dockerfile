# Build context is the repo root (see docker-compose.yml) so this can COPY
# fms_msgs + fms_fleet_server without reaching outside the build context.
#
# Multi-stage: the builder stage compiles mongo-c-driver/mongo-cxx-driver
# from source (same versions as docs/PHASE3_PREREQUISITES.md — current
# mongocxx releases aren't in apt) and colcon-builds fms_msgs +
# fms_fleet_server; the runtime stage only carries the resulting shared
# libraries, not the whole toolchain.

FROM ros:humble-ros-base AS builder

RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential cmake git curl ca-certificates \
      libgrpc++-dev protobuf-compiler-grpc libprotobuf-dev protobuf-compiler \
      librabbitmq-dev nlohmann-json3-dev \
      ros-humble-std-srvs \
      python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# mongo-c-driver (mongocxx's dependency)
WORKDIR /tmp/mongo-c-driver
RUN curl -fsSL https://github.com/mongodb/mongo-c-driver/releases/download/2.3.1/mongo-c-driver-2.3.1.tar.gz \
      -o mongo-c-driver.tar.gz \
    && tar xzf mongo-c-driver.tar.gz --strip-components=1 \
    && mkdir -p build && cd build \
    && cmake -DCMAKE_BUILD_TYPE=Release -DENABLE_AUTOMATIC_INIT_AND_CLEANUP=OFF .. \
    && make -j"$(nproc)" \
    && make install \
    && ldconfig

# mongo-cxx-driver
WORKDIR /tmp/mongo-cxx-driver
RUN curl -fsSL https://github.com/mongodb/mongo-cxx-driver/releases/download/r4.3.1/mongo-cxx-driver-r4.3.1.tar.gz \
      -o mongo-cxx-driver.tar.gz \
    && tar xzf mongo-cxx-driver.tar.gz --strip-components=1 \
    && mkdir -p build && cd build \
    && cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_STANDARD=17 \
             -DBUILD_SHARED_LIBS=ON -DCMAKE_PREFIX_PATH=/usr/local .. \
    && make -j"$(nproc)" \
    && make install \
    && ldconfig

WORKDIR /ws
COPY src/fms_msgs src/fms_msgs
COPY src/fms_fleet_server src/fms_fleet_server

RUN . /opt/ros/humble/setup.sh \
    && colcon build --packages-up-to fms_fleet_server \
         --cmake-args -DCMAKE_BUILD_TYPE=Release

# ── runtime stage ────────────────────────────────────────────────────────────
FROM ros:humble-ros-base AS runtime

RUN apt-get update && apt-get install -y --no-install-recommends \
      libgrpc++1 libgrpc10 libprotobuf23 librabbitmq4 \
      ros-humble-std-srvs \
    && rm -rf /var/lib/apt/lists/*

# mongocxx/mongoc shared libs + headers built in the builder stage
COPY --from=builder /usr/local/lib/ /usr/local/lib/
COPY --from=builder /usr/local/include/ /usr/local/include/
RUN ldconfig

COPY --from=builder /ws/install /ws/install

ENV MONGO_URI=mongodb://localhost:27017 \
    MONGO_DB=fms \
    RABBITMQ_HOST=localhost \
    RABBITMQ_PORT=5672 \
    GRPC_PORT=50051 \
    NUM_ROBOTS=4

ENTRYPOINT ["/bin/bash", "-c", "\
    source /opt/ros/humble/setup.bash && \
    source /ws/install/setup.bash && \
    exec ros2 run fms_fleet_server fleet_server_node --ros-args \
      -p mongo_uri:=\"$MONGO_URI\" \
      -p mongo_db:=\"$MONGO_DB\" \
      -p rabbitmq_host:=\"$RABBITMQ_HOST\" \
      -p rabbitmq_port:=\"$RABBITMQ_PORT\" \
      -p grpc_port:=\"$GRPC_PORT\" \
      -p num_robots:=\"$NUM_ROBOTS\" \
    "]
