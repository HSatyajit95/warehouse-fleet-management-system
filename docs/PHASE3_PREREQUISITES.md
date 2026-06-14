# Phase 3 Prerequisites — Software to Install

Checked against your system (Ubuntu 22.04 "jammy"). Run all commands as
shown; `sudo` is required for installs.

> **Status: ✅ All installed (2026-06-14).** See the Summary table at the
> bottom for what's actually on this machine. The sections below are kept
> as a reference for what was done / how to redo it on another machine.

---

## 1. gRPC + Protobuf (C++) — ✅ mostly easy via apt

`protobuf-compiler` and `libprotobuf-dev` are **already installed**
(3.12.4). Still need the gRPC libraries and the gRPC-aware protoc plugin:

```bash
sudo apt update
sudo apt install -y libgrpc++-dev protobuf-compiler-grpc libgrpc-dev
```

This gives you `protoc` (already present), `protoc-gen-grpc_cpp_plugin`,
and the gRPC C++ headers/libs needed to build `fms_fleet_server`.

---

## 2. RabbitMQ — ✅ easy via apt

Server (the broker itself) + C client library for the C++ code:

```bash
sudo apt install -y rabbitmq-server librabbitmq-dev
```

After install, the service starts automatically. Useful checks:
```bash
sudo systemctl status rabbitmq-server
sudo rabbitmqctl status
sudo rabbitmqctl list_queues   # should run without error
```

(Optional) enable the web management UI (http://localhost:15672,
default login `guest`/`guest`):
```bash
sudo rabbitmq-plugins enable rabbitmq_management
```

---

## 3. MongoDB — ⚠️ needs MongoDB's own apt repo (not in default Ubuntu 22.04 repos)

### 3a. Server
```bash
# Import MongoDB's GPG key and add their repo (MongoDB 7.0)
curl -fsSL https://pgp.mongodb.com/server-7.0.asc | \
  sudo gpg -o /usr/share/keyrings/mongodb-server-7.0.gpg --dearmor

echo "deb [ signed-by=/usr/share/keyrings/mongodb-server-7.0.gpg ] \
  https://repo.mongodb.org/apt/ubuntu jammy/mongodb-org/7.0 multiverse" | \
  sudo tee /etc/apt/sources.list.d/mongodb-org-7.0.list

sudo apt update
sudo apt install -y mongodb-org

sudo systemctl enable --now mongod
mongosh --eval "db.runCommand({ping: 1})"   # should print { ok: 1 }
```
`mongosh` (the shell) is included in `mongodb-org`.

### 3a. Server — done via Docker instead

Native apt install (above) was skipped. Instead MongoDB 7 runs as a Docker
container:
```bash
sg docker -c "docker run -d --name fms-mongo -p 27017:27017 --restart unless-stopped mongo:7"
```
Verify: `sg docker -c "docker exec fms-mongo mongosh --quiet --eval 'db.runCommand({ping:1})'"` → `{ ok: 1 }`

(`sg docker -c "..."` is needed until you log out/in so your shell picks up
group membership added by `usermod -aG docker $USER`.)

### 3b. C++ driver (`mongocxx` / `bsoncxx`) — built from source

The 1.27.0 / r3.10.1 releases referenced in older guides have been removed
from GitHub. Used current releases instead: **mongo-c-driver 2.3.1** +
**mongo-cxx-driver r4.3.1** (designed to work together). ~10–15 minutes.

```bash
# 1. mongo-c-driver (dependency)
cd /tmp
curl -LO https://github.com/mongodb/mongo-c-driver/releases/download/2.3.1/mongo-c-driver-2.3.1.tar.gz
tar xzf mongo-c-driver-2.3.1.tar.gz && cd mongo-c-driver-2.3.1
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release -DENABLE_AUTOMATIC_INIT_AND_CLEANUP=OFF ..
make -j$(nproc)
sudo make install && sudo ldconfig

# 2. mongo-cxx-driver
cd /tmp
curl -LO https://github.com/mongodb/mongo-cxx-driver/releases/download/r4.3.1/mongo-cxx-driver-r4.3.1.tar.gz
tar xzf mongo-cxx-driver-r4.3.1.tar.gz && cd mongo-cxx-driver-r4.3.1
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_STANDARD=17 -DBUILD_SHARED_LIBS=ON -DCMAKE_PREFIX_PATH=/usr/local ..
make -j$(nproc)
sudo make install && sudo ldconfig
```

Verify: `pkg-config --modversion libmongocxx libbsoncxx` → `4.3.1` / `4.3.1`

---

## 4. CMake version check

`mongo-cxx-driver` r3.10.x needs CMake ≥ 3.17. Check yours:
```bash
cmake --version
```
Ubuntu 22.04 ships CMake 3.22 — fine.

---

## 5. Docker (optional now, required for Phase 4)

Not strictly needed for Phase 3 if you install Mongo/RabbitMQ natively
(§2–3), but useful now if you'd rather run them as containers:
```bash
sudo apt install -y docker.io docker-compose-v2
sudo usermod -aG docker $USER   # log out/in afterward
```

---

## Summary — final status (2026-06-14)

| Component | Status | Notes |
|---|---|---|
| gRPC + Protobuf | ✅ installed (1.30.2 / 3.12.4) | `libgrpc++-dev`, `protobuf-compiler-grpc` |
| RabbitMQ | ✅ installed, active (3.9.27) | `librabbitmq-dev` for C++ client |
| Docker | ✅ installed, active (29.1.3) | use `sg docker -c "..."` until next login |
| MongoDB server | ✅ running in Docker (`fms-mongo`, mongo:7, port 27017) | |
| MongoDB C++ driver | ✅ built from source (mongoc 2.3.1 / mongocxx 4.3.1) | installed to `/usr/local` |
| Build tools (cmake 3.22, build-essential, etc.) | ✅ already present | no action needed |

All prerequisites are satisfied. Next: scaffold `fms_fleet_server` (Step 3.1
in `PHASE3_PLAN.md`).
