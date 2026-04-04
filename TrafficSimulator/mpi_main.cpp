/**
 * mpi_main.cpp
 * MPI + spatial decomposition: horizontal strips along y; vehicle migration via MPI_Sendrecv.
 * Rank 0 only: SFML window. Full vehicle set gathered on rank 0 for traffic lights and collisions.
 */

#include "Simulation.h"
#include "Renderer.h"
#include "DomainDecomposition.h"
#include "Vehicle.h"
#include <mpi.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <memory>
#include <vector>

using namespace TrafficSim;

namespace {

constexpr int kTagCount = 7001;
constexpr int kTagBlob = 7002;
constexpr int kTagGatherCnt = 8001;
constexpr int kTagGatherBlob = 8002;
constexpr int kTagScatterCnt = 9001;
constexpr int kTagScatterBlob = 9002;

void exchangeVehicles(
    const DomainDecomposition& dom,
    std::vector<std::unique_ptr<Vehicle>>& local,
    int rank,
    int worldSize,
    MPI_Comm comm) {

    std::vector<std::vector<VehicleSerialized>> outbox(static_cast<size_t>(worldSize));
    for (auto it = local.begin(); it != local.end();) {
        const int owner = dom.ownerRank((*it)->getY());
        if (owner != rank) {
            VehicleSerialized s{};
            (*it)->serializeTo(s);
            outbox[static_cast<size_t>(owner)].push_back(s);
            it = local.erase(it);
        } else {
            ++it;
        }
    }

    const int byteSize = static_cast<int>(Vehicle::serializedByteSize());

    for (int r = 0; r < worldSize; ++r) {
        if (r == rank) continue;
        int sendCount = static_cast<int>(outbox[static_cast<size_t>(r)].size());
        int recvCount = 0;
        MPI_Sendrecv(&sendCount, 1, MPI_INT, r, kTagCount,
                     &recvCount, 1, MPI_INT, r, kTagCount, comm, MPI_STATUS_IGNORE);

        VehicleSerialized* sendPtr = sendCount > 0 ? outbox[static_cast<size_t>(r)].data() : nullptr;
        std::vector<VehicleSerialized> recvBuf;
        VehicleSerialized* recvPtr = nullptr;
        if (recvCount > 0) {
            recvBuf.resize(static_cast<size_t>(recvCount));
            recvPtr = recvBuf.data();
        }
        MPI_Sendrecv(sendPtr, sendCount * byteSize, MPI_BYTE, r, kTagBlob,
                     recvPtr, recvCount * byteSize, MPI_BYTE, r, kTagBlob, comm, MPI_STATUS_IGNORE);

        for (int i = 0; i < recvCount; ++i)
            local.push_back(Vehicle::deserialize(recvBuf[static_cast<size_t>(i)]));
    }
}

void broadcastTrafficLights(Simulation& sim, int rank, MPI_Comm comm) {
    const auto& lights = sim.getTrafficLights();
    const int n = static_cast<int>(lights.size());
    std::vector<std::int32_t> stateIdx(static_cast<size_t>(n));
    std::vector<float> timers(static_cast<size_t>(n));

    if (rank == 0) {
        for (int i = 0; i < n; ++i) {
            stateIdx[static_cast<size_t>(i)] = lights[static_cast<size_t>(i)]->getStateIndex();
            timers[static_cast<size_t>(i)] = lights[static_cast<size_t>(i)]->getTimer();
        }
    }

    MPI_Bcast(stateIdx.data(), n, MPI_INT, 0, comm);
    MPI_Bcast(timers.data(), n, MPI_FLOAT, 0, comm);

    if (rank != 0) {
        for (int i = 0; i < n; ++i) {
            LightState st = LightState::GREEN;
            switch (stateIdx[static_cast<size_t>(i)]) {
                case 0: st = LightState::GREEN; break;
                case 1: st = LightState::YELLOW; break;
                default: st = LightState::RED; break;
            }
            sim.getTrafficLights()[static_cast<size_t>(i)]->assignSyncedState(st, timers[static_cast<size_t>(i)]);
        }
    }
}

/// Move all vehicles to rank 0 into `outAll`; clears each rank's `sim.getVehiclesMutable()`.
void gatherAllToRank0(
    Simulation& sim,
    int rank,
    int worldSize,
    MPI_Comm comm,
    std::vector<std::unique_ptr<Vehicle>>& outAll) {

    auto& local = sim.getVehiclesMutable();
    const int localCount = static_cast<int>(local.size());
    const int byteSize = static_cast<int>(Vehicle::serializedByteSize());

    if (rank == 0) {
        outAll.clear();
        for (auto& v : local)
            outAll.push_back(std::move(v));
        local.clear();

        for (int r = 1; r < worldSize; ++r) {
            int cnt = 0;
            MPI_Recv(&cnt, 1, MPI_INT, r, kTagGatherCnt, comm, MPI_STATUS_IGNORE);
            if (cnt <= 0) continue;
            std::vector<VehicleSerialized> buf(static_cast<size_t>(cnt));
            MPI_Recv(buf.data(), cnt * byteSize, MPI_BYTE, r, kTagGatherBlob, comm, MPI_STATUS_IGNORE);
            for (int i = 0; i < cnt; ++i)
                outAll.push_back(Vehicle::deserialize(buf[static_cast<size_t>(i)]));
        }
    } else {
        MPI_Send(&localCount, 1, MPI_INT, 0, kTagGatherCnt, comm);
        if (localCount > 0) {
            std::vector<VehicleSerialized> buf(static_cast<size_t>(localCount));
            for (int i = 0; i < localCount; ++i)
                local[static_cast<size_t>(i)]->serializeTo(buf[static_cast<size_t>(i)]);
            MPI_Send(buf.data(), localCount * byteSize, MPI_BYTE, 0, kTagGatherBlob, comm);
        }
        local.clear();
    }
}

/// Partition `all` by owner rank (y-strips) and send each bucket to its rank; clears `all`.
void scatterFromRank0(
    std::vector<std::unique_ptr<Vehicle>>& all,
    Simulation& sim,
    int rank,
    int worldSize,
    MPI_Comm comm) {

    DomainDecomposition dom(0, worldSize, static_cast<float>(GRID_HEIGHT));
    const int byteSize = static_cast<int>(Vehicle::serializedByteSize());

    if (rank == 0) {
        std::vector<std::vector<VehicleSerialized>> buckets(static_cast<size_t>(worldSize));
        for (auto& v : all) {
            const int o = dom.ownerRank(v->getY());
            VehicleSerialized s{};
            v->serializeTo(s);
            buckets[static_cast<size_t>(o)].push_back(s);
        }
        all.clear();

        for (int r = 1; r < worldSize; ++r) {
            const int cnt = static_cast<int>(buckets[static_cast<size_t>(r)].size());
            MPI_Send(&cnt, 1, MPI_INT, r, kTagScatterCnt, comm);
            if (cnt > 0)
                MPI_Send(buckets[static_cast<size_t>(r)].data(), cnt * byteSize, MPI_BYTE, r, kTagScatterBlob, comm);
        }
        sim.getVehiclesMutable().clear();
        for (auto& s : buckets[0])
            sim.getVehiclesMutable().push_back(Vehicle::deserialize(s));
    } else {
        int cnt = 0;
        MPI_Recv(&cnt, 1, MPI_INT, 0, kTagScatterCnt, comm, MPI_STATUS_IGNORE);
        sim.getVehiclesMutable().clear();
        if (cnt > 0) {
            std::vector<VehicleSerialized> buf(static_cast<size_t>(cnt));
            MPI_Recv(buf.data(), cnt * byteSize, MPI_BYTE, 0, kTagScatterBlob, comm, MPI_STATUS_IGNORE);
            for (int i = 0; i < cnt; ++i)
                sim.getVehiclesMutable().push_back(Vehicle::deserialize(buf[static_cast<size_t>(i)]));
        }
    }
}

} // namespace

int main(int argc, char** argv) {
    MPI_Init(&argc, &argv);
    int rank = 0;
    int worldSize = 1;
    MPI_Comm_rank(MPI_COMM_WORLD, &rank);
    MPI_Comm_size(MPI_COMM_WORLD, &worldSize);

    constexpr int VEHICLE_COUNT = 120;

    Simulation simulation;
    simulation.createTrafficLightsOnly();
    simulation.spawnVehiclesForMpiRank(rank, worldSize, VEHICLE_COUNT);

    DomainDecomposition dom(rank, worldSize, static_cast<float>(GRID_HEIGHT));

    std::unique_ptr<Renderer> renderer;
    if (rank == 0) {
        renderer = std::make_unique<Renderer>(1200, 720);
        if (!renderer->initialize(false)) {
            std::cerr << "MPI rank 0: renderer init failed.\n";
            MPI_Abort(MPI_COMM_WORLD, 1);
        }
    }

    bool running = true;
    int frames = 0;
    auto tLastFrame = std::chrono::high_resolution_clock::now();
    float fpsDisplay = 60.f;

    std::vector<std::unique_ptr<Vehicle>> gathered;

    while (true) {
        if (rank == 0 && renderer) {
            while (auto ev = renderer->pollEvent()) {
                renderer->handleEvent(*ev);
                if (ev->is<sf::Event::Closed>()) running = false;
            }
        }

        int runFlag = running ? 1 : 0;
        MPI_Bcast(&runFlag, 1, MPI_INT, 0, MPI_COMM_WORLD);
        if (runFlag == 0) break;

        auto nowFrame = std::chrono::high_resolution_clock::now();
        float dt = std::chrono::duration<float>(nowFrame - tLastFrame).count();
        tLastFrame = nowFrame;
        if (rank != 0) dt = 0.f;
        MPI_Bcast(&dt, 1, MPI_FLOAT, 0, MPI_COMM_WORLD);
        dt = std::clamp(dt, 0.001f, 0.05f);
        if (dt > 0.0001f) fpsDisplay = fpsDisplay * 0.88f + (1.f / dt) * 0.12f;

        exchangeVehicles(dom, simulation.getVehiclesMutable(), rank, worldSize, MPI_COMM_WORLD);

        gatherAllToRank0(simulation, rank, worldSize, MPI_COMM_WORLD, gathered);

        if (rank == 0)
            simulation.updateTrafficLights(dt, gathered);

        broadcastTrafficLights(simulation, rank, MPI_COMM_WORLD);

        scatterFromRank0(gathered, simulation, rank, worldSize, MPI_COMM_WORLD);

        simulation.updateVehicles(simulation.getVehiclesMutable(), dt);

        gatherAllToRank0(simulation, rank, worldSize, MPI_COMM_WORLD, gathered);

        if (rank == 0)
            simulation.detectCollisions(gathered);

        if (rank == 0 && renderer) {
            auto& mv = simulation.getVehiclesMutable();
            std::swap(mv, gathered);
            renderer->clear();
            renderer->render(simulation, fpsDisplay, static_cast<int>(simulation.getVehicles().size()), worldSize);
            renderer->display();
            std::swap(mv, gathered);
        }

        scatterFromRank0(gathered, simulation, rank, worldSize, MPI_COMM_WORLD);

        ++frames;
    }

    if (rank == 0) {
        std::cout << "MPI rank 0 done. Frames: " << frames << "\n";
    }

    renderer.reset();
    MPI_Finalize();
    return 0;
}
