#pragma once

#include "interfaces/ballistic_solver.hpp"
#include "ballistic_table.hpp"

class TableSolver : public IBallisticSolver {
    DroneConfig droneConfig;
    BallisticTable table;

    public:
        TableSolver(const DroneConfig& config, std::string table_path);

        auto  solve(
            const Coord& dronePosition,
            const Coord& targetPosition,
            const AmmoParams& ammo
        ) -> Coord override;      
};