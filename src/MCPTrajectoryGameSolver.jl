module MCPTrajectoryGameSolver

using TrajectoryGamesBase:
    control_bounds,
    control_dim,
    get_constraints,
    num_players,
    state_bounds,
    state_dim,
    TrajectoryGame,
    TrajectoryGamesBase
using Symbolics: Symbolics
using ParametricMCPs: ParametricMCPs
using BlockArrays: BlockArrays, mortar, blocks, eachblock

include("utils.jl")
include("solver_setup.jl")

# TODO: think about the public API

end # module MCPTrajectoryGameSolver
