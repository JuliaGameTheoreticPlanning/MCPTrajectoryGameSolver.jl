module MCPTrajectoryGameSolver

using TrajectoryGamesBase: TrajectoryGamesBase, TrajectoryGame, state_dim, control_dim, num_players
using Symbolics: Symbolics, scalarize
using ParametricMCPs: ParametricMCPs
using BlockArrays: BlockArrays, mortar, blocks, eachblock

include("utils.jl")
include("solver_setup.jl")

# TODO: think about the public API

end # module MCPTrajectoryGameSolver
