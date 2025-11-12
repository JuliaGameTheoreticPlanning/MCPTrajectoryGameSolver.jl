module MCPTrajectoryGameSolver

using TrajectoryGamesBase:
    control_bounds,
    control_dim,
    flatten_trajectory,
    get_constraints,
    get_constraints_from_box_bounds,
    num_players,
    OpenLoopStrategy,
    stack_trajectories,
    state_bounds,
    state_dim,
    to_blockvector,
    to_vector_of_blockvectors,
    TrajectoryGame,
    TrajectoryGamesBase,
    unflatten_trajectory,
    unstack_trajectory

using MixedComplementarityProblems: MixedComplementarityProblems as IPMCPs, SymbolicTracingUtils

using BlockArrays: BlockArrays, mortar, blocks, eachblock
using ChainRulesCore: ChainRulesCore

include("solver_setup.jl")
include("solve.jl")

export Solver

end # module MCPTrajectoryGameSolver
