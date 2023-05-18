function TrajectoryGamesBase.solve_trajectory_game!(
    solver,
    game,
    initial_state;
    shared_constraint_premultipliers = ones(num_players(game)),
    context = Float64[],
    # TODO: also provide logic for generating a good initial guess if the user has now provided anything
    initial_guess = nothing,
    parametric_mcp_solve_options = (;),
)
    length(shared_constraint_premultipliers) == num_players(game) ||
        error("Must provide one constraint multiplier per player")
    length(context) == solver.dimensions.context ||
        error("The context state must have the same dimension as the solver's context state")

    θ = compose_parameter_vector(; initial_state, context, shared_constraint_premultipliers)

    raw_solution = ParametricMCPs.solve(
        solver.mcp_problem_representation,
        θ;
        initial_guess = isnothing(initial_guess) ?
                        generate_initial_guess(solver, game, initial_state) : initial_guess,
        parametric_mcp_solve_options...,
    )

    strategy_from_raw_solution(; raw_solution, game, solver)
end

"""
Reshapes the raw solution into a `JointStrategy` over `OpenLoopStrategy`s.
"""
function strategy_from_raw_solution(; raw_solution, game, solver)
    number_of_players = num_players(game)
    z_iter = Iterators.Stateful(raw_solution.z)

    map(1:number_of_players) do player_index
        private_state_dimension = solver.dimensions.state_blocks[player_index]
        private_control_dimension = solver.dimensions.control_blocks[player_index]
        number_of_primals =
            solver.dimensions.horizon * (private_state_dimension + private_control_dimension)
        z_private = Iterators.take(z_iter, number_of_primals) |> collect
        trajectory =
            unflatten_trajectory(z_private, private_state_dimension, private_control_dimension)
        OpenLoopStrategy(trajectory.xs, trajectory.us)
    end |> TrajectoryGamesBase.JointStrategy
end

function generate_initial_guess(solver, game, initial_state)
    z_initial = zeros(ParametricMCPs.get_problem_size(solver.mcp_problem_representation))

    rollout_strategy =
        map(solver.dimensions.control_blocks) do control_dimension_player_i
            (x, t) -> zeros(control_dimension_player_i)
        end |> TrajectoryGamesBase.JointStrategy

    zero_input_trajectory = TrajectoryGamesBase.rollout(
        game.dynamics,
        rollout_strategy,
        initial_state,
        solver.dimensions.horizon,
    )

    copyto!(z_initial, reduce(vcat, flatten_trajetory_per_player(zero_input_trajectory)))

    z_initial
end
