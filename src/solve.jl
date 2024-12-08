function TrajectoryGamesBase.solve_trajectory_game!(
    solver,
    game,
    initial_state;
    shared_constraint_premultipliers = ones(num_players(game)),
    context = Float64[],
    initial_guess = nothing,
    parametric_mcp_solve_options = (; tol = 1e-7),
)
    length(shared_constraint_premultipliers) == num_players(game) ||
        throw(ArgumentError("Must provide one constraint multiplier per player"))
    length(context) == solver.dimensions.context || throw(
        ArgumentError(
            "The context state must have the same dimension as the solver's context state",
        ),
    )

    θ = compose_parameter_vector(; initial_state, context, shared_constraint_premultipliers)

    if isnothing(initial_guess)
        initial_guess = generate_initial_guess(solver, game, initial_state)
    else
        initial_guess = (; x₀ = initial_guess.x, y₀ = initial_guess.y)
    end

    raw_solution = IPMCPs.solve(
        IPMCPs.InteriorPoint(),
        solver.mcp_problem_representation,
        θ;
        initial_guess...,
        parametric_mcp_solve_options...,
    )

    strategy_from_raw_solution(; raw_solution, game, solver)
end

"""
Reshapes the raw solution into a `JointStrategy` over `OpenLoopStrategy`s.
"""
function strategy_from_raw_solution(; raw_solution, game, solver)
    number_of_players = num_players(game)
    # z_iter = Iterators.Stateful(raw_solution.z)
    z_iter = Iterators.Stateful(raw_solution.x)

    substrategies = map(1:number_of_players) do player_index
        private_state_dimension = solver.dimensions.state_blocks[player_index]
        private_control_dimension = solver.dimensions.control_blocks[player_index]
        number_of_primals =
            solver.dimensions.horizon * (private_state_dimension + private_control_dimension)
        z_private = Iterators.take(z_iter, number_of_primals) |> collect
        trajectory =
            unflatten_trajectory(z_private, private_state_dimension, private_control_dimension)
        OpenLoopStrategy(trajectory.xs, trajectory.us)
    end

    info = (; raw_solution)
    TrajectoryGamesBase.JointStrategy(substrategies, info)
end

function generate_initial_guess(solver, game, initial_state)
    ChainRulesCore.ignore_derivatives() do
        x_initial = zeros(solver.mcp_problem_representation.unconstrained_dimension)
        y_initial = zeros(solver.mcp_problem_representation.constrained_dimension)

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

        copyto!(x_initial, reduce(vcat, flatten_trajetory_per_player(zero_input_trajectory)))

        (; x₀ = x_initial, y₀ = y_initial)
    end
end
