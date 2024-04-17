struct Solver{T1,T2}
    "The problem representation of the game via ParametricMCPs.ParametricMCP"
    mcp_problem_representation::T1
    "A named tuple collecting all the problem dimension infos"
    dimensions::T2
end

function Solver(
    game::TrajectoryGame,
    horizon;
    context_dimension = 0,
    compute_sensitivities = true,
    parametric_mcp_options = (;),
)
    dimensions = let
        state_blocks =
            [state_dim(game.dynamics, player_index) for player_index in 1:num_players(game)]
        state = sum(state_blocks)
        control_blocks =
            [control_dim(game.dynamics, player_index) for player_index in 1:num_players(game)]
        control = sum(control_blocks)
        (; state_blocks, state, control_blocks, control, context = context_dimension, horizon)
    end

    initial_state_symbolic =
        SymbolicUtils.make_variables(symbolic_backend, :x0, dimensions.state) |>
        to_blockvector(dimensions.state_blocks)

    xs_symbolic =
        SymbolicUtils.make_variables(symbolic_backend, :X, dimensions.state * horizon) |>
        to_vector_of_blockvectors(dimensions.state_blocks)

    us_symbolic =
        SymbolicUtils.make_variables(symbolic_backend, :U, dimensions.control * horizon) |>
        to_vector_of_blockvectors(dimensions.control_blocks)

    context_symbolic = SymbolicUtils.make_variables(symbolic_backend, :context, context_dimension)

    cost_per_player_symbolic = game.cost(xs_symbolic, us_symbolic, context_symbolic)

    equality_constraints_symbolic = let
        dynamics_constraints = mapreduce(vcat, 2:horizon) do t
            xs_symbolic[t] - game.dynamics(xs_symbolic[t - 1], us_symbolic[t - 1], t - 1)
        end
        initial_state_constraints = xs_symbolic[begin] - initial_state_symbolic
        [dynamics_constraints; initial_state_constraints]
    end

    inequality_constraints_symoblic = let
        environment_constraints =
        # Note: We unstack trajectories here so that we add environment constraints on the sub-state of each player.
        # TODO: If `get_constraints` were to also receive the dynamics, it could handle this special case internally
            mapreduce(
                vcat,
                pairs(unstack_trajectory((; xs = xs_symbolic, us = us_symbolic))),
            ) do (ii, trajectory)
                environment_constraints_ii = get_constraints(game.env, ii)
                # Note: we don't constraint the first state because we have no control authority over that anyway
                mapreduce(environment_constraints_ii, vcat, trajectory.xs[2:end])
            end
        # TODO: technically, we could handle the box constraints here in a smarter way the
        # avoid dual multipliers by directly adding them as bounds in the MCP. (thus
        # reducing the problem size)
        state_box_constraints = let
            g_state_box = get_constraints_from_box_bounds(state_bounds(game.dynamics))
            mapreduce(vcat, 2:horizon) do t
                g_state_box(xs_symbolic[t])
            end
        end
        control_box_constraints = let
            g_control_box = get_constraints_from_box_bounds(control_bounds(game.dynamics))
            mapreduce(vcat, 1:horizon) do t
                g_control_box(us_symbolic[t])
            end
        end
        [
            environment_constraints
            state_box_constraints
            control_box_constraints
        ]
    end

    if isnothing(game.coupling_constraints)
        coupling_constraints_symbolic =
            SymbolicUtils.make_variables(symbolic_backend, :coupling_constraints, 0)
    else
        # Note: we don't constraint the first state because we have no control authority over that anyway
        coupling_constraints_symbolic =
            game.coupling_constraints(xs_symbolic[(begin + 1):end], us_symbolic)
    end

    # set up the duals for all constraints
    # private constraints
    μ_private_symbolic =
        SymbolicUtils.make_variables(symbolic_backend, :μ, length(equality_constraints_symbolic))

    #λ_private_symbolic =
    #    Symbolics.@variables(λ_private[1:length(inequality_constraints_symoblic)]) |>
    #    only |>
    #    scalarize
    λ_private_symbolic = SymbolicUtils.make_variables(
        symbolic_backend,
        :λ_private,
        length(inequality_constraints_symoblic),
    )

    # shared constraints
    λ_shared_symbolic = SymbolicUtils.make_variables(
        symbolic_backend,
        :λ_shared,
        length(coupling_constraints_symbolic),
    )

    # multiplier scaling per player as a runtime parameter
    # TODO: technically, we could have this scaling for *every* element of the constraint and
    # actually every constraint but for now let's keep it simple
    shared_constraint_premultipliers_symbolic =
        SymbolicUtils.make_variables(symbolic_backend, :γ_scaling, num_players(game))

    private_variables_per_player_symbolic =
        flatten_trajetory_per_player((; xs = xs_symbolic, us = us_symbolic))

    ∇lagragian_per_player_symbolic = map(
        cost_per_player_symbolic,
        private_variables_per_player_symbolic,
        shared_constraint_premultipliers_symbolic,
    ) do cost_ii, τ_ii, γ_ii
        # Note: this "Lagrangian" for player ii is *not* exactly their Lagrangian because it contains private constraints of the opponent.
        #       *However*: after taking the gradient w.r.t player ii's private variables, those terms will disappear
        L_ii =
            cost_ii + μ_private_symbolic' * equality_constraints_symbolic -
            λ_private_symbolic' * inequality_constraints_symoblic -
            λ_shared_symbolic' * coupling_constraints_symbolic * γ_ii

        SymbolicUtils.gradient(L_ii, τ_ii)
    end

    # set up the full KKT system as an MCP
    f_symbolic = [
        ∇lagragian_per_player_symbolic...
        equality_constraints_symbolic
        inequality_constraints_symoblic
        coupling_constraints_symbolic
    ]

    z_symbolic = [
        private_variables_per_player_symbolic...
        μ_private_symbolic
        λ_private_symbolic
        λ_shared_symbolic
    ]
    θ_symbolic = compose_parameter_vector(;
        initial_state = initial_state_symbolic,
        context = context_symbolic,
        shared_constraint_premultipliers = shared_constraint_premultipliers_symbolic,
    )

    number_of_primal_decision_variables =
        sum(length(p) for p in private_variables_per_player_symbolic)
    lower_bounds = [
        fill(-Inf, number_of_primal_decision_variables + length(equality_constraints_symbolic))
        fill(0.0, length(inequality_constraints_symoblic) + length(coupling_constraints_symbolic))
    ]
    upper_bounds = fill(Inf, length(lower_bounds))

    mcp_problem_representation = ParametricMCPs.ParametricMCP(
        f_symbolic,
        z_symbolic,
        θ_symbolic,
        lower_bounds,
        upper_bounds;
        compute_sensitivities,
        parametric_mcp_options...,
    )

    Solver(mcp_problem_representation, dimensions)
end

function flatten_trajetory_per_player(trajectory)
    trajectory_per_player = unstack_trajectory(trajectory)
    [flatten_trajectory(trajectory) for trajectory in trajectory_per_player]
end

function compose_parameter_vector(; initial_state, context, shared_constraint_premultipliers)
    [initial_state; context; shared_constraint_premultipliers]
end

