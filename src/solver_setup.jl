struct Solver{T}
    """
    mcp_problem_representation
    dimensions

    Note: Just a NamedTuple for prototyping
    """
    fields::T
end

function Base.getproperty(solver::Solver, name::Symbol)
    if name === :fields
        Base.getfield(solver, name)
    else
        Base.getproperty(solver.fields, name)
    end
end

function Solver(game::TrajectoryGame, horizon;
    context_state_dimension=0,
    compute_sensitivies=false
)

    dimensions = let
        state_blocks = [
            state_dim(game.dynamics, player_index)
            for player_index in 1:num_players(game)
        ]
        state = sum(state_blocks)
        control_blocks = [
            control_dim(game.dynamics, player_index) for player_index in 1:num_players(game)
        ]
        control = sum(control_blocks)
        (; state_blocks, state, control_blocks, control)
    end

    initial_state_symbolic = let
        Symbolics.@variables(x0[1:(dimensions.state)]) |>
        only |>
        scalarize |>
        to_blockvector(dimensions.state_blocks)
    end

    xs_symbolic = let
        Symbolics.@variables(X[1:(dimensions.state*horizon)]) |>
        only |>
        scalarize |>
        to_vector_of_blockvectors(dimensions.state_blocks)
    end

    us_symbolic = let
        Symbolics.@variables(U[1:(dimensions.control*horizon)]) |>
        only |>
        scalarize |>
        to_vector_of_blockvectors(dimensions.control_blocks)
    end

    context_state_symbolic =
        Symbolics.@variables(context[1:context_state_dimension]) |> only |> scalarize

    cost_per_player_symbolic = game.cost(xs_symbolic, us_symbolic, context_state_symbolic)

    equality_constraints_symbolic = let
        dynamics_constraints = mapreduce(vcat, 2:horizon) do t
            xs_symbolic[t] - game.dynamics(xs_symbolic[t-1], us_symbolic[t-1], t - 1)
        end
        initial_state_constraints = xs_symbolic[begin] - initial_state_symbolic
        [dynamics_constraints; initial_state_constraints]
    end

    inequality_constraints_symoblic = let
        environment_constraints =
        # Note: We unstack trajectories here so that we add environment constraints on the sub-state of each player.
        # TODO: If `get_constraints` were to also receive the dynamics, it could handle this special case internally
            mapreduce(vcat, pairs(unstack_trajectory((; xs=xs_symbolic, us=us_symbolic)))) do (ii, trajectory)
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
        coupling_constraints_symbolic = Symbolics.Num[]
    else
        # Note: we don't constraint the first state because we have no control authority over that anyway
        coupling_constraints_symbolic =
            game.coupling_constraints(xs_symbolic[(begin+1):end], us_symbolic)
    end

    # set up the duals for all constraints
    # private constraints
    μ_private_symbolic = Symbolics.@variables(μ[1:length(equality_constraints_symbolic)]) |> only |> scalarize
    λ_private_symbolic = Symbolics.@variables(λ_private[1:length(inequality_constraints_symoblic)]) |> only |> scalarize
    # shared constraints
    λ_shared_symbolic = Symbolics.@variables(λ_shared[1:length(coupling_constraints_symbolic)]) |> only |> scalarize
    # multiplier scaling per player as a runtime parameter
    # TODO: technically, we could have this scaling for *every* element of the constraint and
    # actually every constraint but for now let's keep it simple
    shared_constraint_premultipliers_symbolic =
        Symbolics.@variables(γ_scaling[1:num_players(game)]) |> only |> scalarize


    private_variables_per_player_symbolic = flatten_trajetory_per_player((; xs=xs_symbolic, us=us_symbolic))
end

function flatten_trajetory_per_player(trajectory)
    trajectory_per_player = unstack_trajectory(trajectory)
    return trajectory_per_player
    [flatten_trajectory(trajectory) for trajectory in trajectory_per_player]
end
