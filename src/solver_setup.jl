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

    equality_constraints = let
        dynamics_constraints = mapreduce(vcat, 2:horizon) do t
            xs_symbolic[t] - game.dynamics(xs_symbolic[t-1], us_symbolic[t-1], t - 1)
        end
        initial_state_constraints = xs_symbolic[begin] - initial_state_symbolic
        [dynamics_constraints; initial_state_constraints]
    end

    inequality_constraints = let
        environment_constraints =
        # Note: We unstack trajectories here so that we add environment constraints on the sub-state of each player.
        # TODO: If `get_constraints` were to also receive the dynamics, it could handle this special case internally
            mapreduce(vcat, pairs(unstack_trajectory((; xs=xs_symbolic, us=us_symbolic)))) do (ii, trajectory)
                environment_constraints_ii = get_constraints(game.env, ii)
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



end
