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

function Solver(game::TrajectoryGame;
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

    state = let
        Symbolics.@variables(x0[1:(dimensions.state)]) |>
        only |>
        scalarize |>
        to_blockvector(dimensions.state_blocks)
    end

end
