# TODO: move these to TrajectoryGamesBase or another

function to_blockvector(block_dimensions)
    function (data)
        BlockArrays.BlockArray(data, block_dimensions)
    end
end

function to_vector_of_vectors(vector_dimension)
    function (z)
        reshape(z, vector_dimension, :) |> eachcol .|> collect
    end
end

function to_vector_of_blockvectors(block_dimensions)
    vector_dimension = sum(block_dimensions)
    function (z)
        z |> to_vector_of_vectors(vector_dimension) .|> to_blockvector(block_dimensions) |> collect
    end
end

function TrajectoryGamesBase.state_dim(dynamics::TrajectoryGamesBase.ProductDynamics, player_index)
    TrajectoryGamesBase.state_dim(dynamics.subsystems[player_index])
end

# TODO: patch TrajectoryGamesBase to support this directly
function TrajectoryGamesBase.state_dim(game, player_index)
    TrajectoryGamesBase.state_dim(game.dynamics, player_index)
end

function TrajectoryGamesBase.control_dim(game, player_index)
    TrajectoryGamesBase.control_dim(game.dynamics, player_index)
end
