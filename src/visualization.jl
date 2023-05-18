# TODO: move this into an extension
Makie.@recipe(OpenLoopStrategyViz) do scene
    Makie.Attributes(; line_attributes = nothing, scatter_attributes = nothing)
end

Makie.plottype(::OpenLoopStrategy) = OpenLoopStrategyViz

function Makie.plot!(viz::OpenLoopStrategyViz{<:Tuple{OpenLoopStrategy}})
    strategy = viz[1]

    points = Makie.@lift [Makie.Point2f(x[1:2]) for x in $strategy.xs]

    if isnothing(viz.line_attributes[])
        line_kwargs = (;)
    else
        line_kwargs = map(keys(viz.line_attributes[])) do key
            key => Makie.@lift($(viz.line_kwargs)[key])
        end |> NamedTuple
    end

    if isnothing(viz.scatter_attributes[])
        scatter_kwargs = (;)
    else
        scatter_kwargs = map(keys(viz.scatter_attributes[])) do key
            key => Makie.@lift($(viz.scatter_kwargs)[key])
        end |> NamedTuple
    end

    Makie.lines!(viz, points; line_kwargs...)
    Makie.scatter!(viz, points; scatter_kwargs...)

    # assuming that the number of players does not change between calls
    for player_index in eachindex(strategy[].xs)
        xs = Makie.Observable{Any}(nothing)
        us = Makie.Observable{Any}(nothing)
        Makie.on(strategy) do strategy
            xs[] = strategy.xs[player_index]
            us[] = strategy.us[player_index]
        end
        strategy[] = strategy[]
    end

    viz
end

#=== Joint Strategy Visualization ===#

Makie.@recipe(JointStrategyViz) do scene
    Makie.Attributes(; substrategy_attributes = nothing)
end

Makie.plottype(::TrajectoryGamesBase.JointStrategy) = JointStrategyViz

function Makie.plot!(viz::JointStrategyViz{<:Tuple{TrajectoryGamesBase.JointStrategy}})
    joint_strategy = viz[1]
    # assuming that the number of players does not change between calls
    for substrategy_index in eachindex(joint_strategy[].substrategies)
        substrategy = Makie.Observable{Any}(nothing)
        Makie.on(joint_strategy) do joint_strategy
            substrategy[] = joint_strategy.substrategies[substrategy_index]
        end
        if isnothing(viz.substrategy_attributes[])
            kwargs = (;)
        else
            kwargs =
                map(keys(viz.substrategy_attributes[][substrategy_index])) do key
                    key => Makie.@lift($(viz.substrategy_attributes)[substrategy_index][key])
                end |> NamedTuple
        end
        joint_strategy[] = joint_strategy[]
        Makie.plot!(viz, substrategy; kwargs...)
    end

    viz
end
