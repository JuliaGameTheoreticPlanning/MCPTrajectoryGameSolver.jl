module Demo

using TrajectoryGamesBase:
    TrajectoryGamesBase,
    TrajectoryGame,
    TrajectoryGameCost,
    ProductDynamics,
    GeneralSumCostStructure,
    PolygonEnvironment,
    solve_trajectory_game!,
    RecedingHorizonStrategy,
    rollout
using TrajectoryGamesExamples: planar_double_integrator
using BlockArrays: blocks, mortar
using MCPTrajectoryGameSolver: Solver
using GLMakie: GLMakie
using Zygote: Zygote
# using ParametricMCPs: ParametricMCPs

"""
Set up a simple two-player collision-avoidance game:
   - each player wants to reach their own goal position encoded by the `context` vector
   - both players want to avoid collisions
"""
function simple_game(; collision_avoidance_radius = 1)
    dynamics = let
        single_player_dynamics = planar_double_integrator()
        ProductDynamics([single_player_dynamics, single_player_dynamics])
    end

    cost = TrajectoryGameCost(GeneralSumCostStructure()) do xs, us, context
        g1 = context[1:2]
        g2 = context[3:4]

        sum(zip(xs, us)) do (x, u)
            x1, x2 = blocks(x)
            u1, u2 = blocks(u)
            d1 = x1[1:2] - g1
            d2 = x2[1:2] - g2
            p1_cost = d1' * d1 + 0.05 * u1' * u1
            p2_cost = d2' * d2 + 0.05 * u2' * u2
            [p1_cost, p2_cost]
        end
    end

    environment = PolygonEnvironment()

    function coupling_constraint(xs, us)
        map(xs) do x
            x1, x2 = blocks(x)
            dx = x1[1:2] - x2[1:2]
            dx' * dx - collision_avoidance_radius^2
        end
    end

    TrajectoryGame(dynamics, cost, environment, coupling_constraint)
end

function demo_model_predictive_game_play()
    simulation_horizon = 50
    game = simple_game()
    initial_state = mortar([[-1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]])
    context = let
        goal_p1 = [1.0, -0.1] # slightly offset goal to break symmetry
        goal_p2 = -goal_p1
        [goal_p1; goal_p2]
    end
    planning_horizon = 10
    solver = Solver(game, planning_horizon; context_dimension = length(context))

    receding_horizon_strategy = RecedingHorizonStrategy(;
        solver,
        game,
        solve_kwargs = (; context),
        turn_length = 2,
        # TODO: we could also provide this as a more easy-to-use utility, maybe even via dispatch
        # TODO: potentially allow the user to only warm-start the primals and or add noise
        generate_initial_guess = function (last_strategy, state, time)
            # only warm-start if the last strategy is converged / feasible
            if !isnothing(last_strategy) &&
               # last_strategy.info.raw_solution.status == ParametricMCPs.PATHSolver.MCP_Solved
               last_strategy.info.raw_solution.status === :solved
                initial_guess =
                    (; last_strategy.info.raw_solution.x, last_strategy.info.raw_solution.y)
            else
                nothing
            end
        end,
    )

    # Set up the visualization in terms of `GLMakie.Observable` objectives for reactive programming
    figure = GLMakie.Figure()
    GLMakie.plot(
        figure[1, 1],
        game.env;
        color = :lightgrey,
        axis = (; aspect = GLMakie.DataAspect(), title = "Model predictive game play demo"),
    )
    joint_strategy =
        GLMakie.Observable(solve_trajectory_game!(solver, game, initial_state; context))
    GLMakie.plot!(figure[1, 1], joint_strategy)
    for (player, color) in enumerate([:red, :blue])
        GLMakie.scatter!(
            figure[1, 1],
            GLMakie.@lift(GLMakie.Point2f($joint_strategy.substrategies[player].xs[begin]));
            color,
        )
    end
    display(figure)

    # visualization callback to update the observables on the fly
    function get_info(strategy, state, time)
        joint_strategy[] = strategy.receding_horizon_strategy
        sleep(0.1) # so that there's some time to see the visualization
        nothing # what ever we return here will be stored in `trajectory.infos` in case you need it for later inspection
    end

    # simulate the receding horizon strategy
    trajectory = rollout(
        game.dynamics,
        receding_horizon_strategy,
        initial_state,
        simulation_horizon;
        get_info,
    )
end

function demo_inverse_game()
    game = simple_game()
    planning_horizon = 10
    solver = Solver(game, planning_horizon; context_dimension = 4)
    initial_state = mortar([[-1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]])
    # both players want to reach a goal position at (0, 1))
    context = [0.0, 1.0, 0.0, 1.0]
    initial_joint_strategy = solve_trajectory_game!(solver, game, initial_state; context)

    # to demonstrate the differentiability, let us use gradient descent to find
    # goal positions that minimize each players' control effort
    function loss(context)
        # Zygote's reverse mode AD doesn't play well will some of the mutation in `solve_trajectory_game!`. Hence, we use forward mode here.
        # Note: When combining differentiable games with neural networks, it is advisable
        # to use mixed-mode AD: reverse-mode AD for the neural network, forward mode for the game.
        Zygote.forwarddiff(context) do context
            joint_strategy = solve_trajectory_game!(solver, game, initial_state; context)
            sum(joint_strategy.substrategies) do substrategy
                sum(substrategy.us) do u
                    u' * u
                end
            end
        end
    end

    context_estimate = context
    number_of_gradient_steps = 100
    learning_rate = 1e-2
    for iteration in 1:number_of_gradient_steps
        ∇context = only(Zygote.gradient(loss, context_estimate))
        context_estimate -= learning_rate * ∇context
    end

    final_joint_strategy =
        solve_trajectory_game!(solver, game, initial_state; context = context_estimate)

    # visualize the solution...
    # ...for the initial context estimate
    figure = GLMakie.Figure()
    GLMakie.plot(
        figure[1, 1],
        game.env;
        axis = (;
            aspect = GLMakie.DataAspect(),
            title = "Game solution for initial context estimate",
        ),
    )
    GLMakie.plot!(figure[1, 1], initial_joint_strategy)
    # ...and the optimized context estimate
    GLMakie.plot(
        figure[1, 2],
        game.env;
        axis = (;
            aspect = GLMakie.DataAspect(),
            title = "Game solution for optimized context estimate",
        ),
    )
    GLMakie.plot!(figure[1, 2], final_joint_strategy)
    display(figure)

    # trivially, we find that we can minimize each player's control input by setting
    # their goal positions to the initial positions
    @show (context_estimate - initial_state[[1, 2, 5, 6]])
end
end
