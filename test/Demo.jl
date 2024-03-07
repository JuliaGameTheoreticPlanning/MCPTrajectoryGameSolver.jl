module Demo

using TrajectoryGamesBase:
    TrajectoryGame, TrajectoryGameCost, ProductDynamics, GeneralSumCostStructure, PolygonEnvironment, solve_trajectory_game!
using TrajectoryGamesExamples: planar_double_integrator
using BlockArrays: blocks, mortar
using MCPTrajectoryGameSolver: Solver
using GLMakie: GLMakie
using Zygote: Zygote

"""
Set up a simple two-player collision-avoidance game:
   - each player wants to reach their own goal position encoded by the `context` vector
   - both players want to avoid collisions
"""
function simple_game(; collision_avoidance_radius=1)
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

function demo()
    game = simple_game()
    horizon = 10
    solver = Solver(game, horizon; context_dimension=4)
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

    final_joint_strategy = solve_trajectory_game!(solver, game, initial_state; context=context_estimate)

    # visualize the solution...
    # ...for the initial context estimate
    figure = GLMakie.Figure()
    GLMakie.plot(figure[1, 1], game.env; axis=(; aspect=GLMakie.DataAspect(), title="Game solution for initial context estimate"))
    GLMakie.plot!(figure[1, 1], initial_joint_strategy)
    # ...and the optimized context estimate
    GLMakie.plot(figure[1, 2], game.env; axis=(; aspect=GLMakie.DataAspect(), title="Game solution for optimized context estimate"))
    GLMakie.plot!(figure[1, 2], final_joint_strategy)
    display(figure)

    # trivially, we find that we can minimize each player's control input by setting
    # their goal positions to the initial positions
    @show (context_estimate - initial_state[[1, 2, 5, 6]])
end
end
