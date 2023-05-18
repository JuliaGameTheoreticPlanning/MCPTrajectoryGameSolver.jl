using MCPTrajectoryGameSolver
using TrajectoryGamesBase
using Test: @test, @testset
using BlockArrays: Block, eachblock, mortar
using LinearAlgebra: norm, norm_sqr
using TrajectoryGamesExamples: planar_double_integrator
using StatsBase: mean

function shared_collision_avoidance_coupling_constraints(num_players, min_distance)
    function coupling_constraint(xs, us)
        mapreduce(vcat, 1:(num_players - 1)) do player_i
            mapreduce(vcat, (player_i + 1):num_players) do paired_player
                map(xs) do x
                    my_norm_sqr(x[Block(player_i)][1:2] - x[Block(paired_player)][1:2]) -
                    min_distance^2
                end
            end
        end
    end
end

function my_norm_sqr(x)
    x' * x
end

function two_player_guidance_game_with_collision_avoidance(;
    environment,
    min_distance = 1.0,
    hard_constraints = true,
    collision_avoidance_coefficient = 0,
    dynamics = planar_double_integrator(;
        state_bounds = (; lb = [-Inf, -Inf, -0.8, -0.8], ub = [Inf, Inf, 0.8, 0.8]),
        control_bounds = (; lb = [-10, -10], ub = [10, 10]),
    ),
)
    cost = let
        function target_cost(x, context_state)
            norm_sqr(x[1:2] - context_state[1:2])
        end
        function control_cost(u)
            norm_sqr(u)
        end
        function cost_for_player1(xs, us, context_state)
            mean_target = mean(map(xs) do x
                target_cost(x[Block(1)], context_state)
            end)
            control = mean(map(us) do u
                control_cost(u[Block(1)])
            end)
            # safe_distance_violation = mean(map(xs) do x
            #     distance = norm(x[Block(1)][1:2] - x[Block(2)][1:2])
            #     max(0, min_distance - distance)
            # end)
            safe_distance_violation = mean(
                map(xs) do x
                    max(0, min_distance + 0.2 - norm(x[Block(1)][1:2] - x[Block(2)][1:2]))^3
                    # 1 / sqrt(norm(x[Block(1)][1:2] - x[Block(2)][1:2]) + 1e-5)
                end,
            )
            1.0 * mean_target +
            0.1 * control +
            collision_avoidance_coefficient * safe_distance_violation
        end
        function cost_for_player2(xs, us)
            mean_target = mean(map(xs) do x
                # direction_vector = x[Block(2)] - x[Block(1)]
                # direction_vector = 0.55 * direction_vector/norm(direction_vector)
                # target_cost(x[Block(2)], x[Block(1)] - direction_vector)
                target_cost(x[Block(2)], x[Block(1)])
            end)
            control = mean(map(us) do u
                control_cost(u[Block(2)])
            end)
            # safe_distance_violation = mean(map(xs) do x
            #     distance = norm(x[Block(1)][1:2] - x[Block(2)][1:2])
            #     max(0, min_distance - distance)
            # end)
            safe_distance_violation = mean(
                map(xs) do x
                    # 1 / sqrt(norm(x[Block(1)][1:2] - x[Block(2)][1:2]) + 1e-5)
                    max(0, min_distance + 0.2 - norm(x[Block(1)][1:2] - x[Block(2)][1:2]))^3
                end,
            )
            1.0 * mean_target +
            0.1 * control +
            collision_avoidance_coefficient * safe_distance_violation
        end
        function cost_function(xs, us, context_state)
            cost1 = cost_for_player1(xs, us, context_state)
            cost2 = cost_for_player2(xs, us)
            [cost1, cost2]
        end
        TrajectoryGameCost(cost_function, GeneralSumCostStructure())
    end
    dynamics = ProductDynamics([dynamics for _ in 1:2])
    coupling_constraints =
        hard_constraints ? shared_collision_avoidance_coupling_constraints(2, min_distance) :
        nothing
    TrajectoryGame(dynamics, cost, environment, coupling_constraints)
end

function main()
    environment = TrajectoryGamesBase.PolygonEnvironment(4, 4)
    game = two_player_guidance_game_with_collision_avoidance(; environment)
    horizon = 20
    context_dimension = 2
    solver = MCPTrajectoryGameSolver.Solver(game, horizon; context_dimension)
    initial_state = mortar([[1.0, 0, 0, 0], [-1.0, 0, 0, 0]])

    (; game, solver, initial_state)
end
