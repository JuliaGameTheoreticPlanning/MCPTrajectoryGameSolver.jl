using MCPTrajectoryGameSolver: MCPTrajectoryGameSolver, OpenLoopStrategy

using TrajectoryGamesBase
using Test: @test, @testset, @test_throws
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
            safe_distance_violation = mean(
                map(xs) do x
                    max(0, min_distance + 0.2 - norm(x[Block(1)][1:2] - x[Block(2)][1:2]))^3
                end,
            )
            1.0 * mean_target +
            0.1 * control +
            collision_avoidance_coefficient * safe_distance_violation
        end
        function cost_for_player2(xs, us)
            mean_target = mean(map(xs) do x
                target_cost(x[Block(2)], x[Block(1)])
            end)
            control = mean(map(us) do u
                control_cost(u[Block(2)])
            end)
            safe_distance_violation = mean(
                map(xs) do x
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

function isfeasible(game::TrajectoryGamesBase.TrajectoryGame, trajectory; tol = 1e-4)
    isfeasible(game.dynamics, trajectory; tol) &&
        isfeasible(game.env, trajectory; tol) &&
        all(game.coupling_constraints(trajectory.xs, trajectory.us) .>= 0 - tol)
end

function isfeasible(dynamics::TrajectoryGamesBase.AbstractDynamics, trajectory; tol = 1e-4)
    dynamics_steps_consistent = all(
        map(2:length(trajectory.xs)) do t
            residual =
                trajectory.xs[t] - dynamics(trajectory.xs[t - 1], trajectory.us[t - 1], t - 1)
            sum(abs, residual) < tol
        end,
    )

    state_bounds_feasible = let
        bounds = TrajectoryGamesBase.state_bounds(dynamics)
        all(map(trajectory.xs) do x
            all(bounds.lb .- tol .<= x .<= bounds.ub .+ tol)
        end)
    end

    control_bounds_feasible = let
        bounds = TrajectoryGamesBase.control_bounds(dynamics)
        all(map(trajectory.us) do u
            all(bounds.lb .- tol .<= u .<= bounds.ub .+ tol)
        end)
    end

    dynamics_steps_consistent && state_bounds_feasible && control_bounds_feasible
end

function isfeasible(env::TrajectoryGamesBase.PolygonEnvironment, trajectory; tol = 1e-4)
    trajectory_per_player = MCPTrajectoryGameSolver.unstack_trajectory(trajectory)

    map(enumerate(trajectory_per_player)) do (ii, trajectory)
        constraints = TrajectoryGamesBase.get_constraints(env, ii)
        map(trajectory.xs) do x
            all(constraints(x) .>= 0 - tol)
        end |> all
    end |> all
end

function main()
    environment = TrajectoryGamesBase.PolygonEnvironment(4, 4)
    game = two_player_guidance_game_with_collision_avoidance(; environment)
    horizon = 2
    context = [1, 1]
    initial_state = mortar([[1.0, 0, 0, 0], [-1.0, 0, 0, 0]])
    tol = 1e-4

    local solver, solver_wrong_context

    @testset "Tests" begin
        @testset "solver setup" begin
            solver =
                MCPTrajectoryGameSolver.Solver(game, horizon; context_dimension = length(context))
            solver_wrong_context =
                MCPTrajectoryGameSolver.Solver(game, horizon; context_dimension = 4)
        end

        @testset "solve" begin
            strategy =
                TrajectoryGamesBase.solve_trajectory_game!(solver, game, initial_state; context)
            @testset "input sanity" begin
                @test_throws ArgumentError TrajectoryGamesBase.solve_trajectory_game!(
                    solver,
                    game,
                    initial_state,
                )
                @test_throws ArgumentError TrajectoryGamesBase.solve_trajectory_game!(
                    solver_wrong_context,
                    game,
                    initial_state;
                    context,
                )
                @test_throws ArgumentError TrajectoryGamesBase.solve_trajectory_game!(
                    solver,
                    game,
                    initial_state;
                    context,
                    shared_constraint_premultipliers = [1],
                )
            end

            @testset "solution sanity" begin
                nash_trajectory =
                    TrajectoryGamesBase.rollout(game.dynamics, strategy, initial_state, horizon)
                nash_cost = game.cost(nash_trajectory.xs, nash_trajectory.us, context)

                @test isfeasible(game, nash_trajectory)

                for ii in 1:TrajectoryGamesBase.num_players(game)
                    for t in 1:horizon
                        perturbed_inputs = deepcopy(nash_trajectory.us)
                        perturbed_inputs[t][Block(ii)] .+= tol
                        perturbed_strategy = (state, time) -> perturbed_inputs[time]
                        perturbed_trajectory = TrajectoryGamesBase.rollout(
                            game.dynamics,
                            perturbed_strategy,
                            initial_state,
                            horizon,
                        )
                        perturbed_cost =
                            game.cost(perturbed_trajectory.xs, perturbed_trajectory.us, context)
                        @test perturbed_cost[ii] - nash_cost[ii] >= -tol
                    end
                end
            end
        end
    end
end

main()
