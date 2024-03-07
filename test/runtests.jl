using MCPTrajectoryGameSolver: MCPTrajectoryGameSolver, OpenLoopStrategy

using TrajectoryGamesBase
using Test: @test, @testset, @test_throws
using BlockArrays: Block, blocks, mortar
using LinearAlgebra: norm, norm_sqr
using TrajectoryGamesExamples: planar_double_integrator
using StatsBase: mean
using Zygote: Zygote
using FiniteDiff: FiniteDiff
using Random: Random

include("Demo.jl")

function isfeasible(game::TrajectoryGamesBase.TrajectoryGame, trajectory; tol=1e-4)
    isfeasible(game.dynamics, trajectory; tol) &&
        isfeasible(game.env, trajectory; tol) &&
        all(game.coupling_constraints(trajectory.xs, trajectory.us) .>= 0 - tol)
end

function isfeasible(dynamics::TrajectoryGamesBase.AbstractDynamics, trajectory; tol=1e-4)
    dynamics_steps_consistent = all(
        map(2:length(trajectory.xs)) do t
            residual =
                trajectory.xs[t] - dynamics(trajectory.xs[t-1], trajectory.us[t-1], t - 1)
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

function isfeasible(env::TrajectoryGamesBase.PolygonEnvironment, trajectory; tol=1e-4)
    trajectory_per_player = MCPTrajectoryGameSolver.unstack_trajectory(trajectory)

    map(enumerate(trajectory_per_player)) do (ii, trajectory)
        constraints = TrajectoryGamesBase.get_constraints(env, ii)
        map(trajectory.xs) do x
            all(constraints(x) .>= 0 - tol)
        end |> all
    end |> all
end

function input_sanity(; solver, solver_wrong_context, game, initial_state, context)
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
            context
        )
        @test_throws ArgumentError TrajectoryGamesBase.solve_trajectory_game!(
            solver,
            game,
            initial_state;
            context,
            shared_constraint_premultipliers=[1]
        )
    end
end

function forward_pass_sanity(; solver, game, initial_state, context, horizon, strategy, tol=1e-4)
    @testset "forwardpass sanity" begin
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

function backward_pass_sanity(;
    solver,
    game,
    initial_state,
    rng=Random.MersenneTwister(1),
    θs=[randn(rng, 4) for _ in 1:10]
)
    @testset "backward pass sanity" begin
        function loss(θ)
            Zygote.forwarddiff(θ) do θ
                strategy = TrajectoryGamesBase.solve_trajectory_game!(
                    solver,
                    game,
                    initial_state;
                    context=θ
                )

                sum(strategy.substrategies) do substrategy
                    sum(substrategy.xs) do x_ii
                        norm_sqr(x_ii[1:2])
                    end
                end
            end
        end

        for θ in θs
            ∇_zygote = Zygote.gradient(loss, θ) |> only
            ∇_finitediff = FiniteDiff.finite_difference_gradient(loss, θ)
            @test isapprox(∇_zygote, ∇_finitediff; atol=1e-4)
        end
    end
end

function main()
    game = Demo.simple_game()
    horizon = 2
    context = [0.0, 1.0, 0.0, 1.0]
    initial_state = mortar([[1.0, 0, 0, 0], [-1.0, 0, 0, 0]])

    local solver, solver_wrong_context

    @testset "Tests" begin
        @testset "solver setup" begin
            solver =
                MCPTrajectoryGameSolver.Solver(game, horizon; context_dimension=length(context))
            solver_wrong_context =
                MCPTrajectoryGameSolver.Solver(game, horizon; context_dimension=(length(context) + 1))
        end

        @testset "solve" begin
            input_sanity(; solver, solver_wrong_context, game, initial_state, context)
            strategy =
                TrajectoryGamesBase.solve_trajectory_game!(solver, game, initial_state; context)
            forward_pass_sanity(; solver, game, initial_state, context, horizon, strategy)
            backward_pass_sanity(; solver, game, initial_state)
        end

        @testset "integration test" begin
            Demo.demo()
        end
    end
end

main()
