# MCPTrajectoryGameSolver

[![CI](https://github.com/JuliaGameTheoreticPlanning/MCPTrajectoryGameSolver.jl/actions/workflows/ci.yml/badge.svg)](https://github.com/JuliaGameTheoreticPlanning/MCPTrajectoryGameSolver.jl/actions/workflows/ci.yml)
[![codecov](https://codecov.io/gh/JuliaGameTheoreticPlanning/MCPTrajectoryGameSolver.jl/branch/main/graph/badge.svg?token=C48T44SCP2)](https://codecov.io/gh/JuliaGameTheoreticPlanning/MCPTrajectoryGameSolver.jl)
[![License](https://img.shields.io/badge/license-MIT-blue)](https://opensource.org/licenses/MIT)

This package provides a thin wrapper around the [ParametricMCPs.jl](https://github.com/lassepe/ParametricMCPs.jl) package to solve trajectory games from
[TrajectoryGamesBase.jl](https://github.com/lassepe/TrajectoryGamesBase.jl).

By exploiting the implicit function theorem, game solutions can be differentiated with respect to the game parameters.
This sensitivity information can be used to fit a game-theoretic model to observed behavior as we explore in our work [Learning to Play Trajectory Games Against Opponents with Unknown Objectives](https://xinjie-liu.github.io/projects/game/).

## Quickstart

Installation is as simple as running:

```julia
]add MCPTrajectoryGameSolver
```

This package uses PATH solver (via [PATHSolver.jl](https://github.com/chkwon/PATHSolver.jl)) under the hood. Larger-sized problems require to have a license key. By courtesy of Steven Dirkse, Michael Ferris, and Tudd Munson, temporary license keys are available free of charge. For more details about the license key, please consult [PATHSolver.jl](https://github.com/chkwon/PATHSolver.jl) (License section). Note that, when no license is loaded, PATH does not report an informative error and instead may just report a wrong result. Thus, make sure that the license is loaded correctly before using the solver.

For a full example of how to use this package, please consult the demo in [`test/Demo.jl`](test/Demo.jl):

Start `julia --project` *from the repository root* and run the following commands:
```julia
using TestEnv, Revise # install globally with `] add TestEnv, Revise` if you don't have this
TestEnv.activate()
Revise.includet("test/Demo.jl")
Demo.demo_model_predictive_game_play() # example of receding-horizon interaction
Demo.demo_inverse_game() # example of fitting game parameters via differentiation of the game solver
```

## Citation

The original solver implementation and experiment code used in our research [Learning to Play Trajectory Games Against Opponents with Unknown Objectives](https://arxiv.org/pdf/2211.13779.pdf) can be found [here](https://github.com/xinjie-liu/DifferentiableAdaptiveGames.jl). This repository provides a more optimized implementation of the differentiable game solver. We kindly ask you to cite our paper if you use either of the implementations in your research. Thanks!

[![](https://xinjie-liu.github.io/assets/img/liu2023ral_teaser.png)](https://arxiv.org/pdf/2211.13779.pdf)



```bibtex
@ARTICLE{liu2022learning,
  author={Liu, Xinjie and Peters, Lasse and Alonso-Mora, Javier},
  journal={IEEE Robotics and Automation Letters}, 
  title={Learning to Play Trajectory Games Against Opponents With Unknown Objectives}, 
  year={2023},
  volume={8},
  number={7},
  pages={4139-4146},
  doi={10.1109/LRA.2023.3280809}}
```
<a href ="https://xinjie-liu.github.io/assets/pdf/Liu2023learningPoster(full).pdf"><img src="https://xinjie-liu.github.io/assets/img/liu2023ral_poster.png" width = "560" height = "396"></a>


