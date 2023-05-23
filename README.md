# MCPTrajectoryGameSolver

[![CI](https://github.com/JuliaGameTheoreticPlanning/MCPTrajectoryGameSolver.jl/actions/workflows/ci.yml/badge.svg)](https://github.com/JuliaGameTheoreticPlanning/MCPTrajectoryGameSolver.jl/actions/workflows/ci.yml)
[![codecov](https://codecov.io/gh/JuliaGameTheoreticPlanning/MCPTrajectoryGameSolver.jl/branch/main/graph/badge.svg?token=C48T44SCP2)](https://codecov.io/gh/JuliaGameTheoreticPlanning/MCPTrajectoryGameSolver.jl)
[![License](https://img.shields.io/badge/license-MIT-blue)](https://opensource.org/licenses/MIT)

This package provides a thin wrapper around the [ParametricMCPs.jl](https://github.com/lassepe/ParametricMCPs.jl) package to solve trajectory games from
[TrajectoryGamesBase.jl](https://github.com/lassepe/TrajectoryGamesBase.jl).

By exploiting the implicit function theorem, game solutions can be differentiated with respect to the game parameters.
This sensitivity information can be used to fit a game-theoretic model to observed behavior as we explore in our work [Learning to Play Trajectory Games Against Opponents with Unknown Objectives](https://arxiv.org/pdf/2211.13779.pdf).

## Quickstart

Installation is as simple as running:

```julia
]add MCPTrajectoryGameSolver
```

For a full example of how to use this package, please consult the demo in `test/Demo.jl`:

```julia
using TestEnv # install globally with `] add TestEnv` if you don't have this
TestEnv.activate()
include("test/Demo.jl")
Demo.demo()
```

## Citiation

[![](https://xinjie-liu.github.io/assets/img/liu2023ral_teaser.png)](https://arxiv.org/pdf/2211.13779.pdf)

This package was originally developed as part of the paper [Learning to Play Trajectory Games Against Opponents with Unknown Objectives](https://arxiv.org/pdf/2211.13779.pdf).
Please cite this paper if you use this package in your research.

```bibtex
@article{liu2022learning,
  title={Learning to Play Trajectory Games Against Opponents with Unknown Objectives},
  author={Liu, Xinjie and Peters, Lasse and Alonso-Mora, Javier},
  journal={IEEE Robotics and Automation Letters (RA-L)},
  year={2023}
}
```
