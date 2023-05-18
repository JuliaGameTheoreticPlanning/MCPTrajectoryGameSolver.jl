# ParametricMCPs

[![CI](https://github.com/JuliaGameTheoreticPlanning/MCPTrajectoryGameSolver.jl/actions/workflows/ci.yml/badge.svg)](https://github.com/JuliaGameTheoreticPlanning/MCPTrajectoryGameSolver.jl/actions/workflows/ci.yml)
[![codecov](https://codecov.io/gh/JuliaGameTheoreticPlanning/MCPTrajectoryGameSolver.jl/branch/main/graph/badge.svg?token=knLJ9hVfeO)](https://codecov.io/gh/JuliaGameTheoreticPlanning/MCPTrajectoryGameSolver.jl)
[![License](https://img.shields.io/badge/license-MIT-blue)](https://opensource.org/licenses/MIT)

This package provides a thin wrapper around the [ParametricMCPs.jl](https://github.com/lassepe/ParametricMCPs.jl) package to solve trajectory games from
[TrajectoryGamesBase.jl](https://github.com/lassepe/TrajectoryGamesBase.jl).

By exploiting the implicit function theorem, game solutions can be differentiated with respect to the game parameters.
This sensitivity information can be used to fit a game-theoretic model to observed behavior as we explore in our work [Learning to Play Trajectory Games Against Opponents with Unknown Objectives](https://arxiv.org/pdf/2211.13779.pdf).

## Quickstart

> :warning: This package is not yet registered.
> 
> To install it, run `] add https://github.com/JuliaGameTheoreticPlanning/MCPTrajectoryGameSolver.jl`.

This README will be expanded to include more concrete usage instructions. Until then, please refer to the `test/runtests.jl` file as a source of implicit documentation.

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
