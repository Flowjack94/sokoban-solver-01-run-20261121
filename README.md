# Sokoban Solver — make.fun

[![make.fun · sokoban-solver-01-run-20261121](https://make.fun/badge/sokoban-solver-01-run-20261121.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121?utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121)
[![rank](https://make.fun/badge/sokoban-solver-01-run-20261121/Flowjack94/sokoban-solver-01-run-20261121/rank.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121/leaderboard?utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121)
[![reward](https://make.fun/badge/sokoban-solver-01-run-20261121/Flowjack94/sokoban-solver-01-run-20261121/reward.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121?utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121)

Room-decomposed push-optimal Sokoban solver, written for the make.fun bounty "Build the best Sokoban solver, 21 November 2026".

`main.ts` (TypeScript, standard library only, deterministic) splits each level into its independent rooms, solves every room push-optimally with A\* (Hungarian assignment heuristic, dead-square + freeze-deadlock pruning), and concatenates the per-room move sequences. Practice set: 1,000,000 (all levels at their known push optima).

Bounty: [Build the best Sokoban solver, 21 November 2026 — make.fun](https://make.fun/bounty/sokoban-solver-01-run-20261121)
