# FluxRT Project Rules

## Scope

These rules apply to this project directory and all descendants.

## Required orientation

Before making a material change, read:

1. `docs/PROJECT_OPERATION_LOG.md` current handoff and latest record;
2. `docs/REMEDIATION_ARCHITECTURE_ALLOCATION.md` for code ownership;
3. `docs/ST_VESC_ENGINEERING_IMPROVEMENT_ROADMAP.md` for priority and acceptance gates;
4. the current source and configuration files affected by the task.

Current source code is authoritative when a document and the implementation disagree. Record the
discrepancy and update the relevant document in the same task when appropriate.

## Mandatory operation log

Before reporting completion of any material project operation, update
`docs/PROJECT_OPERATION_LOG.md`. Material operations include code, configuration, build-system,
documentation, dependency, simulation, test, hardware, flashing, debugging, Git, and release work.

- Keep records newest-first and assign one stable `LOG-YYYYMMDD-NNN` ID per coherent operation.
- Update the current handoff block so the next worker can see the verified state and next action.
- Record the goal, exact paths changed, important parameter changes, commands/checks, result,
  evidence level, risks, rollback point, remaining work, and next recommended action.
- Distinguish static inspection, host tests, target build, simulation, flashing, and physical motor
  proof. Never promote one evidence level into another.
- For hardware runs, record board/motor, supply voltage and current limit, load state, firmware
  identity, command, duration, observed faults, and shutdown/recovery state.
- Never invent a Git hash. If the directory is not in Git or no commit was created, say so exactly.
- Do not delete or rewrite historical entries to hide mistakes. Add a correction to the affected
  entry or create a newer corrective record.
- Do not paste secrets or excessive raw logs; link to durable artifacts and summarize the evidence.

## Architecture and safety

- Place new work according to `docs/REMEDIATION_ARCHITECTURE_ALLOCATION.md`.
- Keep MCU register/HAL work in C platform code, deterministic control composition in
  `foc-control`, pure algorithms in `foc-algorithm`, and C ABI conversion in `foc-rt-bridge`.
- Keep all new control features disabled by default until their documented validation gate passes.
- Preserve immediate C-side hardware shutdown and output validation.
- Do not mix a structural refactor with control-law, timing, and hardware changes in one step unless
  the user explicitly requests it and the operation record explains why.

## Git and recovery

This project was not inside a Git work tree when this file was created. Do not initialize a
repository, modify remotes, create commits, rewrite history, or create tags unless the user has
authorized that Git action.

When Git is available:

- record the starting branch and base commit before changes;
- preserve unrelated user changes;
- prefer small commits aligned with operation-log records;
- record associated implementation commits or state explicitly that work remains uncommitted;
- use recoverable `git revert` or a recovery branch instead of destructive reset for rollback;
- record milestone tags only when they actually exist.

## Verification

Use the smallest relevant checks after each step. Structural or firmware changes should normally
run `test.ps1` and the appropriate target build; documentation-only changes require link and path
validation. Record exactly what ran and what did not run.
