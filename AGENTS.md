# FluxRT Project Rules

## Scope

These rules apply to this project directory and all descendants.

## Required orientation

Before making a material change, read:

1. `docs/工程操作日志.md` current handoff and latest record;
2. `docs/整改架构与职责分配.md` for code ownership;
3. `docs/ST与VESC工程改进路线图.md` for priority and acceptance gates;
4. the current source and configuration files affected by the task.

Current source code is authoritative when a document and the implementation disagree. Record the
discrepancy and update the relevant document in the same task when appropriate.

## Mandatory operation log

Before reporting completion of any material project operation, update
`docs/工程操作日志.md`. Material operations include code, configuration, build-system,
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

- Place new work according to `docs/整改架构与职责分配.md`.
- Keep MCU register/HAL work in C platform code, deterministic control composition in
  `foc-control`, pure algorithms in `foc-algorithm`, and C ABI conversion in `foc-rt-bridge`.
- Third-party protocol stacks must come from the protocol owner's official upstream repository or
  official generator output, be pinned to an exact reviewed tag/commit, and remain isolated under
  `third_party/` behind FluxRT ports/adapters. Do not reimplement third-party wire codecs or state
  machines, copy implementation files into FluxRT modules, or edit vendored upstream code. If no
  reusable official implementation exists, keep the feature disabled and deferred.
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

## Mandatory post-change review (standing rule)

**Re-read this section after any context compaction; it is a standing user instruction, not a
one-off.**

After writing any code, before claiming the step is done, review your own change from every angle
and **report the review explicitly** — a bare "reviewed it" is not acceptable. Walk at least these
seven checks and state the outcome of each:

1. **Logic correctness** — does the change do what it claims on every path, including the failure
   paths? Are there off-by-one, ordering or sequence-domain mistakes?
2. **Consistency with callers and callees** — do the units, sequence counters, status codes and
   ownership expectations still match on both sides of every boundary touched?
3. **No regression of existing behaviour** — *this is the highest priority.* Prove it, do not
   assume it: prefer a diff that is additions-only, confirm no unrelated branch was modified, and
   where the change could plausibly interact with existing paths, **re-run those paths on the
   target** and record the result. A change that fixes something by breaking something else is a
   failure, not progress.
4. **ABI and layout drift** — no field reordering, no size or offset change without an explicit
   version decision and a static assertion.
5. **Safety invariants intact** — the change must not weaken the arm gate, the hardware fault
   checks, the fail-closed paths or the single-writer rules. State which invariants were checked.
6. **Failure paths fail closed** — every new error branch must clear output, latch or refuse, and
   must not silently continue.
7. **Test coverage** — new paths need new coverage; existing coverage must still pass. Record what
   ran and what did not run.
8. **Diagnostics must not perturb what they measure** — never call `rt_kprintf` or any other
   blocking/formatting function from the realtime ISR, *including* a "print only on failure" form,
   because the failure path runs inside the same ISR. On the G431 a single per-tick print inflated
   the same measurement by 67–109× and, by overrunning the ISR, tripped the tail deadline check,
   latched a fault and advanced `fault_epoch` — manufacturing a fault that looked like it belonged
   to the code under test. Write diagnostics into RAM fields and emit them from a thread after the
   ISR has exited. When a measurement looks implausible, suspect the instrumentation before the
   subject, and say which of the two a number can actually support.

If a check cannot be satisfied, say so plainly and stop rather than declaring success. When a
probe or measurement change turns out to be ineffective, remove it from the diff instead of
leaving dead or misleading code behind.

