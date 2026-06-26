# Module Guides

This directory indexes module maintenance guides. Module guides describe code
ownership boundaries, public imports, compatibility policy, forbidden
dependencies, focused tests, and future split plans. They are maintenance
guides, not runtime entrypoints.

For current Real and Isaac user-facing startup commands, read
`docs/current_entrypoints.md`.

## Read Before Module Work

Before a Codex or human module task, read:

- `CODEX_RULES.md`
- `docs/current_entrypoints.md`
- `docs/module_guides/README.md`
- The relevant `docs/module_guides/<module>.md`, if it exists.
- The relevant `src/<module>/README.md`, if it exists.

## Existing Guides

- `docs/module_guides/grounding.md`: grounding module policy, public import
  paths, compatibility shims, forbidden dependencies, tests, and future split
  plan.
- `src/grounding/README.md`: local grounding directory guide and file
  responsibilities.
- `docs/module_guides/projector.md`: projector / 2D-to-3D metric-layer
  boundary, current shadow-contract fields, import policy, forbidden
  dependencies, and future packaging plan.
- `docs/module_guides/planning.md`: planning boundary policy for bounded
  relative motion, offline autoregressive planners, parser handoff,
  Cartesian gateway risks, and future `src/planning/` migration staging.
- `docs/h12_planning_import_plan.md`: H12 planning import compatibility plan
  for possible future `src/planning/` migration staging.
- `docs/module_guides/contracts.md`: contracts/readiness boundary policy,
  H13-A classification, no-motion invariants, future `src/contracts/`
  migration policy, and why `src/readiness/` is premature.
- `docs/module_guides/safety.md`: safety boundary policy, H14-A
  classification, fail-closed invariants, future narrow `src/safety/` policy,
  and why mixed execution gateways stay in place.
- `docs/module_guides/execution.md`: execution/operator boundary policy,
  H15-A classification, real/sim/shared execution sensitivity, future narrow
  `src/execution/` policy, and why mixed execution surfaces stay in place.
- `docs/h15_execution_import_plan.md`: H15 execution import compatibility plan
  for possible future narrow `src/execution/` migration staging.
- `docs/module_guides/simulation.md`: simulation / Isaac boundary policy,
  H16-A classification, SIM_ONLY versus REAL_PATH isolation, guarded Isaac
  runtime import policy, artifact/replay sensitivity, and why `src/simulation/`
  and `src/isaac/` are not created yet.
- `docs/h16_simulation_import_plan.md`: H16 simulation / Isaac import
  compatibility plan for possible future conservative `src/isaac/` or
  `src/simulation/` migration staging.
- `docs/module_guides/vision.md`: vision scene/camera snapshot boundary,
  current snapshot/builder file responsibilities, import migration postponement,
  and future `src/vision/snapshot/` package target.

## Future Guides

Other module guides should be added as those modules are audited or migrated.
Likely future guides include:

- calibration
- replay
- entrypoints

Keep this file as a lightweight index. Put detailed module construction rules
in `docs/module_guides/<module>.md` and local directory notes in
`src/<module>/README.md`.
