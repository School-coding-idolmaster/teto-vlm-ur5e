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
- `docs/module_guides/vision.md`: vision scene/camera snapshot boundary,
  current snapshot/builder file responsibilities, import migration postponement,
  and future `src/vision/snapshot/` package target.

## Future Guides

Other module guides should be added as those modules are audited or migrated.
Likely future guides include:

- calibration
- contracts
- execution
- replay
- safety
- entrypoints

Keep this file as a lightweight index. Put detailed module construction rules
in `docs/module_guides/<module>.md` and local directory notes in
`src/<module>/README.md`.
