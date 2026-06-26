# Contract Schema Boundary

This directory is reserved for future schema notes for shared TETO contracts,
evidence manifests, readiness reports, replay records, and no-motion safety
flags.

H8 does not add runtime dataclasses, validators, imports, or call sites here.
Existing contract implementations remain in their current modules until a later
audited migration.

H13-B keeps this directory documentation-only. Schema migration remains future
work and requires a compatibility/import plan, focused tests, clean import
scans, and preserved no-motion semantics.
