# Next Research Targets: hailmary_deploy

This file is branch-local. Keep global rules and permanent history in the
top-level autoresearch docs; keep this file focused on what this branch should
test next.

## Baseline First

- Run the fast oracle planning/control suite and record the result in this
  branch profile.
- Compare against Hailmary as prior evidence, not as a pass/fail substitute.
- Run `official_full_loop` only after the branch is stable enough for a final
  long-course gate.

## Branch-Specific Targets

- Planning/control: Planning/control files did not change; Hailmary planning findings are strong prior evidence.
- Perception: Perception files did not change; camera-fidelity work remains global future work.
- Official full loop: preserve the course geometry; failures on a validated
  oracle course are robot-stack findings unless a concrete sim/scorer defect is
  proven.

## Logging Rules

- Use `log_experiment.py check --branch-scope hailmary_deploy` and
  `log_experiment.py add --branch-scope hailmary_deploy` for checks
  and experiment entries.
- Branch-local terminal duplicates block repeat work.
- Global/Hailmary duplicates warn only; they do not block revalidation when
  this branch changed the relevant subsystem.
