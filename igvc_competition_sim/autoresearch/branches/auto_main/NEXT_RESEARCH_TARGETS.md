# Next Research Targets: auto_main

This file is branch-local. Keep global rules and permanent history in the
top-level autoresearch docs; keep this file focused on what this branch should
test next.

## Baseline First

- Run the Blender-authored loop course only:
  `blender_competition_course`.
- Compare against Hailmary as prior evidence, not as a pass/fail substitute.
- Do not run the older generated courses during the current nightly; they are
  not the authoritative course for this branch.

## Branch-Specific Targets

- Planning/control: Planning/control changed; revalidate Hailmary planning dead ends before treating them as blocked.
- Perception: Perception changed; run the Jetson/camera lane before using detector output as a planning signal.
- Blender loop course: preserve the course geometry; failures on the validated
  oracle course are robot-stack findings unless a concrete sim/scorer defect is
  proven.

## Logging Rules

- Use `log_experiment.py check --branch-scope auto_main` and
  `log_experiment.py add --branch-scope auto_main` for checks
  and experiment entries.
- Branch-local terminal duplicates block repeat work.
- Global/Hailmary duplicates warn only; they do not block revalidation when
  this branch changed the relevant subsystem.
