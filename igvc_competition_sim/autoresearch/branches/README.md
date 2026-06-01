# Branch-Scoped Autoresearch Memory

Each robot-stack branch that runs dual-sim autoresearch should get a branch
profile here. Branch profiles prevent Hailmary-specific results from becoming
false assumptions on branches with different code.

Create or refresh a profile from the autoresearch directory:

```bash
python3 master/orchestrator.py init-branch-profile \
  --robot-branch <branch-under-test> \
  --base-branch hailmary_deploy
```

Use the generated branch scope when checking or logging experiments:

```bash
python3 log_experiment.py check \
  --branch-scope <branch-scope> \
  --hypothesis "<hypothesis>"
```

Rules:

- Global IGVC rules, course integrity, scorer fixes, and safety gates apply to
  every branch.
- Robot-stack tuning findings are branch-scoped unless the profile marks the
  relevant subsystem unchanged.
- Branch-local terminal duplicates block repeat work.
- Global duplicates warn only; they do not block revalidation when the branch
  changed the relevant subsystem.
- Older Hailmary fast-suite findings are not official-full-loop validation.
