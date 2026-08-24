# Review

## When

Read before reviewing a change set.

## Rules

- Inspect the diff and affected tests before judging behavior.
- Check module boundaries and vehicle-agnostic signal handling.
- Require explicit safety behavior for active command publication.
- Verify plans, collectors, processors, and their documentation remain aligned.
- Use `git diff --check` for whitespace errors.

## Sources

- `whl_dyn/planning/`
- `whl_dyn/collection/`
- `whl_dyn/processing/`
- `tests/`
