# Agent Guide

## Commands

- Install: `python3 -m pip install -e .`
- Test: `pytest -q`
- UI: `streamlit run whl_dyn/ui/app.py`
- CLI: `python3 -m whl_dyn.cli <command>`

## Principles

- Read the relevant code and tests before editing.
- Preserve planning, collection, processing, trajectory, and UI boundaries.
- Add or update focused tests for behavior changes.
- Keep vehicle-specific message mappings outside generic Python modules.
- Do not modify unrelated files or discard existing worktree changes.

## Knowledge

Read `.agents/knowledge/` only when its topic is relevant.

## Skills

Read the relevant `.agents/skills/*/SKILL.md` before testing, reviewing, or
preparing a release.
