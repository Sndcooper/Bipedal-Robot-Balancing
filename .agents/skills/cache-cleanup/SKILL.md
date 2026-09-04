---
name: cache-cleanup
description: "Use when the user asks to clean, remove, delete, or inspect cache, build, temporary, or generated files in an attached or specified folder. Always inventory candidates and ask for explicit confirmation before deletion."
---

# Cache Cleanup

Use this workflow for any folder the user attaches or names. Treat the requested folder as the cleanup root and never search outside it.

## Required workflow

1. Inventory candidates recursively before changing anything.
2. Report candidate categories, counts, and total size. Include representative paths when the result is large.
3. Ask the user for explicit confirmation before deleting. Do not infer approval from the original request if the inventory was not shown.
4. After confirmation, delete only the approved categories.
5. Verify that the approved candidates are gone and report the number of removed files and directories.

## Default disposable candidates

Directories:

- `__pycache__`
- `.pio`
- `build`
- `dist`
- `node_modules` only when the user explicitly includes dependency directories
- `cache`
- `.cache`
- `tmp` and `temp` only when clearly generated

Files:

- Python bytecode: `.pyc`, `.pyo`
- Compiled objects: `.o`, `.obj`
- Firmware/build outputs: `.elf`, `.hex`, `.bin`, `.map`
- Temporary files: `.tmp`, `.temp`
- Logs: `.log` only when the user approves log deletion

## Safety rules

- Never delete source code, project configuration, documentation, test data, datasets, firmware source, GUI code, notebooks, session records, or user-created logs by default.
- Do not delete a directory merely because its name contains `build`, `cache`, or `tmp`; verify it is generated output and show its path.
- `.pio` is PlatformIO-generated build output. It can be deleted safely and will be recreated by PlatformIO, but tell the user that the next build may take longer.
- `node_modules` can be regenerated but may be intentionally retained; require separate explicit approval.
- Preserve files with ambiguous extensions or names until the user identifies them.
- Use the platform's structured filesystem commands. Constrain every deletion operation to the approved root and exact candidate names/extensions.
- Do not use destructive repository-wide commands such as `git clean` unless the user explicitly requests them and confirms the exact flags.

## Confirmation wording

Before deletion, state the root folder, the categories selected, and the counts. Ask a direct question such as: "Delete these generated artifacts from `<root>`? Source, configuration, documentation, and session files will remain untouched."

If the user approves only some categories, delete only those categories and re-inventory the rest.
