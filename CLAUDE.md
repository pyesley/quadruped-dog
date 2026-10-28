# quadruped-dog — notes for Claude Code

This repo holds the CAD for the quadruped (`quadruped_12dof_v1/`) and the leg test stand (`testStand/`).
The test-stand software lives in `testStand/software/`, which has its own `README.md` (setup, how to run),
`NOTES.md` (lab notebook) and `CLAUDE.md` (hardware facts and the rest of the rules). Read those before
touching anything there.

## Hard rule: Claude never changes the moteus-c1 configuration

The moteus-c1 motor controller boards are configured **only by a human**. Claude must not run or write
anything that changes their configuration (`conf set`, `conf write`, `moteus_tool --write-config`,
`--zero-offset`, `--calibrate`, persistent register writes), whoever is running the session. A wrong limit,
offset or gain can drive the robot out of its mechanical bounds or over-current the motors. Reading
(`--dump-config`, queries) is fine: propose the exact command and let the human type it.
