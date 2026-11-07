# Rules for coding agents in locatebot

Read `docs/README.md` and `docs/08-LAUNCH-GUIDE.md` first.

- Work only on your assigned branch and only in the files your lane owns (`docs/03-BRANCH-AND-MERGE-PLAN.md`).
- `rc_robot/contracts.py`, root config, `tests/test_integration.py` and merges belong to the integrator.
- Python >=3.10, standard library only, `python3 -m unittest discover -s tests -v`.
- Test first: show the failing test, implement minimally, run the full suite, commit scoped files.
- Vision inference never blocks the control loop. The safety supervisor is the final gate on motion.
- No pushing, merging, package installs, model downloads, credentials, hardware access or nested agents.
- Report the commit SHA, real test output and known limitations. Never claim physical safety from software tests.
