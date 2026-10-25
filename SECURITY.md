# Security policy

fiss_planner is research software for motion planning. It is not certified for use in production
vehicles; any deployment on a real vehicle needs its own safety case, an independent safety layer and
a trained safety driver.

## Supported versions

Fixes are made on `main` and released in the latest 1.x version.

## Reporting a vulnerability

Please do **not** open a public issue for security problems, for example a crafted map, message or
configuration that crashes a node, corrupts memory or makes the planner output an unsafe trajectory
without reporting a failure.

Report them through GitHub's private vulnerability reporting instead:
**Security → Report a vulnerability** on the repository page. Include the version, a minimal input
that reproduces the problem and its effect. You can expect an acknowledgement within a week.
