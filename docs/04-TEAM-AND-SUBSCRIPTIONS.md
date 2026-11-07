# Team and subscription allocation

## Reported resources
Claude Pro ×3; Claude Max $200 tier ×1; ChatGPT Plus ×1. The user permits suitable models of our choice. Account access, remaining allowances and current tool/model entitlements are not verified.

| Resource | Suggested primary responsibility |
|---|---|
| Claude Max owner | Architecture, integration, hard debugging and safety/concurrency review |
| Claude Pro owner 1 | Safety lane |
| Claude Pro owner 2 | Control lane |
| Claude Pro owner 3 | Perception lane |
| ChatGPT Plus owner | Mission lane and independent integration-test/review perspective |

Each teammate uses their own authorized account and isolated checkout. One subscription is not automatically one continuously running agent. No shared credential copying, account rotation to evade limits or assumption that chat subscriptions include API credit. Do not promise supported concurrent-agent counts or specific models without checking current tool/plan access.

## Model strategy when implementation is approved
Use strong reasoning for architecture, safety and concurrency review; capable coding models for bounded lane tasks. Verify actual model selectors before choosing names. Clear contracts and deterministic tests matter more than agent count. Do not alter native Hermes aliases/routing merely to use these accounts. Paid API fallback requires separate approval.

## Observed execution boundary
A check in this Hermes execution environment did not find claude or codex on PATH. This does not establish their state on the user's remote Debian desktop or personal devices. No subscription has been signed into or consumed for this documentation task. No coding subagents have been launched. Markdown handoff prompts are not running agents.

## Later authentication and cost checks
Use each supported coding tool's normal sign-in. Never store credentials, cookies or tokens in project files or chat. Confirm available limits before launch; give each run a bounded task and stop condition. Do not launch recursive agent trees or redundant reviewers. Preserve task state and test evidence in the lane docs so an interrupted run can resume cheaply.

## Review independence
Authors do not approve their own slices. Assign fresh review context or another teammate, especially safety/control and asynchronous shutdown. A second model saying “looks good” is not execution evidence; real tests and physical gates remain mandatory.
