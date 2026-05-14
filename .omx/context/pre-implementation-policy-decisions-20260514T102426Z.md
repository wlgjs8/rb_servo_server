# Pre-Implementation Policy Decisions Context

Generated: 2026-05-14T10:24:26Z

## Task

Run a 4-worker architect review with two Codex workers and two Claude workers:

```bash
OMX_TEAM_WORKER_CLI_MAP=codex,claude,codex,claude omx team 4:architect "<task>"
```

## Goal

Use `.omx/debates/critic-plan-review.md` to decide implementation-blocking policies before any code changes.

## Constraints

- Do not implement.
- Worker panes should use read-only investigation.
- The leader may write the final policy artifact:
  - `.omx/debates/pre-implementation-policy-decisions.md`
- Possible architecture decisions should be made by the architect team.
- Human/product/operator decisions should be marked `Escalation Required: Yes`.

## Required Policy Format

For each policy:

- Decision
- Rationale
- Rejected alternatives
- Required tests
- Implementation touchpoints
- Go-blocking 여부
- Escalation Required: Yes/No

## Required Policies

1. EmergencyStop vs ResetFault simultaneous input
2. malformed JSON / malformed numeric payload
3. `timeout_sec <= 0`
4. `stop_both_arms_on_single_arm_error`
5. `sendServoJ` failure handling
6. realtime setup failure handling
7. UDP bind / network exposure default
8. StatePublisher state ownership
9. parser / command buffer / logger replacement and rollback policy

## Conclusion Contract

The final artifact must conclude with exactly one of:

- Go
- No-Go
- Needs Human Decision
