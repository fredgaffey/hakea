// Gate test from the usage-tracing kit: every dated row in the agent log carries
// a token field (and a green flag, if the project requires one), and the usage
// ledger is valid JSONL with no duplicate agents. Paths come from .claude/usage.json.
import { test } from "node:test";
import assert from "node:assert/strict";
import { readFileSync, existsSync } from "node:fs";
import { join, dirname } from "node:path";
import { fileURLToPath } from "node:url";
import { checkLog, checkLedger, loadConfig, findRoot } from "../tools/usage/trace.mjs";

const root = findRoot(dirname(fileURLToPath(import.meta.url)));
const cfg = loadConfig(root);

test("agent log rows carry 'Nk / M calls' (and 'green: yes|no|n/a' if required)", () => {
  const f = join(root, cfg.log);
  if (!existsSync(f)) return;
  assert.deepEqual(checkLog(readFileSync(f, "utf8"), cfg).problems, []);
});

test("usage ledger is valid JSONL with the fields stats depends on", () => {
  const f = join(root, cfg.ledger);
  if (!existsSync(f)) return;
  assert.deepEqual(checkLedger(readFileSync(f, "utf8")).problems, []);
});
