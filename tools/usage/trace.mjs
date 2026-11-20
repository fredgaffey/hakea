#!/usr/bin/env node
// Usage tracer for Claude Code projects: any project, macOS, Linux or Windows.
//
// Reads the transcripts Claude Code already writes under
//   <claude dir>/projects/<slug>/<session>.jsonl                  (main sessions)
//   <claude dir>/projects/<slug>/<session>/subagents/agent-*.jsonl (Agent-tool subagents)
// where <claude dir> is $CLAUDE_CONFIG_DIR or ~/.claude, and reports tokens, tool
// calls, model and duration per agent and per session. Nothing is hand-copied.
//
//   node tools/usage/trace.mjs agents   [--since YYYY-MM-DD] [--format md|tsv|json|jsonl]
//   node tools/usage/trace.mjs agents   --append [FILE] [--quiet]      # add finished agents to the ledger
//   node tools/usage/trace.mjs sessions [--since D] [--format ...]    # main-session spend
//   node tools/usage/trace.mjs stats    [--since D] [--metric total|fresh|ctxEnd]
//   node tools/usage/trace.mjs row      --agent ID [--outcome "..."] [--green yes|no|n/a]
//   node tools/usage/trace.mjs check    [LOGFILE]                     # validate the agent log and ledger
//   node tools/usage/trace.mjs brief                                  # budget paragraph for an agent brief
//   node tools/usage/trace.mjs config                                 # settings and transcript folders in use
//   node tools/usage/trace.mjs init     --project PATH [--dry-run]    # install into a project
//
// Options: --project PATH, --config FILE, --claude-dir PATH. Per-project settings
// live in <project>/.claude/usage.json (see DEFAULTS below for every key).
//
// Three token figures per agent:
//   total  = sum over turns of input + cache_creation + cache_read + output: everything
//            the API processed. Cache reads dominate; rate limits and cost track this.
//   fresh  = sum of input + cache_creation: new material entering the context.
//   ctxEnd = the largest single-turn context; matches the `subagent_tokens` figure
//            the Agent tool reports to within ~2%.

import {
  readFileSync, readdirSync, existsSync, statSync, appendFileSync, writeFileSync,
  mkdirSync, openSync, readSync, closeSync, realpathSync,
} from "node:fs";
import path from "node:path";
import { homedir } from "node:os";
import { execFileSync } from "node:child_process";
import { fileURLToPath } from "node:url";

const { join, basename, dirname, resolve, relative } = path;
const SELF = fileURLToPath(import.meta.url);
const KIT_ROOT = resolve(dirname(SELF), "..", "..");
const CONFIG_FILE = join(".claude", "usage.json");
const TOOL_REL = "tools/usage/trace.mjs";
export const HOOK_CMD = `node "$CLAUDE_PROJECT_DIR/${TOOL_REL}" agents --append --quiet`;

// ---------------------------------------------------------------- config
export const DEFAULTS = {
  log: "docs/AGENT-LOG.md",          // the hand-annotated agent log (one row per agent)
  ledger: "docs/usage/agents.jsonl",  // machine ledger the SubagentStop hook appends to
  testsDir: "tests",                  // where init puts the gate test
  metric: "total",                    // what `stats` ranks by: total | fresh | ctxEnd
  outlierMultiple: 4,                 // stats flags agents above this multiple of the median
  agentTypes: null,                   // e.g. ["lean"]: report only these subagent types
  extraPaths: [],                     // other checkouts whose transcripts count as this project's
  includeWorktrees: true,             // count sessions run in this repo's git worktrees
  budget: { calls: 40, tokens: 100000, metric: "ctxEnd", stopAt: 0.8 },
  logRow: { metric: "ctxEnd", requireGreen: true },
  prices: null,                       // { "<model substring>": { input, cacheWrite, cacheRead, output } } in USD per million tokens
};

export function loadConfig(root, file) {
  const f = file ? resolve(root, file) : join(root, CONFIG_FILE);
  const user = existsSync(f) ? JSON.parse(readFileSync(f, "utf8")) : {};
  const cfg = { ...DEFAULTS, ...user };
  for (const key of ["budget", "logRow"]) cfg[key] = { ...DEFAULTS[key], ...(user[key] ?? {}) };
  if (!["total", "fresh", "ctxEnd"].includes(cfg.metric)) throw new Error(`metric must be total|fresh|ctxEnd, got ${cfg.metric}`);
  return { ...cfg, root, file: existsSync(f) ? f : null };
}

function git(cwd, ...args) {
  try { return execFileSync("git", args, { cwd, encoding: "utf8", stdio: ["ignore", "pipe", "ignore"] }).trim() || null; }
  catch { return null; }
}

// Project root: the nearest folder (up to the git top level) holding .claude/usage.json,
// else the git top level, else `start` itself.
export function findRoot(start = process.cwd()) {
  const top = git(start, "rev-parse", "--show-toplevel");
  const stop = top ? resolve(top) : null;
  for (let d = resolve(start); ; d = dirname(d)) {
    if (existsSync(join(d, CONFIG_FILE))) return d;
    if (d === stop || dirname(d) === d) break;
  }
  return stop ?? resolve(start);
}

export function claudeDir(o = {}) { return o.claudeDir ?? process.env.CLAUDE_CONFIG_DIR ?? join(homedir(), ".claude"); }

// ---------------------------------------------------------------- paths (platform-aware)
export function projectSlug(p) { return p.replace(/[^a-zA-Z0-9]/g, "-"); }

export function normPath(p, win = process.platform === "win32") {
  const s = (win ? path.win32 : path.posix).resolve(p).replace(/\\/g, "/").replace(/\/+$/, "");
  return win ? s.toLowerCase() : s;
}

export function isWithin(child, parent, win) {
  const c = normPath(child, win), p = normPath(parent, win);
  return c === p || c.startsWith(p + "/");
}

function projectRoots(cfg) {
  const roots = [cfg.root];
  if (cfg.includeWorktrees) {
    for (const l of (git(cfg.root, "worktree", "list", "--porcelain") ?? "").split(/\r?\n/)) {
      if (l.startsWith("worktree ")) roots.push(resolve(l.slice(9)));
    }
  }
  for (const p of cfg.extraPaths ?? []) roots.push(resolve(cfg.root, p));
  const all = roots.flatMap((r) => { try { return [r, realpathSync(r)]; } catch { return [r]; } });
  return [...new Set(all)];
}

// The working directory a transcript was started in, from its first `cwd` field.
function firstCwd(file) {
  const fd = openSync(file, "r");
  try {
    const buf = Buffer.alloc(256 * 1024);
    const n = readSync(fd, buf, 0, buf.length, 0);
    const m = /"cwd":"((?:[^"\\]|\\.)*)"/.exec(buf.toString("utf8", 0, n));
    return m ? JSON.parse(`"${m[1]}"`) : null;
  } finally { closeSync(fd); }
}

// Transcript folders belonging to this project. Matched by slug first, then by the
// cwd recorded inside the transcripts, so Windows drive paths, symlinks, subfolders
// and worktrees all resolve without guessing how Claude Code built the slug.
export function transcriptDirs(cfg, o = {}) {
  const base = join(claudeDir(o), "projects");
  if (!existsSync(base)) throw new Error(`no Claude Code transcripts at ${base} (set --claude-dir or CLAUDE_CONFIG_DIR)`);
  const roots = projectRoots(cfg);
  const slugs = new Set(roots.map(projectSlug));
  const out = [];
  for (const d of readdirSync(base)) {
    const dir = join(base, d);
    try { if (!statSync(dir).isDirectory()) continue; } catch { continue; }
    if (slugs.has(d)) { out.push(dir); continue; }
    for (const f of listSessions(dir).slice(0, 3)) {
      const cwd = firstCwd(f);
      if (!cwd) continue;
      if (roots.some((r) => isWithin(cwd, r))) out.push(dir);
      break;
    }
  }
  if (!out.length) throw new Error(`no transcripts for ${cfg.root} under ${base}`);
  return out;
}

function readJsonl(file) {
  const out = [];
  for (const line of readFileSync(file, "utf8").split(/\r?\n/)) {
    if (!line) continue;
    try { out.push(JSON.parse(line)); } catch { /* partial line */ }
  }
  return out;
}

function listSessions(dir) {
  return readdirSync(dir).filter((f) => f.endsWith(".jsonl")).map((f) => join(dir, f));
}
function listAgents(dir) {
  const out = [];
  for (const e of readdirSync(dir)) {
    const sub = join(dir, e, "subagents");
    if (!existsSync(sub) || !statSync(sub).isDirectory()) continue;
    for (const f of readdirSync(sub)) if (/^agent-.*\.jsonl$/.test(f)) out.push(join(sub, f));
  }
  return out;
}

// ---------------------------------------------------------------- summarising a transcript
export function summarise(records) {
  const usage = { input: 0, cacheCreate: 0, cacheRead: 0, output: 0 };
  const seen = new Set();
  const models = new Map();
  const tools = new Map();
  let turns = 0, toolCalls = 0, first = null, last = null, ctxEnd = 0;
  for (const r of records) {
    if (r.timestamp) { if (!first || r.timestamp < first) first = r.timestamp; if (!last || r.timestamp > last) last = r.timestamp; }
    if (r.type !== "assistant" || !r.message) continue;
    const blocks = Array.isArray(r.message.content) ? r.message.content : [];
    for (const b of blocks) if (b.type === "tool_use") { toolCalls++; tools.set(b.name, (tools.get(b.name) ?? 0) + 1); }
    const id = r.message.id ?? r.uuid;
    if (seen.has(id)) continue; // streaming writes one line per block, same usage
    seen.add(id);
    turns++;
    const u = r.message.usage ?? {};
    const inp = u.input_tokens ?? 0, cc = u.cache_creation_input_tokens ?? 0, cr = u.cache_read_input_tokens ?? 0, out = u.output_tokens ?? 0;
    usage.input += inp; usage.cacheCreate += cc; usage.cacheRead += cr; usage.output += out;
    ctxEnd = Math.max(ctxEnd, inp + cc + cr + out);
    if (r.message.model) models.set(r.message.model, (models.get(r.message.model) ?? 0) + 1);
  }
  const total = usage.input + usage.cacheCreate + usage.cacheRead + usage.output;
  const model = [...models.entries()].sort((a, b) => b[1] - a[1])[0]?.[0] ?? null;
  const minutes = first && last ? Math.round((Date.parse(last) - Date.parse(first)) / 60000) : null;
  return { model, turns, toolCalls, tools: Object.fromEntries(tools), ...usage, fresh: usage.input + usage.cacheCreate, ctxEnd, total, start: first, end: last, minutes };
}

function firstUserText(records) {
  for (const r of records) {
    if (r.type !== "user" || !r.message) continue;
    const c = r.message.content;
    const t = typeof c === "string" ? c : (Array.isArray(c) ? c.find((b) => b.type === "text")?.text : null);
    if (t && !t.startsWith("<")) return t.replace(/\s+/g, " ").slice(0, 80);
  }
  return null;
}

// Parent session: map agentId -> what the Agent tool was asked for.
function agentLaunches(sessionFile) {
  const byToolUse = new Map();
  const out = new Map();
  for (const r of readJsonl(sessionFile)) {
    const blocks = Array.isArray(r.message?.content) ? r.message.content : [];
    for (const b of blocks) {
      if (b.type === "tool_use" && b.name === "Agent") byToolUse.set(b.id, b.input ?? {});
      if (b.type === "tool_result") {
        const text = typeof b.content === "string" ? b.content : JSON.stringify(b.content ?? "");
        const m = /agentId:\s*([a-f0-9]{8,})/.exec(text);
        if (m && byToolUse.has(b.tool_use_id)) {
          const inp = byToolUse.get(b.tool_use_id);
          out.set(m[1], { description: inp.description ?? null, subagentType: inp.subagent_type ?? null, requestedModel: inp.model ?? null, isolation: inp.isolation ?? null });
        }
      }
    }
  }
  return out;
}

// ---------------------------------------------------------------- collectors
function ledgerPath(cfg, file) { return resolve(cfg.root, typeof file === "string" ? file : cfg.ledger); }
function readLedger(cfg) { const f = ledgerPath(cfg); return existsSync(f) ? readJsonl(f) : []; }

// Agents from the transcripts, plus ledger rows for agents whose transcripts
// Claude Code has since pruned (cleanupPeriodDays).
function collectAgents(cfg, o, { history = true, allTypes = false } = {}) {
  const dirs = transcriptDirs(cfg, o);
  const launches = new Map();
  for (const dir of dirs) for (const s of listSessions(dir)) for (const [k, v] of agentLaunches(s)) launches.set(k, v);
  const byId = new Map();
  if (history) for (const r of readLedger(cfg)) if (r.agentId) byId.set(r.agentId, r);
  for (const dir of dirs) {
    for (const f of listAgents(dir)) {
      const recs = readJsonl(f);
      if (!recs.length) continue;
      const agentId = recs[0].agentId ?? basename(f).replace(/^agent-|\.jsonl$/g, "");
      const s = summarise(recs);
      if (!s.turns) continue;
      const l = launches.get(agentId) ?? {};
      byId.set(agentId, { agentId, sessionId: recs[0].sessionId ?? null, description: l.description ?? firstUserText(recs), subagentType: l.subagentType ?? null, requestedModel: l.requestedModel ?? null, isolation: l.isolation ?? null, cwd: recs[0].cwd ?? null, ...s });
    }
  }
  let rows = [...byId.values()];
  if (cfg.agentTypes && !allTypes) rows = rows.filter((r) => cfg.agentTypes.includes(r.subagentType ?? "general-purpose"));
  return filterSince(rows, o.since).sort((a, b) => (a.start ?? "").localeCompare(b.start ?? ""));
}

function collectSessions(cfg, o) {
  const rows = [];
  for (const dir of transcriptDirs(cfg, o)) {
    for (const f of listSessions(dir)) {
      const recs = readJsonl(f);
      const s = summarise(recs);
      if (!s.turns) continue;
      const sub = join(dir, basename(f, ".jsonl"), "subagents");
      const agents = existsSync(sub) ? readdirSync(sub).filter((x) => x.endsWith(".jsonl")).length : 0;
      rows.push({ sessionId: basename(f, ".jsonl"), description: firstUserText(recs), agents, gitBranch: recs.find((r) => r.gitBranch)?.gitBranch ?? null, cwd: recs.find((r) => r.cwd)?.cwd ?? null, ...s });
    }
  }
  return filterSince(rows, o.since).sort((a, b) => (a.start ?? "").localeCompare(b.start ?? ""));
}

function filterSince(rows, since) {
  if (!since) return rows;
  if (!/^\d{4}-\d{2}-\d{2}$/.test(since)) throw new Error(`--since wants YYYY-MM-DD, got ${since}`);
  return rows.filter((r) => r.start && r.start.slice(0, 10) >= since);
}

// ---------------------------------------------------------------- cost
export function costOf(r, prices) {
  if (!prices || !r.model) return null;
  const key = Object.keys(prices).find((k) => r.model.includes(k));
  if (!key) return null;
  const p = prices[key];
  return ((r.input ?? 0) * (p.input ?? 0) + (r.cacheCreate ?? 0) * (p.cacheWrite ?? 0) + (r.cacheRead ?? 0) * (p.cacheRead ?? 0) + (r.output ?? 0) * (p.output ?? 0)) / 1e6;
}
const withCost = (rows, cfg) => (cfg.prices ? rows.map((r) => ({ ...r, cost: costOf(r, cfg.prices) })) : rows);

// ---------------------------------------------------------------- output
export const human = (n) => (n == null ? "-" : n >= 1e6 ? `${(n / 1e6).toFixed(1)}M` : `${Math.round(n / 1000)}k`);
const usd = (n) => (n == null ? "-" : `$${n.toFixed(2)}`);
const shortModel = (m) => (m ?? "-").replace(/^claude-/, "");
const cell = (v) => String(v).replace(/\|/g, "\\|");

function printRows(rows, kind, cfg, o) {
  const format = o.format ?? "md";
  if (format === "json") return console.log(JSON.stringify(rows, null, 2));
  if (format === "jsonl") return rows.forEach((r) => console.log(JSON.stringify(r)));
  const cols = kind === "agents"
    ? ["start", "agentId", "model", "type", "description", "total", "fresh", "ctx", "output", "turns", "calls", "min"]
    : ["start", "sessionId", "model", "description", "agents", "total", "fresh", "output", "turns", "calls", "min"];
  if (cfg.prices) cols.push("cost");
  const val = (r, c) => ({
    start: (r.start ?? "").slice(0, 16).replace("T", " "), agentId: r.agentId, sessionId: r.sessionId?.slice(0, 8),
    model: shortModel(r.model), type: r.subagentType ?? "-", description: (r.description ?? "-").slice(0, 60),
    agents: r.agents, total: human(r.total), fresh: human(r.fresh), ctx: human(r.ctxEnd), output: human(r.output),
    turns: r.turns, calls: r.toolCalls, min: r.minutes ?? "-", cost: usd(r.cost),
  })[c];
  if (format === "tsv") { console.log(cols.join("\t")); rows.forEach((r) => console.log(cols.map((c) => val(r, c)).join("\t"))); return; }
  if (format !== "md") throw new Error(`--format must be md|tsv|json|jsonl, got ${format}`);
  console.log(`| ${cols.join(" | ")} |`);
  console.log(`|${cols.map(() => "---").join("|")}|`);
  rows.forEach((r) => console.log(`| ${cols.map((c) => cell(val(r, c))).join(" | ")} |`));
}

function median(xs) { if (!xs.length) return null; const s = [...xs].sort((a, b) => a - b); const m = s.length >> 1; return s.length % 2 ? s[m] : (s[m - 1] + s[m]) / 2; }
const sum = (xs) => xs.reduce((a, b) => a + (b ?? 0), 0);

function stats(rows, cfg, o) {
  const metric = o.metric ?? cfg.metric;
  if (!["total", "fresh", "ctxEnd"].includes(metric)) throw new Error("--metric must be total|fresh|ctxEnd");
  if (!rows.length) return console.log("no agents found");
  const values = rows.map((r) => r[metric]);
  const med = median(values);
  console.log(`# Usage stats (${rows.length} agents${o.since ? `, since ${o.since}` : ""})\n`);
  console.log(`| metric | value |\n|---|---|`);
  console.log(`| agents | ${rows.length} |`);
  for (const m of ["total", "fresh", "ctxEnd"]) console.log(`| ${m}/agent median | ${human(median(rows.map((r) => r[m])))} |`);
  console.log(`| ${metric}/agent p90 | ${human([...values].sort((a, b) => a - b)[Math.floor(values.length * 0.9)])} |`);
  console.log(`| tool calls/agent median | ${median(rows.map((r) => r.toolCalls))} |`);
  console.log(`| ${metric} tokens, all agents | ${human(sum(values))} |`);
  if (cfg.prices) console.log(`| cost, all agents | ${usd(sum(rows.map((r) => r.cost)))} |`);

  const byModel = new Map();
  for (const r of rows) {
    const e = byModel.get(shortModel(r.model)) ?? { n: 0, v: 0, calls: 0, cost: 0 };
    e.n++; e.v += r[metric]; e.calls += r.toolCalls; e.cost += r.cost ?? 0;
    byModel.set(shortModel(r.model), e);
  }
  const grand = sum(values) || 1;
  console.log(`\n| model | agents | mean ${metric} | mean calls | share |${cfg.prices ? " cost |" : ""}\n|---|---|---|---|---|${cfg.prices ? "---|" : ""}`);
  for (const [m, e] of [...byModel].sort((a, b) => b[1].v - a[1].v)) {
    console.log(`| ${m} | ${e.n} | ${human(e.v / e.n)} | ${Math.round(e.calls / e.n)} | ${Math.round((100 * e.v) / grand)}% |${cfg.prices ? ` ${usd(e.cost)} |` : ""}`);
  }

  const brief = (r, v) => `| ${(r.start ?? "").slice(0, 10)} | ${r.agentId} | ${shortModel(r.model)} | ${cell((r.description ?? "-").slice(0, 50))} | ${v} | ${r.toolCalls} | ${r.minutes ?? "-"} |`;
  const mult = cfg.outlierMultiple;
  const outliers = rows.filter((r) => med && r[metric] > mult * med);
  if (outliers.length) {
    console.log(`\n## Outliers (${metric} > ${mult}x median = ${human(mult * med)})\n\n| start | agentId | model | description | ${metric} | calls | min |\n|---|---|---|---|---|---|---|`);
    for (const r of outliers.sort((a, b) => b[metric] - a[metric])) console.log(brief(r, human(r[metric])));
  }
  const b = cfg.budget;
  const over = rows.filter((r) => r.toolCalls > b.calls || r[b.metric] > b.tokens);
  console.log(`\n## Over budget (> ${b.calls} calls or ${b.metric} > ${human(b.tokens)}): ${over.length} of ${rows.length} (${Math.round((100 * over.length) / rows.length)}%)`);
  if (over.length) {
    console.log(`\n| start | agentId | model | description | ${b.metric} | calls | min |\n|---|---|---|---|---|---|---|`);
    for (const r of over.sort((a, b2) => b2[b.metric] - a[b.metric]).slice(0, 15)) console.log(brief(r, human(r[b.metric])));
    if (over.length > 15) console.log(`\n(${over.length - 15} more; use \`agents --format tsv\` to see all)`);
  }
}

function appendLedger(cfg, o) {
  const file = ledgerPath(cfg, o.append);
  const rows = collectAgents(cfg, o, { history: false, allTypes: true });
  const have = new Set(existsSync(file) ? readJsonl(file).map((r) => r.agentId) : []);
  const add = rows.filter((r) => !have.has(r.agentId) && r.end && Date.now() - Date.parse(r.end) > 60_000); // settled ≥1 min
  if (add.length) { mkdirSync(dirname(file), { recursive: true }); appendFileSync(file, add.map((r) => JSON.stringify(r)).join("\n") + "\n"); }
  if (!o.quiet) console.log(`${add.length} new agent(s) appended to ${relative(cfg.root, file) || file} (${have.size + add.length} total)`);
}

// One agent-log row, in the format `check` enforces.
function row(rows, cfg, o) {
  if (!o.agent) throw new Error("row needs --agent ID (a prefix is enough)");
  const r = rows.find((x) => x.agentId === o.agent) ?? rows.find((x) => x.agentId.startsWith(o.agent));
  if (!r) throw new Error(`no agent matching ${o.agent}`);
  const cells = [(r.end ?? r.start).slice(0, 10), shortModel(r.model), r.subagentType ?? "-", cell(r.description ?? "-"), cell(o.outcome ?? "TODO outcome"), `${human(r[cfg.logRow.metric])} / ${r.toolCalls} calls`];
  if (cfg.logRow.requireGreen) cells.push(`green: ${o.green ?? "TODO"}`);
  console.log(`| ${cells.join(" | ")} |`);
}

// ---------------------------------------------------------------- validation (used by the gate test)
// Every dated row must carry "N[k|M] / M calls" and, unless logRow.requireGreen is false, "green: yes|no|n/a".
export function checkLog(text, cfg = DEFAULTS) {
  const requireGreen = (cfg.logRow ?? DEFAULTS.logRow).requireGreen;
  const problems = [];
  const rows = text.split(/\r?\n/).filter((l) => /^\|\s*\d{4}-\d{2}-\d{2}/.test(l));
  for (const l of rows) {
    if (!/\d+(\.\d+)?[kM]?\s*\/\s*\d+\s*calls/.test(l)) problems.push(`missing "Nk / M calls": ${l.slice(0, 90)}`);
    if (requireGreen && !/green:\s*(yes|no|n\/a)/.test(l)) problems.push(`missing "green: yes|no|n/a": ${l.slice(0, 90)}`);
  }
  return { rows: rows.length, problems };
}

export function checkLedger(text) {
  const problems = [];
  const ids = new Set();
  let n = 0;
  for (const [i, line] of text.split(/\r?\n/).entries()) {
    if (!line) continue;
    n++;
    let r;
    try { r = JSON.parse(line); } catch { problems.push(`line ${i + 1}: not JSON`); continue; }
    for (const f of ["agentId", "model", "total", "fresh", "ctxEnd", "toolCalls", "start"]) if (!(f in r)) problems.push(`line ${i + 1}: ${f} missing`);
    if (ids.has(r.agentId)) problems.push(`line ${i + 1}: duplicate agentId ${r.agentId}`);
    ids.add(r.agentId);
  }
  return { rows: n, problems };
}

// ---------------------------------------------------------------- brief
function brief(cfg) {
  const b = cfg.budget;
  console.log(`> **Budget: ~${b.calls} tool calls / ~${human(b.tokens)} tokens of context.** Batch shell
> commands; read only the files named here; pipe test output through \`tail\`; run the
> full suite once at the end. Commit after each coherent step. At ~${Math.round(b.stopAt * 100)}% of
> budget, commit what is coherent and stop with a handoff report.`);
}

// ---------------------------------------------------------------- init
function template(name) {
  const f = join(KIT_ROOT, "templates", name);
  if (!existsSync(f)) throw new Error(`template ${name} not found in ${KIT_ROOT}/templates; run init from the skill's copy of trace.mjs`);
  return readFileSync(f, "utf8");
}

function init(cfg, o) {
  const root = cfg.root;
  if (normPath(root) === normPath(KIT_ROOT)) throw new Error("init installs the kit into a project; pass --project PATH");
  if (!existsSync(root)) throw new Error(`no such folder: ${root}`);
  const dry = !!o.dryRun;
  const report = [];
  const put = (rel, content, { overwrite = false } = {}) => {
    const dest = join(root, rel);
    const exists = existsSync(dest);
    if (exists && !overwrite) return report.push(`keep    ${rel} (exists)`);
    if (exists && readFileSync(dest, "utf8") === content) return report.push(`same    ${rel}`);
    if (!dry) { mkdirSync(dirname(dest), { recursive: true }); writeFileSync(dest, content); }
    report.push(`${exists ? "update" : "create"}  ${rel}`);
  };

  // An existing config is never touched; write it before running init to customise.
  put(CONFIG_FILE, template("usage.json"));

  put(TOOL_REL, readFileSync(SELF, "utf8"), { overwrite: true });

  const testRel = join(cfg.testsDir, "usage-ledger.test.mjs");
  let imp = relative(dirname(join(root, testRel)), join(root, TOOL_REL)).split(path.sep).join("/");
  if (!imp.startsWith(".")) imp = `./${imp}`;
  put(testRel, template("usage-ledger.test.mjs").replace("../tools/usage/trace.mjs", imp));

  put(cfg.log, template("AGENT-LOG.md")
    .replace("{{green_col}}", cfg.logRow.requireGreen ? " green first try |" : "")
    .replace("{{green_sep}}", cfg.logRow.requireGreen ? "---|" : ""));
  put(join(dirname(cfg.ledger), ".gitkeep"), "");

  const b = cfg.budget;
  put(join(".claude", "agents", "lean.md"), template("lean.md")
    .replaceAll("{{calls}}", String(b.calls)).replaceAll("{{tokens}}", human(b.tokens)).replaceAll("{{stop}}", String(Math.round(b.stopAt * 100))));

  // SubagentStop hook in the project's shared settings; merged, never overwritten.
  const sf = join(root, ".claude", "settings.json");
  let settings = {};
  let ok = true;
  if (existsSync(sf)) { try { settings = JSON.parse(readFileSync(sf, "utf8")); } catch { ok = false; } }
  if (!ok) report.push(`skip    .claude/settings.json (not valid JSON; add the SubagentStop hook by hand: ${HOOK_CMD})`);
  else if (JSON.stringify(settings.hooks?.SubagentStop ?? []).includes(TOOL_REL)) report.push("keep    .claude/settings.json (hook present)");
  else {
    ((settings.hooks ??= {}).SubagentStop ??= []).push({ hooks: [{ type: "command", command: HOOK_CMD }] });
    if (!dry) { mkdirSync(dirname(sf), { recursive: true }); writeFileSync(sf, JSON.stringify(settings, null, 2) + "\n"); }
    report.push(`${existsSync(sf) ? "update" : "create"}  .claude/settings.json (SubagentStop hook)`);
  }
  console.log(`${dry ? "Would install" : "Installed"} usage tracing into ${root}\n\n${report.map((l) => "  " + l).join("\n")}`);
}

// ---------------------------------------------------------------- main
function parseArgs(argv) {
  const cmd = argv[0] && !argv[0].startsWith("--") ? argv[0] : "agents";
  const rest = cmd === argv[0] ? argv.slice(1) : argv;
  const o = {}, positional = [];
  for (let i = 0; i < rest.length; i++) {
    const a = rest[i];
    if (!a.startsWith("--")) { positional.push(a); continue; }
    const eq = a.indexOf("=");
    const name = eq > 0 ? a.slice(2, eq) : a.slice(2);
    const key = name.replace(/-([a-z])/g, (_, c) => c.toUpperCase());
    if (eq > 0) o[key] = a.slice(eq + 1);
    else if (["quiet", "help", "dry-run"].includes(name)) o[key] = true;
    else if (name === "append" && (i + 1 >= rest.length || rest[i + 1].startsWith("--"))) o[key] = true;
    else if (i + 1 < rest.length) o[key] = rest[++i];
    else throw new Error(`--${name} needs a value`);
  }
  return { cmd, o, positional };
}

function main() {
  const { cmd, o, positional } = parseArgs(process.argv.slice(2));
  if (o.help || cmd === "help") {
    const lines = readFileSync(SELF, "utf8").split(/\r?\n/).slice(1);
    return console.log(lines.slice(0, lines.findIndex((l) => !l.startsWith("//"))).map((l) => l.replace(/^\/\/ ?/, "")).join("\n"));
  }
  const root = o.project ? resolve(o.project) : findRoot(process.env.CLAUDE_PROJECT_DIR ?? process.cwd());
  const cfg = loadConfig(root, o.config);
  switch (cmd) {
    case "agents":
      if (o.append) {
        try { appendLedger(cfg, o); } catch (e) { if (!o.quiet) console.error(`trace: ${e.message}`); } // never fail the hook
        return;
      }
      return printRows(withCost(collectAgents(cfg, o), cfg), "agents", cfg, o);
    case "sessions": return printRows(withCost(collectSessions(cfg, o), cfg), "sessions", cfg, o);
    case "stats": return stats(withCost(collectAgents(cfg, o), cfg), cfg, o);
    case "row": return row(collectAgents(cfg, o, { allTypes: true }), cfg, o);
    case "brief": return brief(cfg);
    case "init": return init(cfg, o);
    case "config": {
      let dirs;
      try { dirs = transcriptDirs(cfg, o); } catch (e) { dirs = [e.message]; }
      return console.log(JSON.stringify({ ...cfg, claudeDir: claudeDir(o), transcriptDirs: dirs }, null, 2));
    }
    case "check": {
      const logFile = resolve(root, positional[0] ?? cfg.log);
      let bad = 0;
      for (const [name, f, fn] of [["log", logFile, (t) => checkLog(t, cfg)], ["ledger", ledgerPath(cfg), checkLedger]]) {
        if (!existsSync(f)) { console.log(`${name}: ${relative(root, f)} not found`); continue; }
        const { rows, problems } = fn(readFileSync(f, "utf8"));
        console.log(`${name}: ${rows} rows, ${problems.length} problem(s)`);
        problems.forEach((p) => console.log("  " + p));
        bad += problems.length;
      }
      return process.exit(bad ? 1 : 0);
    }
    default: throw new Error(`unknown command ${cmd}; try --help`);
  }
}

const isMain = (() => { try { return !!process.argv[1] && realpathSync(process.argv[1]) === realpathSync(SELF); } catch { return false; } })();
if (isMain) {
  try { main(); } catch (e) { console.error(`trace: ${e.message}`); process.exit(1); }
}
