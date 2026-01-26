
import json
import matplotlib.pyplot as plt
from matplotlib.patches import Patch

LOG_PATH = r"C:\isaacsim\standalone_examples\voyager-robotic-isaac-0\robot_voyager\logs\20251216_224929\metrics.jsonl"


def load_jsonl(path: str):
    with open(path, "r", encoding="utf-8") as f:
        return [json.loads(line) for line in f if line.strip()]


def get_max_attempts(events):
    for e in events:
        if e.get("event") == "session_start":
            return e.get("data", {}).get("max_attempts")
    return None


def uniquify(name: str, seen: dict) -> str:
    if name not in seen:
        seen[name] = 1
        return name
    seen[name] += 1
    return f"{name} ({seen[name]})"


def parse_from_session_end(events):
    """Best path when session_end exists: it has per-task attempt details."""
    session_end = None
    for e in events:
        if e.get("event") == "session_end":
            session_end = e  
    if not session_end:
        return None

    tasks = []
    seen = {}

    for t in session_end.get("data", {}).get("tasks", []) or []:
        name_raw = t.get("task_name") or t.get("task") or "UNKNOWN_TASK"
        name = uniquify(name_raw, seen)

        attempts_list = t.get("attempts", [])
        if not isinstance(attempts_list, list):
            attempts_list = []

        success_attempts = sum(1 for a in attempts_list if a.get("verifier_passed") is True)
        failed_attempts = sum(1 for a in attempts_list if a.get("verifier_passed") is False)

        total = t.get("total_attempts")
        if not isinstance(total, int):
            total = len(attempts_list)

        success = t.get("success")
        if isinstance(success, bool):
            # If counts don't reconcile, fall back to the known invariant:
            # a succeeded task has exactly 1 successful attempt, failures are the rest.
            if total and (success_attempts + failed_attempts) != total:
                if success:
                    success_attempts = 1
                    failed_attempts = max(total - 1, 0)
                else:
                    success_attempts = 0
                    failed_attempts = total
        else:
            success = None  # unknown

        # If a task says success but we somehow saw 0 success attempts, fix it.
        if success is True and success_attempts == 0 and total:
            success_attempts = 1
            failed_attempts = max(total - 1, 0)

        tasks.append(
            {
                "name": name,
                "total_attempts": total,
                "success_attempts": success_attempts,
                "failed_attempts": failed_attempts,
                "success": success,
            }
        )

    return tasks


def parse_streaming(events, max_attempts):
    """Fallback when session_end isn't present: infer from task_start/attempt_end/task_end."""
    tasks = []
    seen = {}

    cur = None

    def finalize(run):
        if not run:
            return

        # infer total attempts
        total = run["max_attempt"]
        if run["attempt_ends"]:
            total = max(total, len(run["attempt_ends"]))

        # infer success if missing task_end
        if run["success"] is None:
            if any(run["attempt_ends"]):
                run["success"] = True
            elif max_attempts is not None and total >= max_attempts:
                run["success"] = False

        # derive success/failed attempts split
        if run["attempt_ends"]:
            success_attempts = sum(1 for x in run["attempt_ends"] if x is True)
            failed_attempts = sum(1 for x in run["attempt_ends"] if x is False)
        else:
            # no per-attempt verifier info; fall back to invariant
            if run["success"] is True and total:
                success_attempts = 1
                failed_attempts = max(total - 1, 0)
            elif run["success"] is False:
                success_attempts = 0
                failed_attempts = total
            else:
                success_attempts = 0
                failed_attempts = total

        tasks.append(
            {
                "name": run["name"],
                "total_attempts": total,
                "success_attempts": success_attempts,
                "failed_attempts": failed_attempts,
                "success": run["success"],
            }
        )

    for e in events:
        ev = e.get("event")

        if ev == "task_start":
            if cur is not None:
                finalize(cur)
            name = uniquify(e.get("task", "UNKNOWN_TASK"), seen)
            cur = {"name": name, "max_attempt": 0, "attempt_ends": [], "success": None}

        elif ev == "attempt_end":
            if cur is None:
                name = uniquify(e.get("task", "UNKNOWN_TASK"), seen)
                cur = {"name": name, "max_attempt": 0, "attempt_ends": [], "success": None}
            cur["max_attempt"] = max(cur["max_attempt"], int(e.get("attempt", 0) or 0))
            cur["attempt_ends"].append(bool(e.get("verifier_passed")))

        elif ev == "attempt_start":
            if cur is None:
                name = uniquify(e.get("task", "UNKNOWN_TASK"), seen)
                cur = {"name": name, "max_attempt": 0, "attempt_ends": [], "success": None}
            cur["max_attempt"] = max(cur["max_attempt"], int(e.get("attempt", 0) or 0))

        elif ev == "task_end":
            if cur is None:
                name = uniquify(e.get("task", "UNKNOWN_TASK"), seen)
                cur = {"name": name, "max_attempt": 0, "attempt_ends": [], "success": None}
            cur["success"] = bool(e.get("success"))
            cur["max_attempt"] = max(cur["max_attempt"], int(e.get("attempts", 0) or 0))
            finalize(cur)
            cur = None

    if cur is not None:
        finalize(cur)

    return tasks


# -------- main --------
events = load_jsonl(LOG_PATH)
max_attempts = get_max_attempts(events)

tasks = parse_from_session_end(events)
if tasks is None:
    tasks = parse_streaming(events, max_attempts)

if not tasks:
    raise RuntimeError("No tasks found in the log.")

names = [t["name"] for t in tasks]
failed = [t["failed_attempts"] for t in tasks]
succ_att = [t["success_attempts"] for t in tasks]
total = [t["total_attempts"] for t in tasks]

def status_char(s):
    return "S" if s is True else ("F" if s is False else "?")

statuses = [status_char(t["success"]) for t in tasks]

# x labels (truncate)
xt = [n if len(n) <= 28 else (n[:25] + "...") for n in names]

fig, ax = plt.subplots(figsize=(12, 6))

x = list(range(len(tasks)))

# Stacked bars: failed attempts (red) + successful attempts (green)
ax.bar(x, failed, color="red", alpha=0.7, edgecolor="black", label="Failed attempts")
ax.bar(x, succ_att, bottom=failed, color="green", alpha=0.7, edgecolor="black", label="Successful attempt")

ax.set_xticks(x)
ax.set_xticklabels(xt, rotation=45, ha="right", fontsize=10)
ax.set_ylabel("Attempts", fontsize=12)
ax.set_xlabel("Task", fontsize=12)
ax.set_title("Voyager Training: Tasks & Attempts", fontsize=14, fontweight="bold")

# max_attempts reference line
top = max(total) if total else 1
if isinstance(max_attempts, int):
    top = max(top, max_attempts)
    ax.axhline(max_attempts, linestyle="--", linewidth=1)
ax.set_ylim(0, top + 0.75)

# annotate total + final status above each bar
for i in range(len(tasks)):
    ax.text(i, total[i] + 0.08, statuses[i], ha="center", va="bottom", fontsize=14, fontweight="bold")

# stats box (final task outcomes)
succ_tasks = sum(1 for t in tasks if t["success"] is True)
fail_tasks = sum(1 for t in tasks if t["success"] is False)
unk_tasks = sum(1 for t in tasks if t["success"] is None)

ax.text(
    0.98,
    0.98,
    f"Success: {succ_tasks}/{len(tasks)}  Failed: {fail_tasks}  Unknown: {unk_tasks}",
    transform=ax.transAxes,
    ha="right",
    va="top",
    fontsize=12,
    bbox=dict(boxstyle="round", facecolor="wheat", alpha=0.5),
)

ax.legend(loc="upper left")

plt.tight_layout()
out_path = LOG_PATH.replace("metrics.jsonl", "tasks_plot.png")
plt.savefig(out_path, dpi=120)
plt.show()

print(f"Saved plot: {out_path}")
for t in tasks:
    print(f"  {status_char(t['success'])} {t['name']}  total={t['total_attempts']}  failed={t['failed_attempts']}  success={t['success_attempts']}")
