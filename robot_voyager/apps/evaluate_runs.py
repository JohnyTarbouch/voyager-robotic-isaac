"""
Benchmark runner for Voyager sessions.

Usage examples:
    python -m apps.evaluate_runs --num-runs 5 --max-tasks 10 --headless
    python -m apps.evaluate_runs --aggregate-only --num-runs 10
"""

from __future__ import annotations

import argparse
import csv
import json
import re
import subprocess
import sys
import time
from collections import Counter, defaultdict
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence


@dataclass
class RunExecution:
    run_index: int
    return_code: Optional[int]
    duration_s: Optional[float]
    summary_path: Optional[Path]
    error: Optional[str] = None


def _now_utc_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def _safe_float(value: Any, default: float = 0.0) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def _safe_int(value: Any, default: int = 0) -> int:
    try:
        return int(value)
    except (TypeError, ValueError):
        return default


def discover_session_summaries(log_root: Path) -> List[Path]:
    if not log_root.exists():
        return []
    files = list(log_root.rglob("session_summary.json"))
    files.sort(key=lambda p: (p.stat().st_mtime, str(p)))
    return files


def _parse_iso_duration_s(start_iso: Optional[str], end_iso: Optional[str]) -> Optional[float]:
    if not start_iso or not end_iso:
        return None
    try:
        start_dt = datetime.fromisoformat(start_iso)
        end_dt = datetime.fromisoformat(end_iso)
    except ValueError:
        return None
    return (end_dt - start_dt).total_seconds()


def _extract_tokens(call: Dict[str, Any]) -> Dict[str, int]:
    prompt_tokens = _safe_int(call.get("prompt_tokens"), 0)
    completion_tokens = _safe_int(call.get("completion_tokens"), 0)
    total_tokens = call.get("total_tokens")
    if total_tokens is None:
        total_tokens = prompt_tokens + completion_tokens
    else:
        total_tokens = _safe_int(total_tokens, prompt_tokens + completion_tokens)
    return {
        "prompt_tokens": prompt_tokens,
        "completion_tokens": completion_tokens,
        "total_tokens": total_tokens,
    }


def summarize_session(summary_path: Path) -> Dict[str, Any]:
    raw = json.loads(summary_path.read_text(encoding="utf-8"))
    tasks: List[Dict[str, Any]] = raw.get("tasks", []) or []
    generated_tasks: List[Dict[str, Any]] = []

    total_tasks = _safe_int(raw.get("tasks_attempted"), len(tasks))
    completed = _safe_int(raw.get("tasks_completed"), 0)
    failed = _safe_int(raw.get("tasks_failed"), 0)

    total_attempts = 0
    one_attempt_tasks = 0
    llm_calls = 0
    total_llm_latency_ms = 0.0
    total_execution_ms = 0.0
    total_prompt_tokens = 0
    total_completion_tokens = 0
    total_tokens = 0

    for task_index, task in enumerate(tasks, start=1):
        task_attempts = _safe_int(task.get("total_attempts"), 0)
        total_attempts += task_attempts
        if task_attempts == 1:
            one_attempt_tasks += 1

        task_llm_calls = _safe_int(task.get("total_llm_calls"), 0)
        llm_calls += task_llm_calls

        total_llm_latency_ms += _safe_float(task.get("total_llm_latency_ms"), 0.0)
        total_execution_ms += _safe_float(task.get("total_execution_time_ms"), 0.0)

        task_prompt_tokens = 0
        task_completion_tokens = 0
        task_total_tokens = 0
        attempts = task.get("attempts", []) or []
        for attempt in attempts:
            for call in attempt.get("llm_calls", []) or []:
                tok = _extract_tokens(call)
                total_prompt_tokens += tok["prompt_tokens"]
                total_completion_tokens += tok["completion_tokens"]
                total_tokens += tok["total_tokens"]
                task_prompt_tokens += tok["prompt_tokens"]
                task_completion_tokens += tok["completion_tokens"]
                task_total_tokens += tok["total_tokens"]

        generated_tasks.append(
            {
                "task_index": task_index,
                "task_name": task.get("task_name"),
                "task_description": task.get("task_description"),
                "task_difficulty": _safe_int(task.get("task_difficulty"), 0),
                "success": task.get("success"),
                "total_attempts": task_attempts,
                "successful_attempt": task.get("successful_attempt"),
                "total_llm_calls": task_llm_calls,
                "task_total_prompt_tokens": task_prompt_tokens,
                "task_total_completion_tokens": task_completion_tokens,
                "task_total_tokens": task_total_tokens,
                "task_total_llm_latency_s": _safe_float(task.get("total_llm_latency_ms"), 0.0) / 1000.0,
                "task_total_execution_s": _safe_float(task.get("total_execution_time_ms"), 0.0) / 1000.0,
                "saved_skill_name": task.get("saved_skill_name"),
                "saved_skill_path": task.get("saved_skill_path"),
            }
        )

    success_rate = (completed / total_tasks) if total_tasks > 0 else 0.0
    avg_attempts_per_task = (total_attempts / total_tasks) if total_tasks > 0 else 0.0
    one_attempt_rate = (one_attempt_tasks / total_tasks) if total_tasks > 0 else 0.0
    avg_llm_calls_per_task = (llm_calls / total_tasks) if total_tasks > 0 else 0.0
    avg_llm_latency_s_per_task = (total_llm_latency_ms / 1000.0 / total_tasks) if total_tasks > 0 else 0.0
    avg_execution_s_per_task = (total_execution_ms / 1000.0 / total_tasks) if total_tasks > 0 else 0.0
    tokens_per_successful_task = (total_tokens / completed) if completed > 0 else None

    return {
        "session_id": raw.get("session_id"),
        "summary_path": str(summary_path),
        "backend": raw.get("backend"),
        "llm_model": raw.get("llm_model"),
        "max_attempts": _safe_int(raw.get("max_attempts"), 0),
        "start_time": raw.get("start_time"),
        "end_time": raw.get("end_time"),
        "session_duration_s": _parse_iso_duration_s(raw.get("start_time"), raw.get("end_time")),
        "tasks_attempted": total_tasks,
        "tasks_completed": completed,
        "tasks_failed": failed,
        "success_rate": success_rate,
        "avg_attempts_per_task": avg_attempts_per_task,
        "one_attempt_rate": one_attempt_rate,
        "llm_calls": llm_calls,
        "avg_llm_calls_per_task": avg_llm_calls_per_task,
        "total_llm_latency_s": total_llm_latency_ms / 1000.0,
        "avg_llm_latency_s_per_task": avg_llm_latency_s_per_task,
        "total_execution_s": total_execution_ms / 1000.0,
        "avg_execution_s_per_task": avg_execution_s_per_task,
        "total_prompt_tokens": total_prompt_tokens,
        "total_completion_tokens": total_completion_tokens,
        "total_tokens": total_tokens,
        "tokens_per_successful_task": tokens_per_successful_task,
        "generated_tasks": generated_tasks,
    }


def aggregate_runs(run_metrics: Sequence[Dict[str, Any]]) -> Dict[str, Any]:
    run_count = len(run_metrics)
    if run_count == 0:
        return {
            "run_count": 0,
            "total_tasks_attempted": 0,
            "total_tasks_completed": 0,
            "total_tasks_failed": 0,
            "overall_success_rate": 0.0,
            "mean_run_success_rate": 0.0,
            "mean_avg_attempts_per_task": 0.0,
            "mean_avg_llm_calls_per_task": 0.0,
            "mean_one_attempt_rate": 0.0,
            "total_tokens": 0,
            "mean_tokens_per_successful_task": None,
            "mean_session_duration_s": None,
        }

    total_tasks_attempted = sum(_safe_int(x.get("tasks_attempted"), 0) for x in run_metrics)
    total_tasks_completed = sum(_safe_int(x.get("tasks_completed"), 0) for x in run_metrics)
    total_tasks_failed = sum(_safe_int(x.get("tasks_failed"), 0) for x in run_metrics)
    total_tokens = sum(_safe_int(x.get("total_tokens"), 0) for x in run_metrics)

    overall_success_rate = (total_tasks_completed / total_tasks_attempted) if total_tasks_attempted > 0 else 0.0
    mean_run_success_rate = sum(_safe_float(x.get("success_rate"), 0.0) for x in run_metrics) / run_count
    mean_avg_attempts_per_task = sum(_safe_float(x.get("avg_attempts_per_task"), 0.0) for x in run_metrics) / run_count
    mean_avg_llm_calls_per_task = sum(_safe_float(x.get("avg_llm_calls_per_task"), 0.0) for x in run_metrics) / run_count
    mean_one_attempt_rate = sum(_safe_float(x.get("one_attempt_rate"), 0.0) for x in run_metrics) / run_count

    token_per_success_values = [
        _safe_float(x.get("tokens_per_successful_task"), 0.0)
        for x in run_metrics
        if x.get("tokens_per_successful_task") is not None
    ]
    mean_tokens_per_successful_task = (
        sum(token_per_success_values) / len(token_per_success_values)
        if token_per_success_values
        else None
    )

    duration_values = [
        _safe_float(x.get("session_duration_s"), 0.0)
        for x in run_metrics
        if x.get("session_duration_s") is not None
    ]
    mean_session_duration_s = (sum(duration_values) / len(duration_values)) if duration_values else None

    return {
        "run_count": run_count,
        "total_tasks_attempted": total_tasks_attempted,
        "total_tasks_completed": total_tasks_completed,
        "total_tasks_failed": total_tasks_failed,
        "overall_success_rate": overall_success_rate,
        "mean_run_success_rate": mean_run_success_rate,
        "mean_avg_attempts_per_task": mean_avg_attempts_per_task,
        "mean_avg_llm_calls_per_task": mean_avg_llm_calls_per_task,
        "mean_one_attempt_rate": mean_one_attempt_rate,
        "total_tokens": total_tokens,
        "mean_tokens_per_successful_task": mean_tokens_per_successful_task,
        "mean_session_duration_s": mean_session_duration_s,
    }


def run_multiple_sessions(
    num_runs: int,
    max_tasks: int,
    headless: bool,
    logs_root: Path,
    timeout_per_run_s: Optional[int],
    project_root: Path,
) -> List[RunExecution]:
    existing = {str(path.resolve()) for path in discover_session_summaries(logs_root)}
    results: List[RunExecution] = []

    for run_idx in range(1, num_runs + 1):
        cmd = [sys.executable, "-m", "apps.run_voyager", "--max-tasks", str(max_tasks)]
        if headless:
            cmd.append("--headless")

        print(f"[run {run_idx}/{num_runs}] Executing: {' '.join(cmd)}")
        start_t = time.time()
        return_code: Optional[int] = None
        error_msg: Optional[str] = None
        try:
            completed = subprocess.run(
                cmd,
                cwd=str(project_root),
                timeout=timeout_per_run_s if timeout_per_run_s and timeout_per_run_s > 0 else None,
                check=False,
            )
            return_code = completed.returncode
        except subprocess.TimeoutExpired as exc:
            return_code = 124
            error_msg = f"timeout after {exc.timeout}s"
        except Exception as exc:  # pragma: no cover
            return_code = 1
            error_msg = str(exc)
        duration_s = time.time() - start_t

        discovered = discover_session_summaries(logs_root)
        new_paths = [p for p in discovered if str(p.resolve()) not in existing]
        for path in new_paths:
            existing.add(str(path.resolve()))
        summary_path = new_paths[-1] if new_paths else None

        results.append(
            RunExecution(
                run_index=run_idx,
                return_code=return_code,
                duration_s=duration_s,
                summary_path=summary_path,
                error=error_msg,
            )
        )

        if summary_path:
            print(f"[run {run_idx}/{num_runs}] Summary: {summary_path}")
        else:
            print(f"[run {run_idx}/{num_runs}] No new session_summary.json found")

    return results


def _round_for_output(metrics: Dict[str, Any]) -> Dict[str, Any]:
    rounded = dict(metrics)
    for key in [
        "success_rate",
        "avg_attempts_per_task",
        "one_attempt_rate",
        "avg_llm_calls_per_task",
        "total_llm_latency_s",
        "avg_llm_latency_s_per_task",
        "total_execution_s",
        "avg_execution_s_per_task",
        "tokens_per_successful_task",
        "session_duration_s",
    ]:
        if rounded.get(key) is not None:
            rounded[key] = round(float(rounded[key]), 4)
    return rounded


def _write_csv(path: Path, rows: Sequence[Dict[str, Any]]) -> None:
    if not rows:
        path.write_text("", encoding="utf-8")
        return
    fieldnames = sorted({key for row in rows for key in row.keys()})
    with path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def _generate_plots(
    run_rows: Sequence[Dict[str, Any]],
    aggregate: Dict[str, Any],
    generated_tasks_rows: Sequence[Dict[str, Any]],
    output_dir: Path,
    enabled: bool = True,
) -> List[Path]:
    if not enabled:
        return []

    metric_rows = [row for row in run_rows if row.get("tasks_attempted") is not None]
    if not metric_rows:
        return []

    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception as exc:
        print(f"- Plot generation skipped: {exc}")
        return []

    run_ids = [f"R{_safe_int(row.get('run_index'), i + 1)}" for i, row in enumerate(metric_rows)]
    x = list(range(len(metric_rows)))
    width = 0.38

    success_pct = [_safe_float(row.get("success_rate"), 0.0) * 100.0 for row in metric_rows]
    tasks_attempted = [_safe_int(row.get("tasks_attempted"), 0) for row in metric_rows]
    tasks_completed = [_safe_int(row.get("tasks_completed"), 0) for row in metric_rows]
    avg_attempts = [_safe_float(row.get("avg_attempts_per_task"), 0.0) for row in metric_rows]
    avg_llm_calls = [_safe_float(row.get("avg_llm_calls_per_task"), 0.0) for row in metric_rows]
    total_tokens_k = [_safe_int(row.get("total_tokens"), 0) / 1000.0 for row in metric_rows]

    llm_latency_s = [_safe_float(row.get("total_llm_latency_s"), 0.0) for row in metric_rows]
    exec_time_s = [_safe_float(row.get("total_execution_s"), 0.0) for row in metric_rows]
    session_duration_s = [
        _safe_float(row.get("session_duration_s"), _safe_float(row.get("run_duration_s"), 0.0))
        for row in metric_rows
    ]

    run_plot_path = output_dir / "runs_kpis.png"
    fig, axes = plt.subplots(2, 2, figsize=(14, 9))

    ax = axes[0][0]
    ax.bar(x, success_pct, color="#2a9d8f", edgecolor="black")
    ax.set_title("Success Rate by Run")
    ax.set_ylabel("Success (%)")
    ax.set_ylim(0, 100)
    ax.set_xticks(x)
    ax.set_xticklabels(run_ids)

    ax = axes[0][1]
    ax.bar([v - width / 2 for v in x], tasks_attempted, width=width, label="Tasks Attempted", color="#8ecae6")
    ax.bar([v + width / 2 for v in x], tasks_completed, width=width, label="Tasks Completed", color="#219ebc")
    ax.set_title("Tasks by Run")
    ax.set_ylabel("Task Count")
    ax.set_xticks(x)
    ax.set_xticklabels(run_ids)
    ax.legend()

    ax = axes[1][0]
    ax.bar([v - width / 2 for v in x], avg_attempts, width=width, label="Avg Attempts/Task", color="#ffb703")
    ax.bar([v + width / 2 for v in x], avg_llm_calls, width=width, label="Avg LLM Calls/Task", color="#fb8500")
    ax.set_title("Efficiency by Run")
    ax.set_ylabel("Count")
    ax.set_xticks(x)
    ax.set_xticklabels(run_ids)
    ax.legend()

    ax = axes[1][1]
    ax.bar(x, total_tokens_k, color="#6a4c93", edgecolor="black")
    ax.set_title("Token Usage by Run")
    ax.set_ylabel("Total Tokens (thousands)")
    ax.set_xticks(x)
    ax.set_xticklabels(run_ids)

    fig.tight_layout()
    fig.savefig(run_plot_path, dpi=130)
    plt.close(fig)

    timing_plot_path = output_dir / "runs_timing.png"
    fig, axes = plt.subplots(1, 2, figsize=(14, 5))

    ax = axes[0]
    ax.bar([v - width / 2 for v in x], llm_latency_s, width=width, label="Total LLM Latency (s)", color="#90be6d")
    ax.bar([v + width / 2 for v in x], exec_time_s, width=width, label="Total Execution Time (s)", color="#577590")
    ax.set_title("LLM vs Execution Time by Run")
    ax.set_ylabel("Seconds")
    ax.set_xticks(x)
    ax.set_xticklabels(run_ids)
    ax.legend()

    ax = axes[1]
    ax.bar(x, session_duration_s, color="#4d908e", edgecolor="black")
    ax.set_title("Session Duration by Run")
    ax.set_ylabel("Seconds")
    ax.set_xticks(x)
    ax.set_xticklabels(run_ids)

    fig.tight_layout()
    fig.savefig(timing_plot_path, dpi=130)
    plt.close(fig)

    aggregate_plot_path = output_dir / "aggregate_kpis.png"
    fig, axes = plt.subplots(1, 2, figsize=(12, 5))

    pct_names = ["Overall Success", "Mean Run Success", "One-Shot Rate"]
    pct_values = [
        _safe_float(aggregate.get("overall_success_rate"), 0.0) * 100.0,
        _safe_float(aggregate.get("mean_run_success_rate"), 0.0) * 100.0,
        _safe_float(aggregate.get("mean_one_attempt_rate"), 0.0) * 100.0,
    ]
    axes[0].bar(pct_names, pct_values, color=["#2a9d8f", "#219ebc", "#8ecae6"], edgecolor="black")
    axes[0].set_ylim(0, 100)
    axes[0].set_ylabel("Percent")
    axes[0].set_title("Aggregate Success Metrics")

    eff_names = ["Attempts/Task", "LLM Calls/Task", "Tokens/Success (k)", "Session Duration (min)"]
    tokens_per_success = aggregate.get("mean_tokens_per_successful_task")
    duration_s = aggregate.get("mean_session_duration_s")
    eff_values = [
        _safe_float(aggregate.get("mean_avg_attempts_per_task"), 0.0),
        _safe_float(aggregate.get("mean_avg_llm_calls_per_task"), 0.0),
        (_safe_float(tokens_per_success, 0.0) / 1000.0) if tokens_per_success is not None else 0.0,
        (_safe_float(duration_s, 0.0) / 60.0) if duration_s is not None else 0.0,
    ]
    axes[1].bar(eff_names, eff_values, color=["#ffb703", "#fb8500", "#6a4c93", "#4d908e"], edgecolor="black")
    axes[1].set_title("Aggregate Efficiency Metrics")
    axes[1].set_ylabel("Value")
    axes[1].tick_params(axis="x", labelrotation=15)

    fig.tight_layout()
    fig.savefig(aggregate_plot_path, dpi=130)
    plt.close(fig)

    output_paths = [run_plot_path, timing_plot_path, aggregate_plot_path]

    if generated_tasks_rows:
        run_to_counts: Dict[int, Counter[str]] = defaultdict(Counter)
        total_counts: Counter[str] = Counter()

        def _canonical_skill_label(raw_label: Any) -> str:
            label = str(raw_label or "").strip()
            if not label:
                return "unknown"
            # Strip generated timestamp/version suffixes for better grouping.
            label = re.sub(r"_v\d{8,}$", "", label)
            label = re.sub(r"\s+", " ", label)
            return label

        def _short_label(label: str, max_len: int = 26) -> str:
            if len(label) <= max_len:
                return label
            return f"{label[:max_len-3]}..."

        for row in generated_tasks_rows:
            run_idx = _safe_int(row.get("run_index"), 0)
            skill_name = row.get("saved_skill_name")
            task_name = row.get("task_name")
            label = _canonical_skill_label(skill_name or task_name or f"task_{_safe_int(row.get('task_index'), 0)}")
            run_to_counts[run_idx][label] += 1
            total_counts[label] += 1

        ordered_runs = sorted(run_to_counts.keys())
        if ordered_runs and total_counts:
            # Compact combined plot: run x top-skills matrix.
            top_n_skills = 12
            top_skills = [name for name, _ in total_counts.most_common(top_n_skills)]
            has_other = len(total_counts) > len(top_skills)
            skill_columns = top_skills + (["Other"] if has_other else [])

            matrix: List[List[int]] = []
            for run_idx in ordered_runs:
                counts = run_to_counts[run_idx]
                row_vals = [counts.get(skill_name, 0) for skill_name in top_skills]
                if has_other:
                    other_count = 0
                    for skill_name, count in counts.items():
                        if skill_name not in top_skills:
                            other_count += count
                    row_vals.append(other_count)
                matrix.append(row_vals)

            skills_plot_path = output_dir / "skills_by_run.png"
            fig_w = max(8.0, 0.95 * len(skill_columns) + 3.0)
            fig_h = max(3.8, 0.6 * len(ordered_runs) + 2.0)
            fig, ax = plt.subplots(1, 1, figsize=(fig_w, fig_h))

            image = ax.imshow(matrix, aspect="auto", cmap="YlGnBu")
            ax.set_title("Skills Used/Generated by Run (Combined)")
            ax.set_xlabel("Skill")
            ax.set_ylabel("Run")
            ax.set_yticks(list(range(len(ordered_runs))))
            ax.set_yticklabels([f"R{run_idx}" for run_idx in ordered_runs], fontsize=9)
            ax.set_xticks(list(range(len(skill_columns))))
            ax.set_xticklabels([_short_label(name) for name in skill_columns], rotation=35, ha="right", fontsize=8)

            max_value = max((max(row) for row in matrix), default=0)
            for i, row in enumerate(matrix):
                for j, value in enumerate(row):
                    if value <= 0:
                        continue
                    text_color = "white" if max_value > 0 and value >= (0.55 * max_value) else "black"
                    ax.text(j, i, str(value), ha="center", va="center", fontsize=8, color=text_color)

            cbar = fig.colorbar(image, ax=ax, fraction=0.035, pad=0.02)
            cbar.set_label("Count")

            fig.tight_layout()
            fig.savefig(skills_plot_path, dpi=130)
            plt.close(fig)
            output_paths.append(skills_plot_path)

    return output_paths


def main() -> None:
    parser = argparse.ArgumentParser(description="Evaluate Voyager across multiple runs")
    parser.add_argument("--num-runs", type=int, default=3, help="Number of runs to execute (or analyze)")
    parser.add_argument("--max-tasks", type=int, default=10, help="Max tasks per run when executing new runs")
    parser.add_argument("--headless", action="store_true", help="Run Isaac Sim headless")
    parser.add_argument(
        "--aggregate-only",
        action="store_true",
        help="Do not execute runs; aggregate existing session summaries",
    )
    parser.add_argument(
        "--logs-root",
        type=str,
        default="logs",
        help="Path to logs root directory (relative to robot_voyager)",
    )
    parser.add_argument(
        "--output-root",
        type=str,
        default="evaluations",
        help="Path to evaluation output root (relative to robot_voyager)",
    )
    parser.add_argument(
        "--timeout-per-run-s",
        type=int,
        default=0,
        help="Optional timeout per run in seconds (0 disables timeout)",
    )
    parser.add_argument("--no-plots", action="store_true", help="Disable PNG plot generation")
    args = parser.parse_args()

    project_root = Path(__file__).resolve().parent.parent
    logs_root = (project_root / args.logs_root).resolve()
    output_root = (project_root / args.output_root).resolve()

    run_execs: List[RunExecution] = []
    summary_paths: List[Path] = []

    if args.aggregate_only:
        discovered = discover_session_summaries(logs_root)
        if args.num_runs > 0:
            discovered = discovered[-args.num_runs :]
        summary_paths = discovered
        print(f"Aggregate-only mode: analyzing {len(summary_paths)} session summaries")
    else:
        run_execs = run_multiple_sessions(
            num_runs=args.num_runs,
            max_tasks=args.max_tasks,
            headless=args.headless,
            logs_root=logs_root,
            timeout_per_run_s=args.timeout_per_run_s,
            project_root=project_root,
        )
        summary_paths = [x.summary_path for x in run_execs if x.summary_path is not None]

    run_rows: List[Dict[str, Any]] = []
    generated_tasks_rows: List[Dict[str, Any]] = []

    if args.aggregate_only:
        for idx, path in enumerate(summary_paths, start=1):
            try:
                metrics = summarize_session(path)
            except Exception as exc:
                run_rows.append(
                    {
                        "run_index": idx,
                        "return_code": None,
                        "summary_path": str(path),
                        "error": str(exc),
                    }
                )
                continue
            task_items = metrics.pop("generated_tasks", [])
            for task_item in task_items:
                generated_tasks_rows.append(
                    {
                        "run_index": idx,
                        "session_id": metrics.get("session_id"),
                        "summary_path": metrics.get("summary_path"),
                        **task_item,
                    }
                )
            row = {"run_index": idx, "return_code": None, "error": None, **_round_for_output(metrics)}
            run_rows.append(row)
    else:
        for run_exec in run_execs:
            if run_exec.summary_path is None:
                run_rows.append(
                    {
                        "run_index": run_exec.run_index,
                        "return_code": run_exec.return_code,
                        "run_duration_s": round(_safe_float(run_exec.duration_s, 0.0), 4),
                        "summary_path": None,
                        "error": run_exec.error or "missing_session_summary",
                    }
                )
                continue

            try:
                metrics = summarize_session(run_exec.summary_path)
            except Exception as exc:
                run_rows.append(
                    {
                        "run_index": run_exec.run_index,
                        "return_code": run_exec.return_code,
                        "run_duration_s": round(_safe_float(run_exec.duration_s, 0.0), 4),
                        "summary_path": str(run_exec.summary_path),
                        "error": str(exc),
                    }
                )
                continue
            task_items = metrics.pop("generated_tasks", [])
            for task_item in task_items:
                generated_tasks_rows.append(
                    {
                        "run_index": run_exec.run_index,
                        "session_id": metrics.get("session_id"),
                        "summary_path": metrics.get("summary_path"),
                        **task_item,
                    }
                )

            row = {
                "run_index": run_exec.run_index,
                "return_code": run_exec.return_code,
                "run_duration_s": round(_safe_float(run_exec.duration_s, 0.0), 4),
                "error": run_exec.error,
                **_round_for_output(metrics),
            }
            run_rows.append(row)

    metric_rows = [row for row in run_rows if row.get("tasks_attempted") is not None]
    aggregate = aggregate_runs(metric_rows)
    aggregate = {k: (round(v, 4) if isinstance(v, float) else v) for k, v in aggregate.items()}

    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_dir = output_root / ts
    output_dir.mkdir(parents=True, exist_ok=True)

    plot_paths = _generate_plots(
        run_rows,
        aggregate,
        generated_tasks_rows,
        output_dir,
        enabled=not args.no_plots,
    )

    summary_out = {
        "created_at_utc": _now_utc_iso(),
        "mode": "aggregate_only" if args.aggregate_only else "run_and_aggregate",
        "settings": {
            "num_runs": args.num_runs,
            "max_tasks": args.max_tasks,
            "headless": args.headless,
            "logs_root": str(logs_root),
            "output_root": str(output_root),
            "timeout_per_run_s": args.timeout_per_run_s,
            "no_plots": args.no_plots,
        },
        "runs": run_rows,
        "generated_tasks_count": len(generated_tasks_rows),
        "generated_tasks": generated_tasks_rows,
        "aggregate": aggregate,
        "plot_files": [str(path) for path in plot_paths],
    }

    summary_path = output_dir / "summary.json"
    csv_path = output_dir / "runs.csv"
    tasks_csv_path = output_dir / "tasks.csv"

    summary_path.write_text(json.dumps(summary_out, indent=2), encoding="utf-8")
    _write_csv(csv_path, run_rows)
    _write_csv(tasks_csv_path, generated_tasks_rows)

    print("")
    print("Evaluation complete")
    print(f"- Output directory: {output_dir}")
    print(f"- Summary JSON: {summary_path}")
    print(f"- Runs CSV: {csv_path}")
    print(f"- Tasks CSV: {tasks_csv_path}")
    if plot_paths:
        for path in plot_paths:
            print(f"- Plot: {path}")
    if generated_tasks_rows:
        print("")
        print("Generated Tasks (first 12)")
        for task_row in generated_tasks_rows[:12]:
            task_name = task_row.get("task_name") or "unknown_task"
            print(f"- [R{task_row['run_index']}] {task_name}")
        remaining = len(generated_tasks_rows) - 12
        if remaining > 0:
            print(f"- ... and {remaining} more")
    print("")
    print("Aggregate KPIs")
    print(f"- Runs analyzed: {aggregate['run_count']}")
    print(f"- Tasks attempted: {aggregate['total_tasks_attempted']}")
    print(f"- Tasks completed: {aggregate['total_tasks_completed']}")
    print(f"- Overall success rate: {aggregate['overall_success_rate']:.1%}")
    print(f"- Mean run success rate: {aggregate['mean_run_success_rate']:.1%}")
    print(f"- Mean attempts/task: {aggregate['mean_avg_attempts_per_task']:.3f}")
    print(f"- Mean LLM calls/task: {aggregate['mean_avg_llm_calls_per_task']:.3f}")
    print(f"- Mean one-shot rate: {aggregate['mean_one_attempt_rate']:.1%}")
    print(f"- Total tokens: {aggregate['total_tokens']}")
    if aggregate["mean_tokens_per_successful_task"] is not None:
        print(f"- Mean tokens/successful task: {aggregate['mean_tokens_per_successful_task']:.1f}")
    if aggregate["mean_session_duration_s"] is not None:
        print(f"- Mean session duration: {aggregate['mean_session_duration_s']:.1f}s")


if __name__ == "__main__":
    main()
