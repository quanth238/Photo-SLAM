#!/usr/bin/env python3

import argparse
import csv
import json
import math
from collections import Counter, defaultdict
from pathlib import Path


NUM_FIELDS = [
    "timestamp_ms",
    "ref_kfid",
    "neighbor_kfid",
    "matches_raw",
    "matches_in_bounds",
    "oversample",
    "triangulated",
    "cheirality_pass",
    "reproj_pass",
    "parallax_pass",
    "seeds_out",
    "median_reproj",
    "p90_reproj",
    "median_parallax",
    "p90_parallax",
    "task_ms",
    "queue_size",
    "iter",
]


def parse_num(val):
    if val is None:
        return None
    s = str(val).strip()
    if s == "":
        return None
    try:
        v = float(s)
    except ValueError:
        return None
    # Treat negative sentinel values as missing
    if v < 0:
        return None
    return v


def pct(vals, p):
    if not vals:
        return None
    s = sorted(vals)
    idx = int(round(p * (len(s) - 1)))
    return s[idx]


def summarize(vals):
    if not vals:
        return None
    s = sorted(vals)
    return {
        "count": len(s),
        "min": s[0],
        "p50": s[len(s) // 2],
        "p90": pct(s, 0.9),
        "mean": sum(s) / len(s),
        "max": s[-1],
    }


def safe_div(num, den):
    if den is None or den == 0:
        return None
    return num / den


def fmt_num(v, digits=4):
    if v is None:
        return "NA"
    if abs(v) >= 1000 or abs(v) < 0.01:
        return f"{v:.{digits}g}"
    return f"{v:.{digits}f}"


def print_summary(title, stats):
    if stats is None:
        print(f"{title}: NA")
        return
    print(
        f"{title}: n={stats['count']} min={fmt_num(stats['min'])} "
        f"p50={fmt_num(stats['p50'])} p90={fmt_num(stats['p90'])} "
        f"mean={fmt_num(stats['mean'])} max={fmt_num(stats['max'])}"
    )


def main():
    parser = argparse.ArgumentParser(
        description="Summarize CorrInit statistics from corr_log.csv"
    )
    parser.add_argument(
        "--csv",
        type=str,
        default="corr_log.csv",
        help="Path to corr_log.csv",
    )
    parser.add_argument(
        "--out",
        type=str,
        default=None,
        help="Optional output text summary file",
    )
    parser.add_argument(
        "--out_json",
        type=str,
        default=None,
        help="Optional output JSON summary file",
    )
    parser.add_argument(
        "--out_csv",
        type=str,
        default=None,
        help="Optional annotated CSV (per-task, derived ratios and limit reason).",
    )
    parser.add_argument(
        "--num_seeds",
        type=int,
        default=None,
        help="Optional target num_seeds to evaluate coverage (e.g., 4096).",
    )
    args = parser.parse_args()

    csv_path = Path(args.csv)
    if not csv_path.exists():
        raise SystemExit(f"CSV not found: {csv_path}")

    rows = []
    with csv_path.open(newline="") as f:
        reader = csv.DictReader(f)
        for r in reader:
            for k in NUM_FIELDS:
                r[k] = parse_num(r.get(k))
            rows.append(r)

    events = Counter(r.get("event") for r in rows)

    # Prefer task_final if present; else task; else task_nb
    task_event = "task_final" if events.get("task_final") else "task"
    if events.get(task_event, 0) == 0 and events.get("task_nb"):
        task_event = "task_nb"
    task_rows = [r for r in rows if r.get("event") == task_event]

    # Optional output capture
    output_lines = []

    def oprint(s=""):
        print(s)
        output_lines.append(s)

    oprint(f"CorrInit stats from: {csv_path}")
    oprint("")

    # Overview
    oprint("Overview")
    oprint(f"  total_rows: {len(rows)}")
    oprint(f"  events: {dict(events)}")
    if rows:
        t0 = min(r["timestamp_ms"] for r in rows if r["timestamp_ms"] is not None)
        t1 = max(r["timestamp_ms"] for r in rows if r["timestamp_ms"] is not None)
        if t0 is not None and t1 is not None:
            oprint(f"  time_span_s: {fmt_num((t1 - t0) / 1000.0, 3)}")
    oprint(f"  task_event_used: {task_event} (count={len(task_rows)})")
    oprint("")

    # Core task stats
    oprint("Task Stats")
    for key in [
        "matches_raw",
        "matches_in_bounds",
        "oversample",
        "triangulated",
        "cheirality_pass",
        "reproj_pass",
        "parallax_pass",
        "seeds_out",
        "median_reproj",
        "p90_reproj",
        "median_parallax",
        "p90_parallax",
        "task_ms",
        "queue_size",
    ]:
        vals = [r[key] for r in task_rows if r.get(key) is not None]
        oprint_summary = summarize(vals)
        oprint(f"  {key}: " + (f"n={oprint_summary['count']}" if oprint_summary else "NA"))
        if oprint_summary:
            oprint(
                f"    min={fmt_num(oprint_summary['min'])} "
                f"p50={fmt_num(oprint_summary['p50'])} "
                f"p90={fmt_num(oprint_summary['p90'])} "
                f"mean={fmt_num(oprint_summary['mean'])} "
                f"max={fmt_num(oprint_summary['max'])}"
            )
    oprint("")

    # Ratios
    oprint("Ratios (per task)")
    ratios = defaultdict(list)
    for r in task_rows:
        ratios["in_bounds/raw"].append(
            safe_div(r.get("matches_in_bounds"), r.get("matches_raw"))
        )
        ratios["cheirality/in_bounds"].append(
            safe_div(r.get("cheirality_pass"), r.get("matches_in_bounds"))
        )
        ratios["reproj/cheirality"].append(
            safe_div(r.get("reproj_pass"), r.get("cheirality_pass"))
        )
        ratios["parallax/reproj"].append(
            safe_div(r.get("parallax_pass"), r.get("reproj_pass"))
        )
        ratios["seeds/parallax"].append(
            safe_div(r.get("seeds_out"), r.get("parallax_pass"))
        )
    for k, vals in ratios.items():
        vals = [v for v in vals if v is not None]
        s = summarize(vals)
        if s:
            oprint(
                f"  {k}: p50={fmt_num(s['p50'])} "
                f"p90={fmt_num(s['p90'])} mean={fmt_num(s['mean'])}"
            )
        else:
            oprint(f"  {k}: NA")
    oprint("")

    # Neighbor stats
    oprint("Neighbor Stats")
    ref_ids = {int(r["ref_kfid"]) for r in task_rows if r.get("ref_kfid") is not None}
    neigh_ids = {int(r["neighbor_kfid"]) for r in task_rows if r.get("neighbor_kfid") is not None}
    oprint(f"  unique_ref_kfid: {len(ref_ids)}")
    oprint(f"  unique_neighbor_kfid: {len(neigh_ids)}")
    if len(neigh_ids) == 1 and 0 in neigh_ids:
        oprint("  note: neighbor_kfid always 0 in this event (may be log placeholder)")
    oprint("")

    # Warnings / heuristics
    oprint("Heuristics / Warnings")
    if task_rows:
        # Determine target seeds threshold
        max_seeds_out = int(max((r["seeds_out"] or 0) for r in task_rows)) if task_rows else 0
        target_seeds = args.num_seeds if args.num_seeds is not None else max_seeds_out

        low_matches = [r for r in task_rows if (r.get("matches_raw") or 0) < 100]
        zero_seeds = [r for r in task_rows if (r.get("seeds_out") or 0) == 0]
        over_lim = [
            r for r in task_rows
            if r.get("oversample") is not None
            and r.get("matches_in_bounds") is not None
            and r["oversample"] < r["matches_in_bounds"]
        ]
        oprint(f"  tasks_with_matches_raw<100: {len(low_matches)}")
        oprint(f"  tasks_with_seeds_out==0: {len(zero_seeds)}")
        oprint(f"  oversample_limit_hits: {len(over_lim)}")

        # Coverage check for target seeds
        enough_matches = [r for r in task_rows if (r.get("matches_in_bounds") or 0) >= target_seeds]
        enough_oversample = [r for r in task_rows if (r.get("oversample") or 0) >= target_seeds]
        enough_reproj = [r for r in task_rows if (r.get("reproj_pass") or 0) >= target_seeds]
        enough_parallax = [r for r in task_rows if (r.get("parallax_pass") or 0) >= target_seeds]
        enough_seeds = [r for r in task_rows if (r.get("seeds_out") or 0) >= target_seeds]
        oprint(f"  target_num_seeds: {target_seeds}")
        oprint(f"  tasks_with_matches_in_bounds>=target: {len(enough_matches)}")
        oprint(f"  tasks_with_oversample>=target: {len(enough_oversample)}")
        oprint(f"  tasks_with_reproj_pass>=target: {len(enough_reproj)}")
        oprint(f"  tasks_with_parallax_pass>=target: {len(enough_parallax)}")
        oprint(f"  tasks_with_seeds_out>=target: {len(enough_seeds)}")

        q_vals = [r.get("queue_size") for r in task_rows if r.get("queue_size") is not None]
        if q_vals and pct(q_vals, 0.9) > 0:
            oprint(f"  queue_backlog_p90: {fmt_num(pct(q_vals, 0.9))}")
    oprint("")

    # Save outputs
    if args.out:
        Path(args.out).write_text("\n".join(output_lines) + "\n")
    if args.out_json:
        summary = {
            "events": dict(events),
            "task_event_used": task_event,
            "rows": len(rows),
        }
        Path(args.out_json).write_text(json.dumps(summary, indent=2))
    if args.out_csv:
        out_path = Path(args.out_csv)
        out_fields = [
            "timestamp_ms",
            "ref_kfid",
            "neighbor_kfid",
            "matches_raw",
            "matches_in_bounds",
            "oversample",
            "triangulated",
            "cheirality_pass",
            "reproj_pass",
            "parallax_pass",
            "seeds_out",
            "median_reproj",
            "p90_reproj",
            "median_parallax",
            "p90_parallax",
            "task_ms",
            "queue_size",
            "iter",
            "ratio_in_bounds_raw",
            "ratio_cheirality_in_bounds",
            "ratio_reproj_cheirality",
            "ratio_parallax_reproj",
            "ratio_seeds_parallax",
            "limit_reason",
        ]
        num_seeds = args.num_seeds if args.num_seeds is not None else int(
            max((r["seeds_out"] or 0) for r in task_rows)
        ) if task_rows else 0

        def classify(r):
            seeds = r["seeds_out"] or 0
            m_in = r["matches_in_bounds"] or 0
            reproj = r["reproj_pass"] or 0
            par = r["parallax_pass"] or 0
            over = r["oversample"] or 0
            if seeds >= num_seeds:
                return "full"
            if m_in < num_seeds:
                return "limited_by_matches_in_bounds"
            if over < num_seeds:
                return "limited_by_oversample_cap"
            if reproj < num_seeds:
                return "limited_by_reproj"
            if par < num_seeds:
                return "limited_by_parallax"
            return "limited_by_other"

        with out_path.open("w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=out_fields)
            w.writeheader()
            for r in task_rows:
                row = {k: r.get(k) for k in out_fields}
                row["ratio_in_bounds_raw"] = safe_div(r.get("matches_in_bounds"), r.get("matches_raw"))
                row["ratio_cheirality_in_bounds"] = safe_div(r.get("cheirality_pass"), r.get("matches_in_bounds"))
                row["ratio_reproj_cheirality"] = safe_div(r.get("reproj_pass"), r.get("cheirality_pass"))
                row["ratio_parallax_reproj"] = safe_div(r.get("parallax_pass"), r.get("reproj_pass"))
                row["ratio_seeds_parallax"] = safe_div(r.get("seeds_out"), r.get("parallax_pass"))
                row["limit_reason"] = classify(r)
                w.writerow(row)


if __name__ == "__main__":
    main()
