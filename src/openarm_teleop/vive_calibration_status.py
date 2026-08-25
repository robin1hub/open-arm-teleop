#!/usr/bin/env python3
"""Small status window for the VIVE anthropometric calibration."""

from __future__ import annotations

import json
import queue
import sys
import threading
import tkinter as tk


STEPS = (
    (
        "确定正前方向",
        "头显无需佩戴；把头显朝向人体正前方并保持可追踪，然后按一下任意手柄扳机。",
    ),
    ("双臂自然下垂", "双臂放松下垂、手腕自然，按一下扳机并松开。"),
    ("T-Pose 左右平举", "双臂水平向左右伸直，按一下扳机并松开。"),
    ("90° 弯肘向前", "上臂下垂、肘靠近身体、前臂水平向前，按一下扳机。"),
)


def reader(output: queue.Queue[dict[str, object]]) -> None:
    for line in sys.stdin:
        try:
            output.put(json.loads(line))
        except json.JSONDecodeError:
            continue


def main() -> int:
    root = tk.Tk()
    root.title("OpenArm VIVE 标定向导")
    root.geometry("620x500")
    root.minsize(560, 440)
    root.attributes("-topmost", True)
    root.configure(bg="#171a21")

    title = tk.Label(
        root,
        text="OpenArm 人体姿态标定",
        font=("Sans", 20, "bold"),
        fg="#f3f5f7",
        bg="#171a21",
    )
    title.pack(anchor="w", padx=28, pady=(24, 10))

    tracking = tk.Label(
        root,
        text="等待追踪数据…",
        font=("Sans", 12),
        fg="#f0b84b",
        bg="#171a21",
    )
    tracking.pack(anchor="w", padx=28, pady=(0, 18))

    step_labels: list[tk.Label] = []
    for number, (name, _instruction) in enumerate(STEPS, start=1):
        label = tk.Label(
            root,
            text=f"○  {number}. {name}",
            font=("Sans", 14),
            fg="#89919b",
            bg="#171a21",
            anchor="w",
        )
        label.pack(fill="x", padx=36, pady=5)
        step_labels.append(label)

    instruction = tk.Label(
        root,
        text=STEPS[0][1],
        font=("Sans", 13, "bold"),
        fg="#ffffff",
        bg="#26364a",
        justify="left",
        anchor="w",
        wraplength=540,
        padx=16,
        pady=14,
    )
    instruction.pack(fill="x", padx=28, pady=(22, 8))

    hint = tk.Label(
        root,
        text="每一步只按一下扳机，并在下一步前松开。键盘 C 可备用。",
        font=("Sans", 10),
        fg="#aeb5bd",
        bg="#171a21",
    )
    hint.pack(anchor="w", padx=28, pady=5)

    messages: queue.Queue[dict[str, object]] = queue.Queue()
    threading.Thread(target=reader, args=(messages,), daemon=True).start()

    def refresh() -> None:
        latest = None
        try:
            while True:
                latest = messages.get_nowait()
        except queue.Empty:
            pass
        if latest is not None:
            tracked = latest.get("tracking", {})
            states = [
                f"{label}: {'正常' if tracked.get(key) else '未追踪'}"
                for key, label in (
                    ("hmd", "头显"),
                    ("left", "左手柄"),
                    ("right", "右手柄"),
                )
            ]
            all_ok = all(tracked.get(key) for key in ("hmd", "left", "right"))
            tracking.configure(
                text="    ".join(states),
                fg="#55d68b" if all_ok else "#f0b84b",
            )
            completed = int(latest.get("completed", 0))
            total_steps = int(latest.get("total_steps", len(STEPS)))
            calibrated = bool(latest.get("calibrated", False))
            for index, label in enumerate(step_labels):
                if index >= total_steps:
                    label.pack_forget()
                    continue
                if not label.winfo_ismapped():
                    label.pack(fill="x", padx=36, pady=5)
                if index < completed:
                    label.configure(text=f"✓  {index + 1}. {STEPS[index][0]}", fg="#55d68b")
                elif index == completed and not calibrated:
                    label.configure(text=f"▶  {index + 1}. {STEPS[index][0]}", fg="#ffffff")
                else:
                    label.configure(text=f"○  {index + 1}. {STEPS[index][0]}", fg="#89919b")
            if calibrated:
                mode_text = (
                    "共享绝对空间"
                    if latest.get("mapping_mode") == "shared"
                    else "人体比例"
                )
                instruction.configure(
                    text=f"{mode_text}标定完成。两只手柄已振动，现在可以开始仿真测试。",
                    bg="#17633d",
                )
            else:
                current = min(completed, len(STEPS) - 1)
                instruction.configure(text=STEPS[current][1], bg="#26364a")
        root.after(80, refresh)

    root.after(80, refresh)
    root.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
