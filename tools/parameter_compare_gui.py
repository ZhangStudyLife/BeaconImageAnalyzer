from __future__ import annotations

import os
import queue
import re
import subprocess
import sys
import threading
from datetime import datetime
from pathlib import Path
import tkinter as tk
from tkinter import filedialog, messagebox, ttk


ROOT = Path(__file__).resolve().parents[1]
REPORT_SCRIPT = ROOT / "build-codex-check/parameter_effect_report.py"
DEFAULT_VIDEO_DIR = Path(r"E:/Desktop/前后摄45度结构")
DEFAULT_OUTPUT_DIR = DEFAULT_VIDEO_DIR / "参数对比输出"
VIDEO_TYPES = [
    ("视频文件", "*.avi *.mp4 *.mov *.mkv"),
    ("AVI", "*.avi"),
    ("所有文件", "*.*"),
]


class ParameterCompareWindow(tk.Tk):
    def __init__(self) -> None:
        super().__init__()
        self.title("信标算法参数对比")
        self.geometry("900x650")
        self.minsize(760, 560)

        self.process: subprocess.Popen[str] | None = None
        self.messages: queue.Queue[tuple[str, object]] = queue.Queue()
        self.result_path: Path | None = None

        self.video_vars = {
            "beacon": tk.StringVar(value=str(DEFAULT_VIDEO_DIR / "有阳光开窗户2026_07_14_06_34_57_Video.avi")),
            "car": tk.StringVar(value=str(DEFAULT_VIDEO_DIR / "前摄_车灯.avi")),
            "flight": tk.StringVar(value=str(DEFAULT_VIDEO_DIR / "前摄像头_实际飞.avi")),
        }
        self.frame_limit_var = tk.IntVar(value=8000)
        self.output_dir_var = tk.StringVar(value=str(DEFAULT_OUTPUT_DIR))
        self.prefix_var = tk.StringVar(value=datetime.now().strftime("compare-%Y%m%d-%H%M%S"))
        self.status_var = tk.StringVar(value="就绪")

        self._build_ui()
        self.after(100, self._drain_messages)
        self.protocol("WM_DELETE_WINDOW", self._close_window)

    def _build_ui(self) -> None:
        root = ttk.Frame(self, padding=16)
        root.pack(fill=tk.BOTH, expand=True)
        root.columnconfigure(1, weight=1)
        root.rowconfigure(8, weight=1)

        rows = [
            ("信标/环境光视频", "beacon"),
            ("车灯参数视频", "car"),
            ("实际飞行/时序视频", "flight"),
        ]
        for row, (label, key) in enumerate(rows):
            ttk.Label(root, text=label).grid(row=row, column=0, sticky="w", padx=(0, 10), pady=6)
            ttk.Entry(root, textvariable=self.video_vars[key]).grid(row=row, column=1, sticky="ew", pady=6)
            ttk.Button(root, text="选择", command=lambda item=key: self._select_video(item)).grid(
                row=row, column=2, padx=(8, 0), pady=6
            )

        ttk.Button(root, text="同一视频用于全部参数", command=self._select_same_video).grid(
            row=3, column=1, sticky="w", pady=(4, 12)
        )

        ttk.Label(root, text="最多处理帧数").grid(row=4, column=0, sticky="w", padx=(0, 10), pady=6)
        ttk.Spinbox(root, from_=100, to=100000, increment=500, textvariable=self.frame_limit_var, width=14).grid(
            row=4, column=1, sticky="w", pady=6
        )

        ttk.Label(root, text="输出目录").grid(row=5, column=0, sticky="w", padx=(0, 10), pady=6)
        ttk.Entry(root, textvariable=self.output_dir_var).grid(row=5, column=1, sticky="ew", pady=6)
        ttk.Button(root, text="选择", command=self._select_output_dir).grid(row=5, column=2, padx=(8, 0), pady=6)

        ttk.Label(root, text="输出名称").grid(row=6, column=0, sticky="w", padx=(0, 10), pady=6)
        ttk.Entry(root, textvariable=self.prefix_var).grid(row=6, column=1, sticky="ew", pady=6)

        actions = ttk.Frame(root)
        actions.grid(row=7, column=0, columnspan=3, sticky="ew", pady=(12, 10))
        actions.columnconfigure(3, weight=1)
        self.start_button = ttk.Button(actions, text="开始生成", command=self._start_generation)
        self.start_button.grid(row=0, column=0, padx=(0, 8))
        self.open_button = ttk.Button(actions, text="打开结果", command=self._open_result, state=tk.DISABLED)
        self.open_button.grid(row=0, column=1, padx=(0, 12))
        self.progress = ttk.Progressbar(actions, mode="determinate", maximum=37)
        self.progress.grid(row=0, column=2, columnspan=2, sticky="ew")

        log_frame = ttk.Frame(root)
        log_frame.grid(row=8, column=0, columnspan=3, sticky="nsew")
        log_frame.columnconfigure(0, weight=1)
        log_frame.rowconfigure(0, weight=1)
        self.log = tk.Text(log_frame, wrap="word", state=tk.DISABLED, font=("Consolas", 10))
        scrollbar = ttk.Scrollbar(log_frame, orient=tk.VERTICAL, command=self.log.yview)
        self.log.configure(yscrollcommand=scrollbar.set)
        self.log.grid(row=0, column=0, sticky="nsew")
        scrollbar.grid(row=0, column=1, sticky="ns")

        ttk.Label(root, textvariable=self.status_var).grid(row=9, column=0, columnspan=3, sticky="w", pady=(8, 0))

    def _select_video(self, key: str) -> None:
        initial = Path(self.video_vars[key].get()).parent
        path = filedialog.askopenfilename(
            title="选择视频",
            initialdir=str(initial if initial.is_dir() else DEFAULT_VIDEO_DIR),
            filetypes=VIDEO_TYPES,
        )
        if path:
            self.video_vars[key].set(path)

    def _select_same_video(self) -> None:
        path = filedialog.askopenfilename(title="选择视频", initialdir=str(DEFAULT_VIDEO_DIR), filetypes=VIDEO_TYPES)
        if path:
            for variable in self.video_vars.values():
                variable.set(path)

    def _select_output_dir(self) -> None:
        path = filedialog.askdirectory(title="选择输出目录", initialdir=self.output_dir_var.get())
        if path:
            self.output_dir_var.set(path)

    def _validate(self) -> tuple[dict[str, Path], Path, int, str] | None:
        videos = {key: Path(variable.get().strip()) for key, variable in self.video_vars.items()}
        missing = [str(path) for path in videos.values() if not path.is_file()]
        if missing:
            messagebox.showerror("视频不存在", "\n".join(missing))
            return None
        try:
            frame_limit = int(self.frame_limit_var.get())
        except (TypeError, ValueError):
            frame_limit = 0
        if frame_limit <= 0:
            messagebox.showerror("帧数无效", "最多处理帧数必须大于 0。")
            return None
        output_dir = Path(self.output_dir_var.get().strip())
        prefix = re.sub(r"[^A-Za-z0-9._-]+", "-", self.prefix_var.get()).strip("-._")
        if not prefix:
            messagebox.showerror("名称无效", "输出名称至少需要一个 ASCII 字母或数字。")
            return None
        return videos, output_dir, frame_limit, prefix

    def _start_generation(self) -> None:
        if self.process is not None:
            return
        validated = self._validate()
        if validated is None:
            return
        videos, output_dir, frame_limit, prefix = validated
        output_dir.mkdir(parents=True, exist_ok=True)
        self.result_path = None
        self.open_button.configure(state=tk.DISABLED)
        self.start_button.configure(state=tk.DISABLED)
        self.progress.configure(value=0)
        self._set_log("")
        self.status_var.set("正在加载视频并编译参数变体")

        command = [
            sys.executable,
            str(REPORT_SCRIPT),
            "--beacon-video",
            str(videos["beacon"]),
            "--car-video",
            str(videos["car"]),
            "--flight-video",
            str(videos["flight"]),
            "--frame-limit",
            str(frame_limit),
            "--output-dir",
            str(output_dir),
            "--output-prefix",
            prefix,
            "--standalone",
        ]
        thread = threading.Thread(target=self._run_process, args=(command,), daemon=True)
        thread.start()

    def _run_process(self, command: list[str]) -> None:
        try:
            creation_flags = subprocess.CREATE_NO_WINDOW if os.name == "nt" else 0
            environment = os.environ.copy()
            environment["PYTHONIOENCODING"] = "utf-8"
            self.process = subprocess.Popen(
                command,
                cwd=str(ROOT),
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                encoding="utf-8",
                errors="replace",
                creationflags=creation_flags,
                env=environment,
            )
            assert self.process.stdout is not None
            for line in self.process.stdout:
                self.messages.put(("log", line))
                progress_match = re.search(r"\[(\d+)/37\]", line)
                if progress_match:
                    self.messages.put(("progress", int(progress_match.group(1))))
                if line.startswith("standalone="):
                    self.messages.put(("result", line.split("=", 1)[1].strip()))
            return_code = self.process.wait()
            self.messages.put(("done", return_code))
        except Exception as error:
            self.messages.put(("error", str(error)))
        finally:
            self.process = None

    def _drain_messages(self) -> None:
        try:
            while True:
                kind, payload = self.messages.get_nowait()
                if kind == "log":
                    self._append_log(str(payload))
                elif kind == "progress":
                    value = int(payload)
                    self.progress.configure(value=value)
                    self.status_var.set(f"正在处理参数 {value}/37")
                elif kind == "result":
                    self.result_path = Path(str(payload))
                elif kind == "done":
                    self.start_button.configure(state=tk.NORMAL)
                    if int(payload) == 0 and self.result_path is not None:
                        self.progress.configure(value=37)
                        self.open_button.configure(state=tk.NORMAL)
                        self.status_var.set("生成完成")
                        self._open_result()
                    else:
                        self.status_var.set(f"生成失败，退出码 {payload}")
                        messagebox.showerror("生成失败", "请查看日志中的错误信息。")
                elif kind == "error":
                    self.start_button.configure(state=tk.NORMAL)
                    self.status_var.set("生成失败")
                    messagebox.showerror("生成失败", str(payload))
        except queue.Empty:
            pass
        self.after(100, self._drain_messages)

    def _set_log(self, value: str) -> None:
        self.log.configure(state=tk.NORMAL)
        self.log.delete("1.0", tk.END)
        self.log.insert(tk.END, value)
        self.log.configure(state=tk.DISABLED)

    def _append_log(self, value: str) -> None:
        self.log.configure(state=tk.NORMAL)
        self.log.insert(tk.END, value)
        self.log.see(tk.END)
        self.log.configure(state=tk.DISABLED)

    def _open_result(self) -> None:
        if self.result_path is not None and self.result_path.is_file():
            os.startfile(self.result_path)

    def _close_window(self) -> None:
        if self.process is not None:
            if not messagebox.askyesno("任务运行中", "参数对比仍在生成，确定要终止吗？"):
                return
            self.process.terminate()
        self.destroy()


if __name__ == "__main__":
    ParameterCompareWindow().mainloop()
