"""PNG output (matplotlib, no display)."""

from __future__ import annotations

from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from .analyze import Step  # noqa: E402


def histograms(series: dict[str, np.ndarray], path: Path, unit: str = "us", title: str = "") -> None:
    """One log-scale histogram per series (values already in `unit`)."""
    names = [n for n, v in series.items() if len(v)]
    if not names:
        return
    cols = 2
    rows = (len(names) + cols - 1) // cols
    fig, axes = plt.subplots(rows, cols, figsize=(11, 2.8 * rows), squeeze=False)
    for ax, name in zip(axes.flat, names):
        v = np.asarray(series[name], float)
        ax.hist(v, bins=200, log=True, color="C0")
        ax.set_title(f"{name} (n={v.size}, max={v.max():.0f} {unit})", fontsize=9)
        ax.set_xlabel(unit)
    for ax in list(axes.flat)[len(names):]:
        ax.axis("off")
    if title:
        fig.suptitle(title)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


def compare_histograms(a: dict[str, np.ndarray], b: dict[str, np.ndarray], labels: tuple[str, str],
                       path: Path, unit: str = "us") -> None:
    names = [n for n in a if len(a.get(n, [])) and len(b.get(n, []))]
    if not names:
        return
    cols = 2
    rows = (len(names) + cols - 1) // cols
    fig, axes = plt.subplots(rows, cols, figsize=(11, 2.8 * rows), squeeze=False)
    for ax, name in zip(axes.flat, names):
        va, vb = np.asarray(a[name], float), np.asarray(b[name], float)
        hi = max(va.max(), vb.max())
        bins = np.linspace(0, hi * 1.02 if hi > 0 else 1, 200)
        ax.hist(va, bins=bins, log=True, histtype="step", label=labels[0])
        ax.hist(vb, bins=bins, log=True, histtype="step", label=labels[1])
        ax.set_title(name, fontsize=9)
        ax.set_xlabel(unit)
        ax.legend(fontsize=8)
    for ax in list(axes.flat)[len(names):]:
        ax.axis("off")
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


def _step(ax, sig: Step, y: float, label: str, color: str) -> None:
    t = np.append(sig.times, sig.t_end)
    v = np.append(sig.values, sig.values[-1]).astype(float)
    ax.step(t, y + 0.8 * v, where="post", color=color, lw=1)
    ax.text(-0.01, y + 0.4, label, transform=ax.get_yaxis_transform(), ha="right", va="center", fontsize=8)


def timing_diagram(path: Path, title: str, rows: list[tuple[str, Step]], commands: list[tuple[float, str]],
                   pulses: dict[str, np.ndarray] | None = None,
                   position: tuple[np.ndarray, np.ndarray] | None = None) -> None:
    """Step signals stacked, OSC commands as vertical lines, pulse trains as ticks,
    and the polled position below."""
    n_extra = 1 if position is not None and len(position[0]) else 0
    fig, axes = plt.subplots(1 + n_extra, 1, figsize=(13, 4.5 + 2 * n_extra), sharex=True, squeeze=False)
    ax = axes[0, 0]
    y = 0.0
    for i, (label, sig) in enumerate(rows):
        _step(ax, sig, y, label, f"C{i}")
        y += 1.2
    for label, t in (pulses or {}).items():
        ax.vlines(t, y, y + 0.8, color="k", lw=0.5)
        ax.text(-0.01, y + 0.4, label, transform=ax.get_yaxis_transform(), ha="right", va="center", fontsize=8)
        y += 1.2
    for t, text in commands:
        ax.axvline(t, color="gray", lw=0.5, ls="--")
        ax.text(t, y + 0.1, text, rotation=60, fontsize=7, va="bottom")
    ax.set_yticks([])
    ax.set_ylim(-0.3, y + 2.5)
    ax.set_title(title, fontsize=10)
    if n_extra:
        pax = axes[1, 0]
        pax.plot(position[0], position[1], ".-", ms=2, lw=0.5)
        pax.set_ylabel("position")
    axes[-1, 0].set_xlabel("time from trigger [s]")
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


def waveform(path: Path, title: str, rate: float, volts: np.ndarray, spans: list[tuple[str, float, float]],
             snippet: float = 2e-3) -> None:
    """A short snippet of the waveform at the start of each named span."""
    fig, axes = plt.subplots(len(spans), 1, figsize=(10, 2.2 * len(spans)), squeeze=False)
    for ax, (name, t0, _) in zip(axes[:, 0], spans):
        i0, i1 = int(t0 * rate), int((t0 + snippet) * rate)
        seg = volts[i0:i1]
        ax.plot(np.arange(seg.size) / rate * 1e3, seg, lw=0.6)
        ax.set_title(name, fontsize=9)
        ax.set_ylabel("V")
    axes[-1, 0].set_xlabel("ms")
    fig.suptitle(title)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)
