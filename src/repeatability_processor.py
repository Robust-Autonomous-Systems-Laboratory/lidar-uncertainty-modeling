import os
import glob
import re
import datetime
import io
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import yaml

from reportlab.lib.pagesizes import letter
from reportlab.lib import colors
from reportlab.lib.units import inch
from reportlab.lib.styles import getSampleStyleSheet, ParagraphStyle
from reportlab.platypus import (
    SimpleDocTemplate, Paragraph, Spacer, Table, TableStyle, Image, KeepTogether, HRFlowable
)
from reportlab.pdfgen import canvas


class NumberedCanvas(canvas.Canvas):
    """Handles running headers and two-pass 'Page X of Y' page numbering."""
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._saved_page_states = []

    def showPage(self):
        self._saved_page_states.append(dict(self.__dict__))
        self._startPage()

    def save(self):
        num_pages = len(self._saved_page_states)
        for state in self._saved_page_states:
            self.__dict__.update(state)
            self.draw_page_decorations(num_pages)
            super().showPage()
        super().save()

    def draw_page_decorations(self, page_count):
        self.saveState()
        self.setFont("Helvetica", 8)
        self.setFillColor(colors.HexColor("#4A5568"))

        if self._pageNumber > 1:
            self.drawString(54, 750, "LiDAR Experimental Repeatability & Uncertainty Report")
            self.setStrokeColor(colors.HexColor("#CBD5E1"))
            self.setLineWidth(0.5)
            self.line(54, 742, 558, 742)

        page_text = f"Page {self._pageNumber} of {page_count}"
        self.drawRightString(558, 36, page_text)
        self.drawString(54, 36, f"ISO/IEC GUM Standard Evaluation | Generated: {datetime.datetime.now().strftime('%Y-%m-%d')}")
        self.setStrokeColor(colors.HexColor("#CBD5E1"))
        self.setLineWidth(0.5)
        self.line(54, 48, 558, 48)
        self.restoreState()


from src.target_processing import beam_range_residuals


def refit_range_residuals(xyz: np.ndarray) -> np.ndarray:
    """Least-squares plane fit; returns each point's range offset to it along its own beam."""
    centroid = xyz.mean(axis=0)
    _, _, vh = np.linalg.svd(xyz - centroid, full_matrices=False)
    normal = vh[-1]
    return beam_range_residuals(xyz, centroid, normal)


# Chart styling: fixed categorical order (dataviz reference palette, light), recessive axes,
# text in ink colours rather than series colours.
SERIES_COLORS = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#008300", "#6250d6", "#e34948"]
SERIES_MARKERS = ["o", "s", "^", "D"]
SERIES_LINESTYLES = ["-", "--", ":", "-."]
INK_PRIMARY = "#0b0b0b"
INK_SECONDARY = "#52514e"
GRID_COLOR = "#e4e3df"
AXIS_COLOR = "#a8a7a0"


def natural_key(text) -> list:
    """Sort key so 'target2' comes before 'target10'."""
    return [int(part) if part.isdigit() else part.lower() for part in re.split(r"(\d+)", str(text))]


def natural_sort(df: pd.DataFrame, columns: list) -> pd.DataFrame:
    """Sort rows by the given columns using natural (number-aware) ordering."""
    order = sorted(range(len(df)), key=lambda i: [natural_key(df[c].iloc[i]) for c in columns])
    return df.iloc[order].reset_index(drop=True)


def parse_serial_number(target_cfg: dict) -> str:
    """Serial number from an explicit 'serial_number' key, else from 'SN ...' in other_info."""
    if target_cfg.get("serial_number") not in (None, ""):
        return str(target_cfg["serial_number"])
    match = re.search(r"\bS/?N[\s:#.]*([A-Za-z0-9-]+)", str(target_cfg.get("other_info", "")))
    return match.group(1) if match else ""


def load_target_metadata(experiment_file) -> dict:
    """{target name: {serial_number, nominal_range, reflectivity, size, other_info}} from the experiment YAML."""
    if not experiment_file or not os.path.exists(experiment_file):
        return {}
    with open(experiment_file, "r") as f:
        yaml_data = yaml.safe_load(f) or {}
    if not yaml_data:
        return {}
    _, targets = next(iter(yaml_data.items()))
    return {
        str(name): {
            "serial_number": parse_serial_number(cfg),
            "nominal_range": cfg.get("nominal_range"),
            "reflectivity": cfg.get("reflectivity"),
            "size": cfg.get("size"),
            "other_info": cfg.get("other_info", ""),
        }
        for name, cfg in (targets or {}).items()
    }


class LiDARRepeatabilityAnalyzer:
    def __init__(self, root_dir="results", range_column="calculated_range",
                 residual_column="range_residual", experiment_file=None, chart_lidars=None):
        """
        :param chart_lidars: lidar names to plot in the per-target charts, in legend order.
            None plots every lidar. Tables always include every lidar.
        """
        self.root_dir = root_dir
        self.range_column = range_column
        self.residual_column = residual_column
        self.experiment_file = experiment_file
        self.target_metadata = load_target_metadata(experiment_file)
        self.chart_lidars = list(chart_lidars) if chart_lidars else None
        self.results_df = pd.DataFrame()
        self.last_exported_csv = None

    def scan_and_process(self):
        """Scans repository CSVs and calculates Type A uncertainty stats per target."""
        records = []
        pattern = os.path.join(self.root_dir, "**", "*.csv")
        csv_files = glob.glob(pattern, recursive=True)

        for filepath in csv_files:
            norm_path = filepath.replace("\\", "/")
            parts = norm_path.split("/")
            
            test_id = parts[-2] if len(parts) >= 3 else "test_not_specified"
            lidar_id = parts[-3] if len(parts) >= 2 else "lidar_not_specified"
            filename = parts[-1]

            trial_match = re.search(r"target_points_(\d+)", filename)
            trial_id = f"{trial_match.group(1)}" if trial_match else filename.replace(".csv", "")

            try:
                data = pd.read_csv(filepath)
                if self.range_column not in data.columns:
                    continue
                
                data = data.dropna(subset=[self.range_column])

                # Repeatability is evaluated on each point's range offset from the fitted
                # target plane, measured along its own beam: target tilt / off-axis geometry
                # does not inflate s_r, and s_r stays in true range units.
                if self.residual_column in data.columns:
                    residuals = data[self.residual_column].to_numpy(dtype=float)
                    sr_basis = "irls_plane_beam"
                else:
                    # Legacy results saved before range residuals were exported: refit a
                    # least-squares plane to the saved (already IRLS-filtered) inlier points.
                    xyz = data[["x", "y", "z"]].to_numpy(dtype=float)
                    residuals = refit_range_residuals(xyz)
                    sr_basis = "refit_plane_beam"

                valid = ~np.isnan(residuals)
                data, residuals = data[valid], residuals[valid]
                N = len(data)
                if N < 2:
                    continue

                mean_r = data[self.range_column].astype(float).mean()
                range_spread = data[self.range_column].astype(float).std(ddof=1)
                std_sr = np.std(residuals, ddof=1)
                std_u = std_sr / np.sqrt(N)
                expanded_u95 = 2.0 * std_u

                records.append({
                    "test_id": test_id,
                    "lidar_id": lidar_id,
                    "trial_id": trial_id,
                    "file_path": norm_path,
                    "sample_count_N": N,
                    "mean_range_m": mean_r,
                    "repeatability_sr_m": std_sr,
                    "std_uncertainty_u_m": std_u,
                    "expanded_u95_m": expanded_u95,
                    "range_spread_m": range_spread,
                    "sr_basis": sr_basis,
                })
            except Exception as e:
                print(f"Error reading {filepath}: {e}")

        self.results_df = pd.DataFrame(records)
        return self.results_df

    def export_summary_csv(self, output_csv_path="results/lidar_repeatability_summary.csv"):
        """Exports repeatability summary to CSV and records the output path."""
        if self.results_df.empty:
            raise ValueError("No data to export. Call scan_and_process() first.")
        
        csv_dir = os.path.dirname(output_csv_path)
        if csv_dir:
            os.makedirs(csv_dir, exist_ok=True)

        self.results_df.to_csv(output_csv_path, index=False)
        self.last_exported_csv = os.path.abspath(output_csv_path)
        print(f"Exported CSV: {self.last_exported_csv}")
        return self.last_exported_csv

    def per_target_summary(self, df: pd.DataFrame) -> pd.DataFrame:
        """Aggregate trial results into one row per (lidar, target), with target metadata.

        pooled_s_r is the degree-of-freedom weighted within-trial repeatability;
        trial_to_trial_s is the standard deviation of the trial mean ranges.
        """
        rows = []
        for (lidar, target), group in df.groupby(["lidar_id", "test_id"]):
            dof = group["sample_count_N"] - 1
            total_dof = dof.sum()
            pooled = np.sqrt(np.sum(dof * group["repeatability_sr_m"] ** 2) / total_dof) if total_dof > 0 else np.nan
            meta = self.target_metadata.get(str(target), {})
            rows.append({
                "lidar_id": lidar,
                "target": target,
                "serial_number": meta.get("serial_number", ""),
                "nominal_range_m": meta.get("nominal_range"),
                "reflectivity": meta.get("reflectivity"),
                "trials": len(group),
                "total_points": int(group["sample_count_N"].sum()),
                "mean_range_m": group["mean_range_m"].mean(),
                "pooled_s_r_m": pooled,
                "trial_to_trial_s_m": group["mean_range_m"].std(ddof=1) if len(group) > 1 else np.nan,
                "mean_u_m": group["std_uncertainty_u_m"].mean(),
                "max_u95_m": group["expanded_u95_m"].max(),
            })
        summary = pd.DataFrame(rows)
        return summary if summary.empty else natural_sort(summary, ["target", "lidar_id"])

    def all_targets(self, df: pd.DataFrame) -> list:
        """Every target in the experiment YAML or the results, in natural order."""
        names = set(self.target_metadata) | set(df["test_id"].astype(str))
        return sorted(names, key=natural_key)

    def target_heading(self, target: str) -> str:
        """'target3 · SN 0121252684 — nominal 30 m, 80% reflectivity' (parts omitted when unknown)."""
        meta = self.target_metadata.get(target, {})
        heading = f"{target}  ·  SN {meta['serial_number']}" if meta.get("serial_number") else target
        details = []
        if meta.get("nominal_range") is not None:
            details.append(f"nominal {meta['nominal_range']} m")
        if meta.get("reflectivity") is not None:
            details.append(f"{meta['reflectivity'] * 100:.0f}% reflectivity")
        return heading + (f"  —  {', '.join(details)}" if details else "")

    def chart_lidar_selection(self, df: pd.DataFrame) -> tuple[list, str]:
        """Lidars to chart, plus a note for the report when the selection had to change."""
        available = sorted(df["lidar_id"].astype(str).unique(), key=natural_key)
        if not self.chart_lidars:
            return available, ""
        selected = [l for l in self.chart_lidars if l in available]
        missing = [l for l in self.chart_lidars if l not in available]
        if not selected:
            return available, ("None of the selected chart lidars (" + ", ".join(self.chart_lidars)
                               + ") are in these results, so all lidars are charted.")
        note = f"Charted lidars: {', '.join(selected)}."
        if missing:
            note += f" Not in these results: {', '.join(missing)}."
        return selected, note

    def _per_target_charts(self, df: pd.DataFrame) -> tuple[list, str]:
        """Render one chart per target: s_r (mm) for each trial, a series per charted lidar.

        :return: ([(PNG buffer or None, height in inches, target), ...] in natural target
                  order, note about the lidar selection). A None buffer means none of the
                  charted lidars have data for that target.
        """
        lidars, note = self.chart_lidar_selection(df)
        # Each lidar keeps one style in every chart. Colours follow the fixed categorical
        # order and every lidar also gets its own marker, so identity never rests on colour
        # alone; past eight lidars colours repeat with a new line style.
        styles = {
            lidar: {
                "color": SERIES_COLORS[i % len(SERIES_COLORS)],
                "marker": SERIES_MARKERS[i % len(SERIES_MARKERS)],
                "linestyle": SERIES_LINESTYLES[(i // len(SERIES_COLORS)) % len(SERIES_LINESTYLES)],
            }
            for i, lidar in enumerate(lidars)
        }

        charts = []
        chart_df = df[df["lidar_id"].astype(str).isin(lidars)]
        # One s_r scale for every target chart so charts can be compared page to page
        sr_max_mm = chart_df["repeatability_sr_m"].max() * 1000 if not chart_df.empty else 1.0
        y_top = float(np.ceil(sr_max_mm * 1.08)) if np.isfinite(sr_max_mm) and sr_max_mm > 0 else 1.0
        for target in self.all_targets(df):
            tdf = chart_df[chart_df["test_id"].astype(str) == target].copy()
            tdf["trial_num"] = pd.to_numeric(tdf["trial_id"], errors="coerce")
            present = [l for l in lidars if l in set(tdf["lidar_id"].astype(str))]
            if not present:
                charts.append((None, 0, target))
                continue

            # Grow the figure with the legend so long lidar lists still fit
            height_in = max(2.5, 0.6 + 0.19 * len(present))
            fig, ax = plt.subplots(figsize=(6.5, height_in), dpi=200)
            fig.patch.set_facecolor("white")

            # x is the distance offset from the target's nominal range, in mm (same units as
            # s_r); without a nominal range, offsets are taken from the mean of all trials.
            meta = self.target_metadata.get(target, {})
            if meta.get("nominal_range") is not None:
                reference_m = float(meta["nominal_range"])
                x_label = "Mean distance − nominal (mm)"
            else:
                reference_m = float(tdf["mean_range_m"].mean())
                x_label = "Mean distance − mean of all trials (mm)"

            # Each trial is a small faint dot at (its mean-distance offset, its s_r). Each lidar
            # also gets one bold summary marker at (average offset, pooled s_r) with a
            # horizontal bar spanning ± pooled s_r; that marker carries the legend.
            x_extent = 1.0
            for lidar in present:
                ldf = tdf[tdf["lidar_id"].astype(str) == lidar].sort_values("trial_num")
                st = styles[lidar]
                offsets_mm = (ldf["mean_range_m"] - reference_m) * 1000
                ax.plot(offsets_mm, ldf["repeatability_sr_m"] * 1000, linestyle="none",
                        marker=st["marker"], markersize=3.5, color=st["color"], alpha=0.35,
                        markeredgewidth=0, zorder=2)

                dof = ldf["sample_count_N"] - 1
                pooled_mm = (np.sqrt(np.sum(dof * ldf["repeatability_sr_m"] ** 2) / dof.sum()) * 1000
                             if dof.sum() > 0 else float(ldf["repeatability_sr_m"].mean() * 1000))
                centre_mm = float(offsets_mm.mean())
                container = ax.errorbar(
                    [centre_mm], [pooled_mm], xerr=[pooled_mm],
                    fmt=st["marker"], color=st["color"], markersize=8.5,
                    markeredgecolor="white", markeredgewidth=1.2,
                    ecolor=st["color"], elinewidth=2.0, capsize=4, capthick=2.0,
                    label=f"{lidar}  (n={len(ldf)})", zorder=4,
                )
                for bar in container[2]:
                    bar.set_linestyle(st["linestyle"])
                x_extent = max(x_extent, float(offsets_mm.abs().max()), abs(centre_mm) + pooled_mm)

            # Symmetric x-axis keeps the nominal distance (0) in the centre of every chart
            x_lim = x_extent * 1.12
            ax.set_xlim(-x_lim, x_lim)
            ax.axvline(0, color=AXIS_COLOR, linewidth=0.8, linestyle=(0, (2, 2)), zorder=1)

            serial = meta.get("serial_number") or "no serial number"
            details = []
            if meta.get("nominal_range") is not None:
                details.append(f"nominal {meta['nominal_range']} m")
            if meta.get("reflectivity") is not None:
                details.append(f"{meta['reflectivity'] * 100:.0f}% reflectivity")
            ax.set_title(f"{target}  ·  SN {serial}" if meta.get("serial_number") else f"{target}  ·  {serial}",
                         loc="left", fontsize=10, fontweight="bold", color=INK_PRIMARY, pad=14)
            if details:
                ax.text(0, 1.015, ", ".join(details), transform=ax.transAxes,
                        fontsize=7.5, color=INK_SECONDARY, va="bottom")

            ax.set_xlabel(x_label, fontsize=8, color=INK_SECONDARY)
            ax.set_ylabel("Repeatability $s_r$ (mm)", fontsize=8, color=INK_SECONDARY)
            ax.xaxis.set_major_locator(plt.MaxNLocator(nbins=9, symmetric=True))
            ax.set_ylim(0, y_top)
            ax.tick_params(labelsize=7, colors=INK_SECONDARY, length=0)
            ax.grid(True, axis="y", color=GRID_COLOR, linewidth=0.6)
            ax.set_axisbelow(True)
            for side in ("top", "right", "left"):
                ax.spines[side].set_visible(False)
            ax.spines["bottom"].set_color(AXIS_COLOR)

            legend = ax.legend(title="LiDAR", loc="upper left", bbox_to_anchor=(1.01, 1.0),
                               frameon=False, fontsize=7, title_fontsize=7.5, handlelength=2.6)
            for text in legend.get_texts():
                text.set_color(INK_PRIMARY)
            legend.get_title().set_color(INK_SECONDARY)

            fig.tight_layout()
            buffer = io.BytesIO()
            fig.savefig(buffer, format="png", dpi=200, facecolor="white")
            plt.close(fig)
            buffer.seek(0)
            charts.append((buffer, height_in, target))
        return charts, note

    def export_pdf_report(self, csv_path=None, pdf_filename=None):
        """
        Converts the summary CSV file to a formatted PDF report and saves it
        in the exact same directory as the CSV file.
        """
        # Determine CSV source path
        if csv_path is None:
            if self.last_exported_csv and os.path.exists(self.last_exported_csv):
                csv_path = self.last_exported_csv
            else:
                raise ValueError("No CSV path specified and no recent export found.")

        csv_path = os.path.abspath(csv_path)
        if not os.path.exists(csv_path):
            raise FileNotFoundError(f"CSV file not found: {csv_path}")

        # Extract target directory from CSV path
        csv_dir = os.path.dirname(csv_path)
        if pdf_filename is None:
            base_name = os.path.splitext(os.path.basename(csv_path))[0]
            pdf_filename = f"{base_name}.pdf"

        output_pdf_path = os.path.join(csv_dir, pdf_filename)

        # Load summary CSV data
        df = pd.read_csv(csv_path)
        if df.empty:
            raise ValueError("Provided CSV file is empty.")

        # Compute degree-of-freedom weighted pooled statistics
        pooled_records = []
        for lidar, group in df.groupby("lidar_id"):
            df_series = group["sample_count_N"] - 1
            total_df = df_series.sum()
            s_pooled = np.sqrt(np.sum(df_series * (group["repeatability_sr_m"] ** 2)) / total_df) if total_df > 0 else 0.0
            pooled_records.append({
                "lidar_id": lidar,
                "target_count": len(group),
                "total_points": group["sample_count_N"].sum(),
                "avg_range_m": group["mean_range_m"].mean(),
                "pooled_s_r_mm": s_pooled * 1000,
                "max_u95_mm": group["expanded_u95_m"].max() * 1000
            })
        pooled_df = pd.DataFrame(pooled_records)

        # One repeatability chart per target (Matplotlib buffers)
        target_charts, chart_note = self._per_target_charts(df)

        # Build PDF via ReportLab
        doc = SimpleDocTemplate(
            output_pdf_path,
            pagesize=letter,
            leftMargin=54, rightMargin=54, topMargin=54, bottomMargin=54
        )

        styles = getSampleStyleSheet()
        title_style = ParagraphStyle("DocTitle", parent=styles["Heading1"], fontName="Helvetica-Bold", fontSize=18, leading=22, textColor=colors.HexColor("#1A365D"))
        subtitle_style = ParagraphStyle("DocSub", parent=styles["Normal"], fontName="Helvetica", fontSize=9, leading=12, textColor=colors.HexColor("#4A5568"))
        section_style = ParagraphStyle("SecTitle", parent=styles["Heading2"], fontName="Helvetica-Bold", fontSize=11, leading=14, textColor=colors.HexColor("#1A365D"), spaceBefore=10, spaceAfter=4)

        story = []
        story.append(Paragraph("LiDAR Measurement Repeatability Report", title_style))
        story.append(Paragraph(f"Source Summary File: <i>{os.path.basename(csv_path)}</i>", subtitle_style))
        story.append(Spacer(1, 4))
        story.append(HRFlowable(width="100%", thickness=1.5, color=colors.HexColor("#0D9488"), spaceAfter=10))
        story.append(Paragraph(
            "Repeatability s<sub>r</sub> is the sample standard deviation of each point's range offset "
            "from the robustly fitted target plane, measured along that point's own beam. This removes "
            "geometric range variation across the target face (e.g. targets not square-on to the sensor) "
            "while keeping s<sub>r</sub> in true range units. Mean range is the mean of calculated point "
            "ranges.", subtitle_style))
        story.append(Spacer(1, 6))

        # 1. Per-target charts: s_r for every trial, one colour/marker per lidar
        story.append(Paragraph("Repeatability by Target", section_style))
        story.append(Paragraph(
            "One chart per target. Faint dots are individual trials, plotted at the trial's mean distance "
            "offset from the target's nominal range (mm) against its repeatability s<sub>r</sub>. The bold "
            "marker for each lidar sits at its average offset and pooled s<sub>r</sub>, with a horizontal "
            "bar spanning ± pooled s<sub>r</sub>; n is the number of trials. All charts share one "
            "s<sub>r</sub> scale. Each lidar keeps the same colour and marker in every chart. "
            + chart_note, subtitle_style))
        story.append(Spacer(1, 4))
        note_style = ParagraphStyle("Note", parent=subtitle_style, fontName="Helvetica-Oblique",
                                    textColor=colors.HexColor("#9A3412"))
        for img_buffer, height_in, target in target_charts:
            if img_buffer is None:
                story.append(Paragraph(f"<b>{self.target_heading(target)}</b>", subtitle_style))
                story.append(Paragraph("No points or statistics from the charted lidars for this target.", note_style))
                story.append(Spacer(1, 8))
                continue
            story.append(KeepTogether([Image(img_buffer, width=6.5*inch, height=height_in*inch), Spacer(1, 6)]))
        story.append(Spacer(1, 4))

        # 2. Pooled Metrics Summary Table
        story.append(Paragraph("Pooled Sensor Summary", section_style))
        pooled_headers = ["Sensor ID", "Trials", "Total Points", "Avg Distance (m)", "Pooled s_r (mm)", "Max U_95 (mm)"]
        pooled_rows = [pooled_headers]
        for _, r in pooled_df.iterrows():
            pooled_rows.append([
                str(r["lidar_id"]), str(r["target_count"]), f"{int(r['total_points']):,}",
                f"{r['avg_range_m']:.3f}", f"{r['pooled_s_r_mm']:.3f}", f"{r['max_u95_mm']:.3f}"
            ])

        t_pooled = Table(pooled_rows, colWidths=[80, 60, 90, 90, 94, 90])
        t_pooled.setStyle(TableStyle([
            ('BACKGROUND', (0,0), (-1,0), colors.HexColor("#1A365D")),
            ('TEXTCOLOR', (0,0), (-1,0), colors.white),
            ('FONTNAME', (0,0), (-1,0), 'Helvetica-Bold'),
            ('FONTSIZE', (0,0), (-1,-1), 8.5),
            ('ALIGN', (0,0), (-1,-1), 'CENTER'),
            ('ROWBACKGROUNDS', (0,1), (-1,-1), [colors.white, colors.HexColor("#F8FAFC")]),
            ('GRID', (0,0), (-1,-1), 0.5, colors.HexColor("#CBD5E1")),
            ('TOPPADDING', (0,0), (-1,-1), 4),
            ('BOTTOMPADDING', (0,0), (-1,-1), 4),
        ]))
        story.append(t_pooled)
        story.append(Spacer(1, 8))

        # 3. Per-target statistics for each lidar, labelled with target name and serial number
        target_df = self.per_target_summary(df)
        target_csv_path = os.path.join(csv_dir, f"{os.path.splitext(os.path.basename(csv_path))[0]}_per_target.csv")
        target_df.to_csv(target_csv_path, index=False)
        print(f"Exported per-target CSV: {target_csv_path}")

        def fmt(value, scale=1.0, digits=3):
            return "—" if value is None or pd.isna(value) else f"{value * scale:.{digits}f}"

        header_style = [
            ('TEXTCOLOR', (0,0), (-1,0), colors.white),
            ('FONTNAME', (0,0), (-1,0), 'Helvetica-Bold'),
            ('ALIGN', (0,0), (-1,-1), 'CENTER'),
            ('VALIGN', (0,0), (-1,-1), 'MIDDLE'),
            ('GRID', (0,0), (-1,-1), 0.5, colors.HexColor("#CBD5E1")),
            ('TOPPADDING', (0,0), (-1,-1), 3),
            ('BOTTOMPADDING', (0,0), (-1,-1), 3),
        ]
        cell_style = ParagraphStyle("Cell", parent=styles["Normal"], fontName="Helvetica-Bold",
                                    fontSize=7.5, leading=9, alignment=1, textColor=colors.white)

        if self.target_metadata:
            story.append(Paragraph("Targets", section_style))
            story.append(Paragraph(
                f"Target names and serial numbers from <i>{os.path.basename(self.experiment_file)}</i>.",
                subtitle_style))
            story.append(Spacer(1, 4))
            meta_rows = [["Target", "Serial No.", "Nominal Range (m)", "Reflectivity", "Size (m)", "Description"]]
            for name in sorted(self.target_metadata, key=natural_key):
                m = self.target_metadata[name]
                size = m.get("size")
                meta_rows.append([
                    name, m["serial_number"] or "—",
                    "—" if m["nominal_range"] is None else str(m["nominal_range"]),
                    "—" if m["reflectivity"] is None else f"{m['reflectivity'] * 100:.0f}%",
                    " × ".join(str(s) for s in size) if isinstance(size, (list, tuple)) else "—",
                    Paragraph(str(m["other_info"] or ""), ParagraphStyle(
                        "Desc", parent=styles["Normal"], fontName="Helvetica", fontSize=7.5, leading=9)),
                ])
            t_meta = Table(meta_rows, colWidths=[52, 70, 80, 64, 64, 174], repeatRows=1)
            t_meta.setStyle(TableStyle(header_style + [
                ('BACKGROUND', (0,0), (-1,0), colors.HexColor("#1A365D")),
                ('FONTSIZE', (0,0), (-1,-1), 7.5),
                ('ROWBACKGROUNDS', (0,1), (-1,-1), [colors.white, colors.HexColor("#F8FAFC")]),
            ]))
            story.append(t_meta)
            story.append(Spacer(1, 8))

        story.append(Paragraph("Statistics by Target", section_style))
        story.append(Paragraph(
            "One table per target with a row for each lidar, combining all of that lidar's trials. "
            "Pooled s<sub>r</sub> is the degree-of-freedom weighted within-trial repeatability; "
            "trial-to-trial s is the standard deviation of the trial mean ranges (— with fewer than two "
            "trials). Max U<sub>95</sub> is the largest single-trial expanded uncertainty.", subtitle_style))

        target_head_style = ParagraphStyle(
            "TargetHead", parent=styles["Normal"], fontName="Helvetica-Bold", fontSize=9.5,
            leading=12, textColor=colors.HexColor("#0F766E"), spaceBefore=10, spaceAfter=3)
        all_lidars = sorted(df["lidar_id"].astype(str).unique(), key=natural_key)
        per_target_headers = ["LiDAR", "Trials", "Points", "Mean Range (m)", "Pooled s_r (mm)",
                              "Trial-to-trial s (mm)", "Max U_95 (mm)"]
        for target in self.all_targets(df):
            group = target_df[target_df["target"].astype(str) == target]
            block = [Paragraph(self.target_heading(target), target_head_style)]
            if group.empty:
                block.append(Paragraph("No points or statistics for this target.", note_style))
                story.append(KeepTogether(block))
                continue

            rows = [[Paragraph(h, cell_style) for h in per_target_headers]]
            for _, r in group.iterrows():
                rows.append([
                    str(r["lidar_id"]), str(r["trials"]), f"{r['total_points']:,}",
                    fmt(r["mean_range_m"], digits=4), fmt(r["pooled_s_r_m"], 1000),
                    fmt(r["trial_to_trial_s_m"], 1000), fmt(r["max_u95_m"], 1000),
                ])
            t_target = Table(rows, colWidths=[110, 46, 62, 72, 72, 76, 66], repeatRows=1)
            t_target.setStyle(TableStyle(header_style + [
                ('BACKGROUND', (0,0), (-1,0), colors.HexColor("#0F766E")),
                ('FONTSIZE', (0,1), (-1,-1), 7.5),
                ('ROWBACKGROUNDS', (0,1), (-1,-1), [colors.white, colors.HexColor("#F0FDFA")]),
            ]))
            block.append(t_target)

            missing = [l for l in all_lidars if l not in set(group["lidar_id"].astype(str))]
            if missing:
                block.append(Spacer(1, 2))
                block.append(Paragraph(f"No points or statistics for this target from: {', '.join(missing)}.",
                                       note_style))
            story.append(KeepTogether(block))

        # 4. Per-trial detail (appendix)
        story.append(Paragraph("Appendix: Per-Trial Uncertainty Breakdown", section_style))
        detail_headers = ["Target", "LiDAR", "Trial", "N", "Mean Range (m)", "s_r (mm)", "u (mm)", "U_95 (mm)"]
        detail_rows = [detail_headers]

        for _, row in natural_sort(df, ["lidar_id", "test_id", "trial_id"]).iterrows():
            detail_rows.append([
                str(row["test_id"]), str(row["lidar_id"]), str(row["trial_id"]), str(int(row["sample_count_N"])),
                f"{row['mean_range_m']:.4f}", f"{row['repeatability_sr_m']*1000:.3f}",
                f"{row['std_uncertainty_u_m']*1000:.3f}", f"{row['expanded_u95_m']*1000:.3f}"
            ])

        t_detail = Table(detail_rows, colWidths=[50, 110, 40, 35, 80, 60, 60, 65], repeatRows=1)
        t_detail.setStyle(TableStyle([
            ('BACKGROUND', (0,0), (-1,0), colors.HexColor("#2B6CB0")),
            ('TEXTCOLOR', (0,0), (-1,0), colors.white),
            ('FONTNAME', (0,0), (-1,0), 'Helvetica-Bold'),
            ('FONTSIZE', (0,0), (-1,-1), 8),
            ('ALIGN', (0,0), (-1,-1), 'CENTER'),
            ('ROWBACKGROUNDS', (0,1), (-1,-1), [colors.white, colors.HexColor("#F7FAFC")]),
            ('GRID', (0,0), (-1,-1), 0.5, colors.HexColor("#E2E8F0")),
            ('TOPPADDING', (0,0), (-1,-1), 3.5),
            ('BOTTOMPADDING', (0,0), (-1,-1), 3.5),
        ]))
        story.append(t_detail)

        doc.build(story, canvasmaker=NumberedCanvas)
        print(f"Exported PDF report: {output_pdf_path}")
        return output_pdf_path