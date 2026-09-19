import os
import glob
import re
import datetime
import io
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

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


class LiDARRepeatabilityAnalyzer:
    def __init__(self, root_dir="results", range_column="calculated_range"):
        self.root_dir = root_dir
        self.range_column = range_column
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
                
                series = data[self.range_column].dropna().astype(float)
                N = len(series)
                if N < 2:
                    continue

                mean_r = series.mean()
                std_sr = series.std(ddof=1)
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
                    "expanded_u95_m": expanded_u95
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

        # Generate Repeatability Trend Chart (Matplotlib Buffer)
        fig, ax = plt.subplots(figsize=(6.5, 3.0), dpi=300)
        for lidar, group in df.groupby("lidar_id"):
            group_sorted = group.sort_values("mean_range_m")
            ax.errorbar(
                group_sorted["mean_range_m"],
                group_sorted["repeatability_sr_m"] * 1000,
                yerr=group_sorted["std_uncertainty_u_m"] * 1000,
                fmt="-o", linewidth=1.8, markersize=5, capsize=3, label=f"{lidar}"
            )
        ax.set_title("Lidar Repeatability ($s_r$) vs Target Range", fontsize=11, fontweight="bold", pad=8)
        ax.set_xlabel("Target Distance (m)", fontsize=9)
        ax.set_ylabel("Standard Deviation $s_r$ (mm)", fontsize=9)
        ax.legend(title="Sensor Unit", loc="upper left", frameon=True, fontsize=8)
        ax.grid(True, linestyle="--", alpha=0.5)
        plt.tight_layout()

        img_buffer = io.BytesIO()
        plt.savefig(img_buffer, format="png", dpi=300)
        plt.close(fig)
        img_buffer.seek(0)

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

        # 1. Visualization Chart
        story.append(Paragraph("Repeatability Precision vs Range", section_style))
        story.append(Image(img_buffer, width=6.5*inch, height=3.0*inch))
        story.append(Spacer(1, 8))

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

        # 3. Target Level Details Table
        story.append(Paragraph("Target Uncertainty Breakdown", section_style))
        detail_headers = ["Target", "LiDAR", "Trial", "N", "Mean Range (m)", "s_r (mm)", "u (mm)", "U_95 (mm)"]
        detail_rows = [detail_headers]

        for _, row in df.sort_values(by=["lidar_id", "test_id", "mean_range_m"]).iterrows():
            detail_rows.append([
                str(row["test_id"]), str(row["lidar_id"]), str(row["trial_id"]), str(int(row["sample_count_N"])),
                f"{row['mean_range_m']:.4f}", f"{row['repeatability_sr_m']*1000:.3f}",
                f"{row['std_uncertainty_u_m']*1000:.3f}", f"{row['expanded_u95_m']*1000:.3f}"
            ])

        t_detail = Table(detail_rows, colWidths=[50, 110, 40, 35, 80, 60, 60, 65])
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