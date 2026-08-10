import csv
from pathlib import Path
import statistics


def summarize_first_column_stdlib(directory_path: str):
    path = Path(directory_path)
    csv_files = list(path.glob("*.csv"))

    if not csv_files:
        print(f"No CSV files found in directory: {directory_path}")
        return

    numeric_values = []
    text_values = []

    for file_path in csv_files:
        try:
            with open(
                file_path, mode="r", encoding="utf-8-sig", errors="ignore"
            ) as f:
                reader = csv.reader(f)
                for row in reader:
                    if row:  # Ensure row is not empty
                        first_item = row[0].strip()
                        try:
                            numeric_values.append(float(first_item) / 1000.0)  # adjust to remove / 1000 in normal conditions!
                        except ValueError:
                            # Keep track of text/non-numeric items
                            if first_item:
                                text_values.append(first_item)
        except Exception as e:
            print(f"Error reading '{file_path.name}': {e}")

    print("=" * 45)
    print(f"SUMMARY STATISTICS ({len(csv_files)} files analyzed)")
    print("=" * 45)

    if numeric_values:
        print(f"Total Numeric Items: {len(numeric_values)}")
        print(f"Mean:                {statistics.mean(numeric_values):.4f}")
        print(f"Median:              {statistics.median(numeric_values):.4f}")
        print(f"Min:                 {min(numeric_values):.4f}")
        print(f"Max:                 {max(numeric_values):.4f}")

        if len(numeric_values) > 1:
            print(
                f"Std Deviation:       {statistics.stdev(numeric_values):.4f}"
            )
    elif text_values:
        print(f"Total Non-Numeric Items: {len(text_values)}")
        print(f"Unique Values:           {len(set(text_values))}")
    else:
        print("No valid items found.")


if __name__ == "__main__":
    TARGET_DIRECTORY = "./results/june_26_2026_laser_rangefinder/ouster/points/target1"
    summarize_first_column_stdlib(TARGET_DIRECTORY)