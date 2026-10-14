"""Performance analysis: metrics recording, run summaries, CSV export, scenarios.
Nothing in this package imports pygame, so it also works in headless batch runs.
"""
from analysis.metrics import MetricsRecorder, RunSummary, Sample, CSV_COLUMNS, default_csv_path

__all__ = ["MetricsRecorder", "RunSummary", "Sample", "CSV_COLUMNS", "default_csv_path"]
