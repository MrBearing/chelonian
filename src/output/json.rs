use crate::models::AnalysisReport;
use serde_json::Result;

pub fn format_json_report(report: &AnalysisReport) -> Result<String> {
    serde_json::to_string_pretty(report)
}
