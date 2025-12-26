use crate::models::{AnalysisReport, Violation};

pub fn format_text_report(report: &AnalysisReport) -> String {
    let mut out = String::new();
    out.push_str("════════════════════════════════════════════════════════════════\n");
    out.push_str("                    ROS Workspace Analysis Report               \n");
    out.push_str("════════════════════════════════════════════════════════════════\n\n");

    out.push_str("## Summary\n\n");
    out.push_str(&format!("  Total packages:   {}\n", report.summary.get("total_packages").cloned().unwrap_or(0)));
    out.push_str(&format!("  Total violations: {}\n\n", report.summary.get("total_violations").cloned().unwrap_or(0)));

    out.push_str("## Rule Violations\n\n");
    for v in &report.violations {
        out.push_str(&format_violation(v));
    }

    out
}

fn format_violation(v: &Violation) -> String {
    let mut s = String::new();
    s.push_str(&format!("  [{}] {}:{}\n", v.severity.to_uppercase(), v.file, v.line.map(|l| l.to_string()).unwrap_or_default()));
    s.push_str(&format!("    {}\n", v.message));
    if let Some(sug) = &v.suggestion {
        s.push_str(&format!("    Suggestion: {}\n\n", sug));
    }
    s
}
