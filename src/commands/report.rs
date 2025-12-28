use std::fs;
use std::path::PathBuf;

use anyhow::{Context, Result};
use clap::Args;

use crate::models::AnalysisReport;

#[derive(Args, Debug, Clone)]
pub struct ReportArgs {
    /// Input analysis JSON file (produced by `chel analyze --format json`)
    pub input: PathBuf,

    /// Output HTML file (default stdout)
    #[arg(short, long)]
    pub output: Option<PathBuf>,
}

pub fn run(args: ReportArgs) -> Result<()> {
    let bytes = fs::read(&args.input)
        .with_context(|| format!("failed to read input: {}", args.input.display()))?;

    let report: AnalysisReport = serde_json::from_slice(&bytes)
        .with_context(|| "failed to parse analysis JSON".to_string())?;

    let html = render_html(&report);

    if let Some(out) = args.output {
        fs::write(&out, html).with_context(|| format!("failed to write output: {}", out.display()))?;
    } else {
        print!("{}", html);
    }

    Ok(())
}

fn render_html(report: &AnalysisReport) -> String {
    let total_packages = report.summary.get("total_packages").copied().unwrap_or(0);
    let total_findings = report.summary.get("total_findings").copied().unwrap_or(report.findings.len());

    let mut out = String::new();
    out.push_str("<!doctype html>\n<html lang=\"en\">\n<head>\n");
    out.push_str("<meta charset=\"utf-8\">\n<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">\n");
    out.push_str("<title>Chelonian Report</title>\n");
    out.push_str("</head>\n<body>\n");

    out.push_str("<h1>Chelonian Report</h1>\n");
    out.push_str("<ul>\n");
    out.push_str(&format!("<li>Total packages: {}</li>\n", total_packages));
    out.push_str(&format!("<li>Total findings: {}</li>\n", total_findings));
    out.push_str("</ul>\n");

    out.push_str("<h2>Findings</h2>\n");
    out.push_str("<table border=\"1\" cellspacing=\"0\" cellpadding=\"6\">\n");
    out.push_str("<thead><tr><th>Severity</th><th>Rule</th><th>File</th><th>Line</th><th>Message</th><th>Suggestion</th></tr></thead>\n");
    out.push_str("<tbody>\n");

    for f in &report.findings {
        out.push_str("<tr>");
        out.push_str(&format!("<td>{}</td>", escape_html(&f.severity)));
        out.push_str(&format!("<td>{}</td>", escape_html(&f.rule_id)));
        out.push_str(&format!("<td>{}</td>", escape_html(&f.file)));
        out.push_str(&format!(
            "<td>{}</td>",
            f.line.map(|v| v.to_string()).unwrap_or_default()
        ));
        out.push_str(&format!("<td>{}</td>", escape_html(&f.message)));
        out.push_str(&format!(
            "<td>{}</td>",
            f.suggestion
                .as_deref()
                .map(escape_html)
                .unwrap_or_default()
        ));
        out.push_str("</tr>\n");
    }

    out.push_str("</tbody></table>\n");
    out.push_str("</body>\n</html>\n");
    out
}

fn escape_html(s: &str) -> String {
    let mut out = String::with_capacity(s.len());
    for c in s.chars() {
        match c {
            '&' => out.push_str("&amp;"),
            '<' => out.push_str("&lt;"),
            '>' => out.push_str("&gt;"),
            '"' => out.push_str("&quot;"),
            '\'' => out.push_str("&#39;"),
            _ => out.push(c),
        }
    }
    out
}
