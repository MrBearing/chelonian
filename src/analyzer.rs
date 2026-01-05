use crate::models::{AnalysisReport, Finding};
use crate::plugins::engine;
use crate::plugins::rule::Rule;
use crate::scanner::ScanResult;
use std::collections::HashMap;

pub fn analyze(scan: &ScanResult, rules: &[Rule]) -> AnalysisReport {
    let mut findings: Vec<Finding> = Vec::new();

    // Apply plugin rules
    let mut from_rules = engine::apply_rules(scan, rules);
    findings.append(&mut from_rules);

    // summary counts
    let mut summary: HashMap<String, usize> = HashMap::new();
    summary.insert("total_packages".to_string(), scan.packages.len());
    summary.insert("total_findings".to_string(), findings.len());

    AnalysisReport {
        summary,
        packages: scan.packages.clone(),
        findings,
    }
}
