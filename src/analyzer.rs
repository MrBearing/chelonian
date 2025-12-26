use crate::scanner::ScanResult;
use crate::models::{AnalysisReport, Violation};
use crate::plugins::rule::Rule;
use crate::plugins::engine;
use std::collections::HashMap;

pub fn analyze(scan: &ScanResult, rules: &[Rule]) -> AnalysisReport {
    let mut violations: Vec<Violation> = Vec::new();

    // Apply plugin rules
    let mut from_rules = engine::apply_rules(scan, rules);
    violations.append(&mut from_rules);

    // summary counts
    let mut summary: HashMap<String, usize> = HashMap::new();
    summary.insert("total_packages".to_string(), scan.packages.len());
    summary.insert("total_violations".to_string(), violations.len());

    AnalysisReport {
        summary,
        packages: scan.packages.clone(),
        violations,
    }
}
