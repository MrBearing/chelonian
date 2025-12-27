use crate::plugins::rule::Rule;
use crate::scanner::ScanResult;
use crate::models::Finding;
use regex::Regex;
use glob::Pattern;

pub fn apply_rules(scan: &ScanResult, rules: &[Rule]) -> Vec<Finding> {
    let mut findings: Vec<Finding> = Vec::new();

    for rule in rules {
        let target = rule.target.as_deref().unwrap_or("cpp");
        match target {
            "cpp" => {
                for (file, cpp) in &scan.cpp_map {
                    if let Some(fp) = &rule.match_rule.file_pattern {
                        if let Ok(pat) = Pattern::new(fp) {
                            if !pat.matches(file) { continue; }
                        }
                    }

                    match rule.match_rule.kind.as_str() {
                        "include" => {
                            for inc in &cpp.includes {
                                if inc.path == rule.match_rule.pattern || inc.path.ends_with(&rule.match_rule.pattern) || inc.path.contains(&rule.match_rule.pattern) {
                                    findings.push(Finding {
                                        rule_id: rule.id.clone(),
                                        severity: rule.severity.clone().unwrap_or_else(|| "warning".to_string()),
                                        file: file.clone(),
                                        line: Some(inc.line),
                                        message: rule.output.message.clone(),
                                        suggestion: rule.output.suggestion.clone(),
                                        effort_hours: rule.output.effort_hours,
                                    });
                                }
                            }
                        }
                        "regex" => {
                            // Use stored full_text from scanner to avoid re-reading files
                            if let Some(text) = &cpp.full_text {
                                if let Ok(re) = Regex::new(&rule.match_rule.pattern) {
                                    if re.is_match(text) {
                                        findings.push(Finding {
                                            rule_id: rule.id.clone(),
                                            severity: rule.severity.clone().unwrap_or_else(|| "warning".to_string()),
                                            file: file.clone(),
                                            line: None,
                                            message: rule.output.message.clone(),
                                            suggestion: rule.output.suggestion.clone(),
                                            effort_hours: rule.output.effort_hours,
                                        });
                                    }
                                }
                            }
                        }
                        _ => {}
                    }
                }
            }
            "package_xml" => {
                for pkg in &scan.packages {
                    match rule.match_rule.kind.as_str() {
                        "dependency" => {
                            for d in &pkg.dependencies {
                                if d.name == rule.match_rule.pattern || d.name.contains(&rule.match_rule.pattern) {
                                    findings.push(Finding {
                                        rule_id: rule.id.clone(),
                                        severity: rule.severity.clone().unwrap_or_else(|| "warning".to_string()),
                                        file: format!("{}/package.xml", pkg.path),
                                        line: None,
                                        message: rule.output.message.clone(),
                                        suggestion: rule.output.suggestion.clone(),
                                        effort_hours: rule.output.effort_hours,
                                    });
                                }
                            }
                        }
                        "build_type" => {
                            if let Some(bt) = &pkg.build_type {
                                if bt == &rule.match_rule.pattern {
                                    findings.push(Finding {
                                        rule_id: rule.id.clone(),
                                        severity: rule.severity.clone().unwrap_or_else(|| "warning".to_string()),
                                        file: format!("{}/package.xml", pkg.path),
                                        line: None,
                                        message: rule.output.message.clone(),
                                        suggestion: rule.output.suggestion.clone(),
                                        effort_hours: rule.output.effort_hours,
                                    });
                                }
                            }
                        }
                        _ => {}
                    }
                }
            }
            _ => {}
        }
    }

    findings
}

#[allow(dead_code)]
fn cpp_text_for(_file: &str) -> Option<String> {
    // For now the scanner doesn't store file text; could be extended.
    None
}
