use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct Violation {
    pub rule_id: String,
    pub severity: String,
    pub file: String,
    pub line: Option<usize>,
    pub message: String,
    pub suggestion: Option<String>,
    pub effort_hours: Option<f32>,
}

#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct AnalysisReport {
    pub summary: std::collections::HashMap<String, usize>,
    pub packages: Vec<crate::models::package::Package>,
    pub violations: Vec<Violation>,
}
