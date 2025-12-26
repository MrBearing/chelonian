use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct Include {
    pub path: String,
    pub line: usize,
}

#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct CppAnalysis {
    pub includes: Vec<Include>,
    pub full_text: Option<String>,
}
