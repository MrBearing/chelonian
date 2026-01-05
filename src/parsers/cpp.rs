use crate::models::cpp::{CppAnalysis, Include};
use anyhow::Result;
use regex::Regex;
use std::fs;

pub fn parse_cpp_file(path: &str) -> Result<CppAnalysis> {
    let text = fs::read_to_string(path)?;
    let mut analysis = CppAnalysis {
        includes: Vec::new(),
        full_text: Some(text.clone()),
    };

    let re = Regex::new(r#"^\s*#\s*include\s*[<"]([^>"]+)[>"]"#).unwrap();
    for (i, line) in text.lines().enumerate() {
        if let Some(cap) = re.captures(line) {
            if let Some(m) = cap.get(1) {
                analysis.includes.push(Include {
                    path: m.as_str().to_string(),
                    line: i + 1,
                });
            }
        }
    }

    Ok(analysis)
}
