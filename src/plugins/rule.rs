use serde::Deserialize;

#[allow(dead_code)]
#[derive(Debug, Clone, Deserialize)]
pub struct RuleMatch {
	#[serde(rename = "type")]
	pub kind: String,
	pub pattern: String,
	pub file_pattern: Option<String>,
}

#[derive(Debug, Clone, Deserialize)]
pub struct RuleOutput {
	pub message: String,
	pub suggestion: Option<String>,
	pub effort_hours: Option<f32>,
}

#[allow(dead_code)]
#[derive(Debug, Clone, Deserialize)]
pub struct Rule {
	pub id: String,
	pub name: Option<String>,
	pub description: Option<String>,
	pub severity: Option<String>,
	pub category: Option<String>,
	pub target: Option<String>,
	#[serde(rename = "match")]
	pub match_rule: RuleMatch,
	#[serde(rename = "output")]
	pub output: RuleOutput,
}

#[allow(dead_code)]
#[derive(Debug, Clone, Deserialize)]
pub struct RulesFile {
	pub meta: Option<Meta>,
	pub rules: Option<Vec<Rule>>,
}

#[allow(dead_code)]
#[derive(Debug, Clone, Deserialize)]
pub struct Meta {
	pub name: Option<String>,
	pub version: Option<String>,
	pub description: Option<String>,
}
