use anyhow::{Context, Result};
use std::fs;
use std::path::{Path, PathBuf};
use crate::plugins::rule::{Rule, RulesFile};

fn load_file(p: &Path) -> Result<Vec<Rule>> {
    let txt = fs::read_to_string(p).with_context(|| format!("reading rule file {}", p.display()))?;
    let rf: RulesFile = toml::from_str(&txt).with_context(|| format!("parsing toml {}", p.display()))?;
    Ok(rf.rules.unwrap_or_default())
}

pub fn load_rules_from_path(path_opt: Option<PathBuf>, include_builtin: bool) -> Result<Vec<Rule>> {
    let mut rules: Vec<Rule> = Vec::new();

    // 1. user-specified
    if let Some(p) = path_opt {
        let p = p.as_path();
        if p.is_file() {
            rules.append(&mut load_file(p)?);
        } else if p.is_dir() {
            for entry in fs::read_dir(p)? {
                let e = entry?;
                let path = e.path();
                if path.extension().and_then(|s| s.to_str()) == Some("toml") {
                    rules.append(&mut load_file(&path)?);
                }
            }
        }
    }

    // 2. config dir
    if let Some(mut dir) = dirs_next::config_dir() {
        dir.push("chelonian");
        dir.push("rules");
        if dir.exists() {
            for entry in fs::read_dir(dir)? {
                let e = entry?;
                let path = e.path();
                if path.extension().and_then(|s| s.to_str()) == Some("toml") {
                    rules.append(&mut load_file(&path)?);
                }
            }
        }
    }

    // 3. built-in
    if include_builtin {
        rules.append(&mut built_in_rules());
    }

    Ok(rules)
}

fn built_in_rules() -> Vec<Rule> {
    // Per "TOML-only" policy, keep built-in rules minimal and rely on TOML rule files
    // shipped in `examples/rules/` and copied by `chel init`.
    // Returning empty vector here makes the runtime rely on TOML inputs only.
    Vec::new()
}
