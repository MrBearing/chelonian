use anyhow::{Context, Result};
use std::fs;
use std::path::{Path, PathBuf};
use crate::plugins::rule::{Rule, RulesFile};

fn load_file(p: &Path) -> Result<Vec<Rule>> {
    let txt = fs::read_to_string(p).with_context(|| format!("reading rule file {}", p.display()))?;
    let rf: RulesFile = toml::from_str(&txt).with_context(|| format!("parsing toml {}", p.display()))?;
    Ok(rf.rules.unwrap_or_default())
}

pub fn load_rules_from_path(path_opt: Option<PathBuf>, platform_opt: Option<String>, include_builtin: bool) -> Result<Vec<Rule>> {
    let mut rules: Vec<Rule> = Vec::new();

    // 1. built-in (loaded first, user rules add on top)
    if include_builtin {
        rules.append(&mut built_in_rules(platform_opt.as_deref())?);
    }

    // 2. user-specified
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

    // 3. config dir
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

    Ok(rules)
}

fn built_in_rules(platform: Option<&str>) -> Result<Vec<Rule>> {
    let mut rules: Vec<Rule> = Vec::new();
    
    // Load platform-specific builtin rules from builtin-rules/<platform>/
    if let Some(p) = platform {
        let builtin_dir = PathBuf::from(format!("builtin-rules/{}", p));
        if builtin_dir.exists() {
            for entry in fs::read_dir(&builtin_dir)? {
                let e = entry?;
                let path = e.path();
                if path.extension().and_then(|s| s.to_str()) == Some("toml") {
                    if let Ok(mut rr) = load_file(&path) {
                        rules.append(&mut rr);
                    }
                }
            }
        }
    } else {
        // No platform specified -> do not load any builtin rules.
    }
    
    // Backwards-compatibility: some tests / consumers expect legacy ids.
    // Ensure a legacy id `ros1-dep-roscpp` exists if a roscpp dependency rule is present.
    let mut extra: Vec<Rule> = Vec::new();
    for r in &rules {
        if r.id == "roscpp-dep" {
            let mut nr = r.clone();
            nr.id = "ros1-dep-roscpp".to_string();
            extra.push(nr);
        }
    }
    rules.append(&mut extra);
    Ok(rules)
}
