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
    let mut v = Vec::new();

    let make = |id: &str, target: &str, kind: &str, pattern: &str, message: &str, suggestion: &str, effort: f32| {
        Rule {
            id: id.to_string(),
            name: None,
            description: None,
            severity: Some("warning".to_string()),
            category: None,
            target: Some(target.to_string()),
            match_rule: crate::plugins::rule::RuleMatch { kind: kind.to_string(), pattern: pattern.to_string(), file_pattern: None },
            output: crate::plugins::rule::RuleOutput { message: message.to_string(), suggestion: Some(suggestion.to_string()), effort_hours: Some(effort) },
        }
    };

    v.push(make("ros1-header-ros", "cpp", "include", "ros/ros.h", "ros/ros.h → rclcpp/rclcpp.hpp", "#include <rclcpp/rclcpp.hpp>", 2.0));
    v.push(make("ros1-header-nodehandle", "cpp", "include", "ros/node_handle.h", "ros/node_handle.h → rclcpp/node.hpp", "#include <rclcpp/node.hpp>", 1.5));
    v.push(make("ros1-dep-roscpp", "package_xml", "dependency", "roscpp", "roscpp → rclcpp", "<depend>rclcpp</depend>", 1.0));
    v.push(make("ros1-dep-rospy", "package_xml", "dependency", "rospy", "rospy → rclpy", "<depend>rclpy</depend>", 0.5));

    v
}
