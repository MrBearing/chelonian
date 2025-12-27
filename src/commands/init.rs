use anyhow::{Context, Result};
use std::fs;
use std::io::{self, Write};
use std::path::{Path, PathBuf};

pub fn run_init(path: &Path, force: bool, no_rules: bool, platform: Option<String>, interactive: bool) -> Result<()> {
    let base = PathBuf::from(path);
    let dot = base.join(".chelonian");

    if dot.exists() && !force {
        anyhow::bail!("{}.chelonian already exists; use --force to overwrite", base.display());
    }

    // create directories
    fs::create_dir_all(dot.join("rules")).with_context(|| "creating rules dir")?;
    fs::create_dir_all(dot.join("output")).with_context(|| "creating output dir")?;

    // write config.toml
    let cfg = r#"# chelonian configuration
rules = ["./.chelonian/rules"]
output = "./.chelonian/output"
"#;
    fs::write(dot.join("config.toml"), cfg).with_context(|| "writing config.toml")?;

    if !no_rules {
        // determine platform folder: prefer explicit platform, else interactive choice, else default "ros1"
        let chosen = if let Some(pf) = platform {
            pf
        } else if interactive {
            // prompt user
            println!("Select target platform for rules:\n  1) ros1\n  2) ros2\nEnter choice (1/2): ");
            print!("> "); io::stdout().flush().ok();
            let mut input = String::new();
            io::stdin().read_line(&mut input).ok();
            match input.trim() {
                "2" | "ros2" => "ros2".to_string(),
                _ => "ros1".to_string(),
            }
        } else {
            "ros1".to_string()
        };

        // copy default rules from examples/rules/<chosen> if present
        let repo_examples = PathBuf::from("examples/rules").join(&chosen);
        if repo_examples.exists() {
            for entry in fs::read_dir(&repo_examples).with_context(|| format!("reading examples/rules/{}", chosen))? {
                let e = entry?;
                let p = e.path();
                if p.extension().and_then(|s| s.to_str()) == Some("toml") {
                    let fname = p.file_name().unwrap();
                    fs::copy(&p, dot.join("rules").join(fname)).with_context(|| format!("copying {:?}", p))?;
                }
            }
        } else {
            // fallback: try examples/rules root
            let repo_examples_root = PathBuf::from("examples/rules");
            if repo_examples_root.exists() {
                for entry in fs::read_dir(&repo_examples_root).with_context(|| "reading examples/rules")? {
                    let e = entry?;
                    let p = e.path();
                    if p.extension().and_then(|s| s.to_str()) == Some("toml") {
                        let fname = p.file_name().unwrap();
                        fs::copy(&p, dot.join("rules").join(fname)).with_context(|| format!("copying {:?}", p))?;
                    }
                }
            }
        }
    }

    println!("Initialized chelonian in {}", base.display());
    println!("Created: {}", dot.display());
    Ok(())
}
