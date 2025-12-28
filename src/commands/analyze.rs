use std::path::PathBuf;

use anyhow::Result;
use clap::Args;

use crate::{analyzer, output, plugins, scanner};

#[derive(Args, Debug, Clone)]
pub struct AnalyzeArgs {
    /// Workspace path to analyze
    pub workspace_path: Option<PathBuf>,

    /// Output format [text|json]
    #[arg(short, long, default_value = "json", value_parser = ["text", "json"])]
    pub format: String,

    /// Output file (default stdout)
    #[arg(short, long)]
    pub output: Option<PathBuf>,

    /// Rules path
    #[arg(short = 'r', long)]
    pub rules: Option<PathBuf>,

    /// Platform [ros1|ros2] (selects builtin rules)
    #[arg(short = 'p', long, value_parser = ["ros1", "ros2"])]
    pub platform: Option<String>,

    /// Disable builtin rules
    #[arg(long)]
    pub no_builtin: bool,

    /// List available rules and exit
    #[arg(long)]
    pub list_rules: bool,

    /// Verbose
    #[arg(short, long, action = clap::ArgAction::Count)]
    pub verbose: u8,
}

pub fn run(args: AnalyzeArgs) -> Result<()> {
    if !args.no_builtin && args.platform.is_none() {
        eprintln!(
            "warning: builtin rules are enabled but --platform is not set; no builtin rules will be loaded"
        );
        eprintln!("hint: pass -p ros1|ros2 (or use --no-builtin)");
    }

    // Load rules (also needed for --list-rules)
    let rules = match plugins::load_rules_from_path(
        args.rules.clone(),
        args.platform.clone(),
        !args.no_builtin,
    ) {
        Ok(r) => r,
        Err(e) => {
            eprintln!("warning: failed to load rules: {}", e);
            Vec::new()
        }
    };

    if args.verbose > 0 {
        eprintln!("Loaded {} rules", rules.len());
    }

    if args.list_rules {
        for r in &rules {
            println!("{} - {}", r.id, r.name.clone().unwrap_or_default());
        }
        return Ok(());
    }

    let ws = match args.workspace_path {
        Some(p) => p.to_string_lossy().to_string(),
        None => anyhow::bail!("workspace path is required"),
    };

    if args.verbose > 0 {
        eprintln!("Scanning workspace: {}", ws);
    }

    let scan = scanner::scan_workspace(&ws);
    let report = analyzer::analyze(&scan, &rules);

    if args.format == "json" {
        let s = output::format_json_report(&report)?;
        if let Some(p) = args.output {
            std::fs::write(p, s)?;
        } else {
            println!("{}", s);
        }
    } else {
        let s = output::format_text_report(&report);
        if let Some(p) = args.output {
            std::fs::write(p, s)?;
        } else {
            println!("{}", s);
        }
    }

    Ok(())
}
