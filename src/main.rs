mod models;
mod parsers;
mod scanner;
mod analyzer;
mod output;
mod plugins;

use clap::Parser;
use std::path::PathBuf;

#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Args {
    /// Workspace path to analyze
    workspace_path: Option<PathBuf>,

    /// Output format [text|json]
    #[arg(short, long, default_value = "json")]
    format: String,

    /// Output file (default stdout)
    #[arg(short, long)]
    output: Option<PathBuf>,

    /// Rules path
    #[arg(short = 'r', long)]
    rules: Option<PathBuf>,

    /// Platform [ros1|ros2] (selects builtin rules)
    #[arg(short = 'p', long, value_parser = ["ros1", "ros2"])]
    platform: Option<String>,

    /// Disable builtin rules
    #[arg(long)]
    no_builtin: bool,

    /// List available rules and exit
    #[arg(long)]
    list_rules: bool,

    /// Verbose
    #[arg(short, long, action = clap::ArgAction::Count)]
    verbose: u8,
}

fn main() {
    let args = Args::parse();

    if !args.no_builtin && args.platform.is_none() {
        eprintln!("warning: builtin rules are enabled but --platform is not set; no builtin rules will be loaded");
        eprintln!("hint: pass -p ros1|ros2 (or use --no-builtin)");
    }

    // load rules (needed for --list-rules as well)
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
        return;
    }

    let ws = match args.workspace_path {
        Some(p) => p.to_string_lossy().to_string(),
        None => {
            eprintln!("workspace path is required");
            std::process::exit(2);
        }
    };

    if args.verbose > 0 {
        eprintln!("Scanning workspace: {}", ws);
    }

    let scan = scanner::scan_workspace(&ws);
    let report = analyzer::analyze(&scan, &rules);

    if args.format == "json" {
        match output::format_json_report(&report) {
            Ok(s) => {
                if let Some(p) = args.output { std::fs::write(p, s).expect("failed write"); }
                else { println!("{}", s); }
            }
            Err(e) => eprintln!("failed to serialize json: {}", e),
        }
    } else {
        let s = output::format_text_report(&report);
        if let Some(p) = args.output { std::fs::write(p, s).expect("failed write"); }
        else { println!("{}", s); }
    }
}

#[cfg(test)]
mod tests;
