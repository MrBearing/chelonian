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
    workspace_path: PathBuf,

    /// Output format [text|json]
    #[arg(short, long, default_value = "text")]
    format: String,

    /// Output file (default stdout)
    #[arg(short, long)]
    output: Option<PathBuf>,

    /// Rules path
    #[arg(short = 'r', long)]
    rules: Option<PathBuf>,

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

    let ws = args.workspace_path.to_string_lossy().to_string();
    if args.verbose > 0 {
        eprintln!("Scanning workspace: {}", ws);
    }

    // load rules
    let rules = match plugins::load_rules_from_path(args.rules.clone(), !args.no_builtin) {
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
