mod models;
mod parsers;
mod scanner;
mod analyzer;
mod output;
mod plugins;
mod commands;

use clap::{Parser, Subcommand};
use std::path::PathBuf;

#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Args {
    #[command(subcommand)]
    command: Option<Commands>,

    /// Workspace path to analyze (omit when using subcommands such as init)
    workspace_path: Option<PathBuf>,

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

#[derive(Subcommand, Debug)]
enum Commands {
    /// Initialize chelonian files in a workspace
    Init {
        /// Path to initialize (default: current directory)
        #[arg(short, long)]
        path: Option<PathBuf>,

        /// Force overwrite existing files
        #[arg(short, long)]
        force: bool,

        /// Do not copy default rules
        #[arg(long)]
        no_rules: bool,
        /// Platform to initialize rules for [ros1|ros2]
        #[arg(short = 'p', long, value_parser = ["ros1", "ros2"])]
        platform: Option<String>,
        /// Interactive selection if platform not provided
        #[arg(long)]
        interactive: bool,
    },
}

fn main() {
    let args = Args::parse();

    // handle subcommands
    if let Some(cmd) = args.command {
        match cmd {
            Commands::Init { path, force, no_rules, platform, interactive } => {
                let p = path.unwrap_or_else(|| std::env::current_dir().expect("cwd"));
                if let Err(e) = commands::init::run_init(&p, force, no_rules, platform, interactive) {
                    eprintln!("init failed: {}", e);
                    std::process::exit(1);
                }
                return;
            }
        }
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
