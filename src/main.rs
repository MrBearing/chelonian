mod analyzer;
mod commands;
mod models;
mod output;
mod parsers;
mod plugins;
mod scanner;

use clap::{Parser, Subcommand};
use std::ffi::OsString;

#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Cli {
    #[command(subcommand)]
    command: Commands,
}

#[derive(Subcommand, Debug)]
enum Commands {
    /// Analyze a ROS workspace
    Analyze(commands::analyze::AnalyzeArgs),

    /// Convert analysis JSON into a static HTML report
    Report(commands::report::ReportArgs),

    /// Analyze a workspace and generate a report in one step
    Run(commands::run::RunArgs),
}

fn should_inject_run_shorthand(args: &[OsString]) -> bool {
    // args[0] is the binary name.
    if args.len() < 2 {
        return false;
    }

    let first = &args[1];

    // If it looks like an option, let clap handle it normally.
    if first.to_string_lossy().starts_with('-') {
        return false;
    }

    // Don't inject when the user already used a known command.
    let s = first.to_string_lossy();
    !matches!(s.as_ref(), "analyze" | "report" | "run" | "help")
}

fn parse_cli_with_run_shorthand() -> Cli {
    let args: Vec<OsString> = std::env::args_os().collect();
    if should_inject_run_shorthand(&args) {
        let mut injected: Vec<OsString> = Vec::with_capacity(args.len() + 1);
        injected.push(args[0].clone());
        injected.push(OsString::from("run"));
        injected.extend_from_slice(&args[1..]);
        Cli::parse_from(injected)
    } else {
        Cli::parse_from(args)
    }
}

fn main() {
    let cli = parse_cli_with_run_shorthand();

    let result = match cli.command {
        Commands::Analyze(args) => commands::analyze::run(args),
        Commands::Report(args) => commands::report::run(args),
        Commands::Run(args) => commands::run::run(args),
    };

    if let Err(e) = result {
        eprintln!("{}", e);
        std::process::exit(1);
    }
}

#[cfg(test)]
mod tests;
