mod models;
mod parsers;
mod scanner;
mod analyzer;
mod output;
mod plugins;
mod commands;

use clap::{Parser, Subcommand};

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
}

fn main() {
    let cli = Cli::parse();

    let result = match cli.command {
        Commands::Analyze(args) => commands::analyze::run(args),
        Commands::Report(args) => commands::report::run(args),
    };

    if let Err(e) = result {
        eprintln!("{}", e);
        std::process::exit(1);
    }
}

#[cfg(test)]
mod tests;
