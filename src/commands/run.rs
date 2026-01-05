use std::path::PathBuf;

use anyhow::Result;
use clap::Args;
use tempfile::tempdir;

use crate::commands;

#[derive(Args, Debug, Clone)]
pub struct RunArgs {
    /// Workspace path to analyze
    pub workspace_path: PathBuf,

    /// Output path.
    ///
    /// - If it ends with `.html`, writes a single HTML file.
    /// - Otherwise, writes a directory bundle: `index.html` + `assets/`.
    #[arg(short, long)]
    pub output: Option<PathBuf>,

    /// Report configuration TOML.
    ///
    /// Currently supports customizing sidebar section ordering/visibility.
    #[arg(long)]
    pub config: Option<PathBuf>,

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

pub fn run(args: RunArgs) -> Result<()> {
    let td = tempdir()?;
    let analysis_json = td.path().join("analysis.json");

    let analyze_args = commands::analyze::AnalyzeArgs {
        workspace_path: Some(args.workspace_path),
        format: "json".to_string(),
        output: Some(analysis_json.clone()),
        rules: args.rules,
        platform: args.platform,
        no_builtin: args.no_builtin,
        list_rules: args.list_rules,
        verbose: args.verbose,
    };

    commands::analyze::run(analyze_args)?;

    if args.list_rules {
        return Ok(());
    }

    let report_args = commands::report::ReportArgs {
        input: analysis_json,
        output: args.output,
        config: args.config,
    };

    commands::report::run(report_args)
}
