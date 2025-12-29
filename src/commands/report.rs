use std::collections::{BTreeMap, BTreeSet};
use std::fs;
use std::path::{Path, PathBuf};

use anyhow::{Context, Result};
use clap::Args;
use serde::{Deserialize, Serialize};

use crate::models::AnalysisReport;

#[derive(Args, Debug, Clone)]
pub struct ReportArgs {
    /// Input analysis JSON file (produced by `chel analyze --format json`)
    pub input: PathBuf,

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
}

pub fn run(args: ReportArgs) -> Result<()> {
    let bytes = fs::read(&args.input)
        .with_context(|| format!("failed to read input: {}", args.input.display()))?;

    let report: AnalysisReport = serde_json::from_slice(&bytes)
        .with_context(|| "failed to parse analysis JSON".to_string())?;

    match args.output {
        None => {
            // legacy: write a single HTML to stdout
            let html = render_html(&report);
            print!("{}", html);
        }
        Some(out) => {
            if looks_like_html_file(&out) {
                let html = render_html(&report);
                fs::write(&out, html)
                    .with_context(|| format!("failed to write output: {}", out.display()))?;
            } else {
                let cfg = load_report_config(args.config.as_deref())?;
                write_results_bundle(&out, &report, &cfg)?;
            }
        }
    }

    Ok(())
}

fn looks_like_html_file(p: &Path) -> bool {
    if p.exists() {
        return p.is_file() && p.extension().and_then(|s| s.to_str()).is_some_and(|e| e.eq_ignore_ascii_case("html"));
    }
    p.extension()
        .and_then(|s| s.to_str())
        .is_some_and(|e| e.eq_ignore_ascii_case("html"))
}

fn write_results_bundle(out_dir: &Path, report: &AnalysisReport, config: &ReportConfig) -> Result<()> {
    fs::create_dir_all(out_dir)
        .with_context(|| format!("failed to create output dir: {}", out_dir.display()))?;

    let assets_dir = out_dir.join("assets");
    fs::create_dir_all(&assets_dir)
        .with_context(|| format!("failed to create assets dir: {}", assets_dir.display()))?;

    // Embedded templates / vendor
    const INDEX_HTML: &str = include_str!("../../assets/report/index.html");
    const STYLE_CSS: &str = include_str!("../../assets/report/style.css");
    const APP_JS: &str = include_str!("../../assets/report/app.js");
    const CYTO_JS: &[u8] = include_bytes!("../../assets/vendor/cytoscape.min.js");
    const CYTO_LICENSE: &str = include_str!("../../assets/vendor/cytoscape.LICENSE");

    let graph = build_graph_json(report);
    let graph_json = serde_json::to_string_pretty(&graph)?;
    let embedded_graph_json = escape_json_for_script_tag(&graph_json);

    let report_data = build_report_data(report);
    let report_data_json = serde_json::to_string_pretty(&report_data)?;
    let embedded_report_data_json = escape_json_for_script_tag(&report_data_json);

    let report_config_json = serde_json::to_string_pretty(config)?;
    let embedded_report_config_json = escape_json_for_script_tag(&report_config_json);

    let index_html = INDEX_HTML
        .replace("__GRAPH_JSON__", &embedded_graph_json)
        .replace("__REPORT_DATA_JSON__", &embedded_report_data_json)
        .replace("__REPORT_CONFIG_JSON__", &embedded_report_config_json);

    fs::write(out_dir.join("index.html"), index_html)
        .with_context(|| format!("failed to write {}", out_dir.join("index.html").display()))?;
    fs::write(assets_dir.join("style.css"), STYLE_CSS)
        .with_context(|| format!("failed to write {}", assets_dir.join("style.css").display()))?;
    fs::write(assets_dir.join("app.js"), APP_JS)
        .with_context(|| format!("failed to write {}", assets_dir.join("app.js").display()))?;
    fs::write(assets_dir.join("cytoscape.min.js"), CYTO_JS)
        .with_context(|| format!("failed to write {}", assets_dir.join("cytoscape.min.js").display()))?;

    fs::write(assets_dir.join("graph.json"), format!("{}\n", graph_json))
        .with_context(|| format!("failed to write {}", assets_dir.join("graph.json").display()))?;

    let notices = format!(
        "This report bundle includes third-party software.\n\n- cytoscape.js v3.30.2\n\n{}\n",
        CYTO_LICENSE
    );
    fs::write(out_dir.join("THIRD_PARTY_NOTICES.txt"), notices)
        .with_context(|| format!("failed to write {}", out_dir.join("THIRD_PARTY_NOTICES.txt").display()))?;

    Ok(())
}

fn escape_json_for_script_tag(json: &str) -> String {
    // Prevent prematurely closing the <script> tag.
    // This keeps the embedded JSON safe even if it contains "</script>".
    json.replace("</script>", "<\\/script>")
}

#[derive(Debug, Clone, Deserialize, Default)]
struct ReportConfigToml {
    sections: Option<Vec<String>>,
    hidden: Option<Vec<String>>,
}

#[derive(Debug, Clone, Serialize)]
struct ReportConfig {
    sections: Vec<String>,
    hidden: Vec<String>,
}

fn load_report_config(config_path: Option<&Path>) -> Result<ReportConfig> {
    let defaults = ReportConfig {
        sections: vec![
            "summary".to_string(),
            "overview".to_string(),
            "packages".to_string(),
            "package_findings".to_string(),
            "external_deps".to_string(),
            "legend".to_string(),
        ],
        hidden: Vec::new(),
    };

    let Some(path) = config_path else {
        return Ok(defaults);
    };

    let txt = fs::read_to_string(path)
        .with_context(|| format!("failed to read config: {}", path.display()))?;

    let cfg: ReportConfigToml = toml::from_str(&txt)
        .with_context(|| format!("failed to parse config TOML: {}", path.display()))?;

    Ok(ReportConfig {
        sections: cfg.sections.unwrap_or(defaults.sections),
        hidden: cfg.hidden.unwrap_or_default(),
    })
}

#[derive(Debug, Clone, Serialize)]
struct ReportData {
    summary: Vec<SummaryItem>,
    overview: OverviewData,
    packages: PackagesData,
    external_deps: ExternalDepsData,
}

#[derive(Debug, Clone, Serialize)]
struct SummaryItem {
    key: String,
    value: usize,
}

#[derive(Debug, Clone, Serialize)]
struct OverviewData {
    total_packages: usize,
    total_findings: usize,
    source_root: Option<String>,
    top_groups: Vec<GroupSummary>,
}

#[derive(Debug, Clone, Serialize)]
struct GroupSummary {
    name: String,
    packages: usize,
    packages_with_findings: usize,
}

#[derive(Debug, Clone, Serialize)]
struct ExternalDepsData {
    total_unique: usize,
    top: Vec<ExternalDepUsage>,
}

#[derive(Debug, Clone, Serialize)]
struct ExternalDepUsage {
    name: String,
    count: usize,
}

#[derive(Debug, Clone, Serialize)]
struct PackagesData {
    total: usize,
    items: Vec<PackageItem>,
}

#[derive(Debug, Clone, Serialize)]
struct PackageItem {
    name: String,
    path: String,
    findings_count: usize,
    has_findings: bool,
}

fn build_report_data(report: &AnalysisReport) -> ReportData {
    let total_packages = report.summary.get("total_packages").copied().unwrap_or(report.packages.len());
    let total_findings = report.summary.get("total_findings").copied().unwrap_or(report.findings.len());

    let mut summary: Vec<SummaryItem> = report
        .summary
        .iter()
        .filter_map(|(k, v)| Some(SummaryItem {
            key: k.clone(),
            value: *v,
        }))
        .collect();
    summary.sort_by(|a, b| a.key.cmp(&b.key));

    let source_root = common_path_prefix(report.packages.iter().map(|p| Path::new(&p.path)));
    let source_root_str = source_root.as_ref().map(|p| p.to_string_lossy().to_string());

    let mut package_has_findings: BTreeMap<String, bool> = BTreeMap::new();
    let mut package_findings_count: BTreeMap<String, usize> = BTreeMap::new();
    for p in &report.packages {
        let mut cnt = 0usize;
        for f in &report.findings {
            if f.file.starts_with(&p.path) {
                cnt += 1;
            }
        }
        package_findings_count.insert(p.name.clone(), cnt);
        package_has_findings.insert(p.name.clone(), cnt > 0);
    }

    let mut packages_items: Vec<PackageItem> = report
        .packages
        .iter()
        .map(|p| {
            let cnt = package_findings_count.get(&p.name).copied().unwrap_or(0);
            PackageItem {
                name: p.name.clone(),
                path: p.path.clone(),
                findings_count: cnt,
                has_findings: cnt > 0,
            }
        })
        .collect();
    packages_items.sort_by(|a, b| {
        b.findings_count
            .cmp(&a.findings_count)
            .then_with(|| a.name.cmp(&b.name))
    });

    // Group by first directory under common root.
    let mut groups: BTreeMap<String, (usize, usize)> = BTreeMap::new();
    if let Some(root) = &source_root {
        for p in &report.packages {
            let pkg_path = Path::new(&p.path);
            let rel = pkg_path.strip_prefix(root).unwrap_or(pkg_path);
            let group = rel
                .components()
                .next()
                .map(|c| c.as_os_str().to_string_lossy().to_string())
                .filter(|s| !s.is_empty())
                .unwrap_or_else(|| "(root)".to_string());

            let has = package_has_findings.get(&p.name).copied().unwrap_or(false) as usize;
            let e = groups.entry(group).or_insert((0, 0));
            e.0 += 1;
            e.1 += has;
        }
    }

    let mut top_groups: Vec<GroupSummary> = groups
        .into_iter()
        .map(|(name, (packages, packages_with_findings))| GroupSummary {
            name,
            packages,
            packages_with_findings,
        })
        .collect();
    top_groups.sort_by(|a, b| b.packages.cmp(&a.packages).then_with(|| a.name.cmp(&b.name)));
    top_groups.truncate(15);

    // External deps usage counts: number of workspace packages that depend on it (deduped per package).
    let workspace_names: BTreeSet<String> = report.packages.iter().map(|p| p.name.clone()).collect();
    let mut counts: BTreeMap<String, usize> = BTreeMap::new();
    for p in &report.packages {
        let mut seen: BTreeSet<String> = BTreeSet::new();
        for d in &p.dependencies {
            if workspace_names.contains(&d.name) {
                continue;
            }
            if seen.insert(d.name.clone()) {
                *counts.entry(d.name.clone()).or_insert(0) += 1;
            }
        }
    }

    let total_unique = counts.len();
    let mut top: Vec<ExternalDepUsage> = counts
        .into_iter()
        .map(|(name, count)| ExternalDepUsage { name, count })
        .collect();
    top.sort_by(|a, b| b.count.cmp(&a.count).then_with(|| a.name.cmp(&b.name)));
    top.truncate(50);

    ReportData {
        summary,
        overview: OverviewData {
            total_packages,
            total_findings,
            source_root: source_root_str,
            top_groups,
        },
        packages: PackagesData {
            total: report.packages.len(),
            items: packages_items,
        },
        external_deps: ExternalDepsData { total_unique, top },
    }
}

fn common_path_prefix<'a, I>(paths: I) -> Option<PathBuf>
where
    I: IntoIterator<Item = &'a Path>,
{
    let mut iter = paths.into_iter();
    let first = iter.next()?.components().collect::<Vec<_>>();
    if first.is_empty() {
        return None;
    }

    let mut prefix_len = first.len();
    for p in iter {
        let comps = p.components().collect::<Vec<_>>();
        let mut i = 0usize;
        while i < prefix_len && i < comps.len() && first[i] == comps[i] {
            i += 1;
        }
        prefix_len = i;
        if prefix_len == 0 {
            return None;
        }
    }

    let mut out = PathBuf::new();
    for c in &first[..prefix_len] {
        out.push(c.as_os_str());
    }
    Some(out)
}

#[derive(Debug, Clone, Serialize)]
struct GraphJson {
    nodes: Vec<GraphNode>,
    edges: Vec<GraphEdge>,
}

#[derive(Debug, Clone, Serialize)]
struct GraphNode {
    id: String,
    label: String,
    kind: String,
    has_findings: bool,
}

#[derive(Debug, Clone, Serialize)]
struct GraphEdge {
    source: String,
    target: String,
    dep_type: String,
}

fn build_graph_json(report: &AnalysisReport) -> GraphJson {
    // Use stable ordering for deterministic output.
    let mut nodes: BTreeMap<String, GraphNode> = BTreeMap::new();

    let workspace_names: BTreeSet<String> = report.packages.iter().map(|p| p.name.clone()).collect();

    // Precompute which packages have findings by path prefix.
    let mut has_findings: BTreeMap<String, bool> = BTreeMap::new();
    for p in &report.packages {
        let pfx = p.path.clone();
        let found = report
            .findings
            .iter()
            .any(|f| f.file.starts_with(&pfx));
        has_findings.insert(p.name.clone(), found);
    }

    for p in &report.packages {
        nodes.insert(
            p.name.clone(),
            GraphNode {
                id: p.name.clone(),
                label: p.name.clone(),
                kind: "workspace".to_string(),
                has_findings: has_findings.get(&p.name).copied().unwrap_or(false),
            },
        );
    }

    let mut edges: Vec<GraphEdge> = Vec::new();
    for p in &report.packages {
        for d in &p.dependencies {
            let target = d.name.clone();
            let dep_type = d
                .kind
                .as_deref()
                .unwrap_or("build")
                .to_string();

            if !workspace_names.contains(&target) {
                nodes.entry(target.clone()).or_insert(GraphNode {
                    id: target.clone(),
                    label: target.clone(),
                    kind: "external".to_string(),
                    has_findings: false,
                });
            }

            edges.push(GraphEdge {
                source: p.name.clone(),
                target,
                dep_type,
            });
        }
    }

    edges.sort_by(|a, b| {
        (a.source.as_str(), a.target.as_str(), a.dep_type.as_str()).cmp(&(
            b.source.as_str(),
            b.target.as_str(),
            b.dep_type.as_str(),
        ))
    });

    GraphJson {
        nodes: nodes.into_values().collect(),
        edges,
    }
}

fn render_html(report: &AnalysisReport) -> String {
    let total_packages = report.summary.get("total_packages").copied().unwrap_or(0);
    let total_findings = report.summary.get("total_findings").copied().unwrap_or(report.findings.len());

    let mut out = String::new();
    out.push_str("<!doctype html>\n<html lang=\"en\">\n<head>\n");
    out.push_str("<meta charset=\"utf-8\">\n<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">\n");
    out.push_str("<title>Chelonian Report</title>\n");
    out.push_str("</head>\n<body>\n");

    out.push_str("<h1>Chelonian Report</h1>\n");
    out.push_str("<ul>\n");
    out.push_str(&format!("<li>Total packages: {}</li>\n", total_packages));
    out.push_str(&format!("<li>Total findings: {}</li>\n", total_findings));
    out.push_str("</ul>\n");

    out.push_str("<h2>Findings</h2>\n");
    out.push_str("<table border=\"1\" cellspacing=\"0\" cellpadding=\"6\">\n");
    out.push_str("<thead><tr><th>Severity</th><th>Rule</th><th>File</th><th>Line</th><th>Message</th><th>Suggestion</th></tr></thead>\n");
    out.push_str("<tbody>\n");

    for f in &report.findings {
        out.push_str("<tr>");
        out.push_str(&format!("<td>{}</td>", escape_html(&f.severity)));
        out.push_str(&format!("<td>{}</td>", escape_html(&f.rule_id)));
        out.push_str(&format!("<td>{}</td>", escape_html(&f.file)));
        out.push_str(&format!(
            "<td>{}</td>",
            f.line.map(|v| v.to_string()).unwrap_or_default()
        ));
        out.push_str(&format!("<td>{}</td>", escape_html(&f.message)));
        out.push_str(&format!(
            "<td>{}</td>",
            f.suggestion
                .as_deref()
                .map(escape_html)
                .unwrap_or_default()
        ));
        out.push_str("</tr>\n");
    }

    out.push_str("</tbody></table>\n");
    out.push_str("</body>\n</html>\n");
    out
}

fn escape_html(s: &str) -> String {
    let mut out = String::with_capacity(s.len());
    for c in s.chars() {
        match c {
            '&' => out.push_str("&amp;"),
            '<' => out.push_str("&lt;"),
            '>' => out.push_str("&gt;"),
            '"' => out.push_str("&quot;"),
            '\'' => out.push_str("&#39;"),
            _ => out.push(c),
        }
    }
    out
}
