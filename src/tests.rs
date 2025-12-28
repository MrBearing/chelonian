#[cfg(test)]
mod tests {
    use std::fs::{create_dir_all, write};
        use std::path::PathBuf;
    use tempfile::tempdir;

    use crate::plugins::loader::load_rules_from_path;
    use crate::scanner::scan_workspace;
    use crate::analyzer::analyze;
        use crate::commands;
        use crate::models::AnalysisReport;

    #[test]
    fn integration_rules_and_scan() {
        let td = tempdir().expect("tempdir");
        let base = td.path();

        // create test workspace structure
        let src = base.join("src");
        let my_robot = src.join("my_robot");
        let my_robot_src = my_robot.join("src");
        create_dir_all(&my_robot_src).expect("mkdir");

        // package.xml (ROS1 with roscpp)
        let pkg = r#"<package format="2">
  <name>my_robot</name>
  <version>0.1.0</version>
  <depend>roscpp</depend>
</package>"#;
        write(my_robot.join("package.xml"), pkg).expect("write package.xml");

        // CMakeLists.txt
        let cmake = r#"cmake_minimum_required(VERSION 3.0.2)
find_package(catkin REQUIRED COMPONENTS roscpp)
"#;
        write(my_robot.join("CMakeLists.txt"), cmake).expect("write cmake");

        // C++ file with ros include
        let cpp = r#"#include <ros/ros.h>
int main() { return 0; }
"#;
        write(my_robot_src.join("my_node.cpp"), cpp).expect("write cpp");

        // create a custom rules dir with a TOML rule (optional)
        let rules_dir = base.join("rules");
        create_dir_all(&rules_dir).expect("mkdir rules");
        let rule_toml = r###"[meta]
name = "test_rules"
version = "0.1"

[[rules]]
id = "custom-include-ros"
name = "custom ros include"
severity = "info"
target = "cpp"

[rules.match]
type = "include"
pattern = "ros/ros.h"

[rules.output]
message = "custom detect ros include"
suggestion = "#include <rclcpp/rclcpp.hpp>"
"###;
        write(rules_dir.join("custom.toml"), rule_toml).expect("write toml");

        // load rules (pass rules dir and platform=ros1)
        let rules = load_rules_from_path(Some(rules_dir.clone()), Some("ros1".to_string()), true).expect("load rules");
        assert!(rules.iter().any(|r| r.id == "ros1-header-ros"));
        assert!(rules.iter().any(|r| r.id == "custom-include-ros"));

        // scan and analyze
        let scan = scan_workspace(base.to_str().unwrap());
        let report = analyze(&scan, &rules);

        // Expect at least two findings: built-in header and dependency (or custom)
        assert!(report.findings.len() >= 1, "expected some findings");

        // check that ros include finding exists
        let has_ros_include = report.findings.iter().any(|v| v.rule_id == "ros1-header-ros" || v.rule_id == "custom-include-ros");
        assert!(has_ros_include, "expected ros include rule to trigger");

        // check that roscpp dependency finding exists
        let has_roscpp = report.findings.iter().any(|v| v.rule_id == "ros1-dep-roscpp");
        assert!(has_roscpp, "expected roscpp dependency rule to trigger");
    }

        #[test]
        fn analyze_command_writes_json_report() {
                let td = tempdir().expect("tempdir");
                let base = td.path();

                // Minimal workspace
                let src = base.join("src");
                let pkg_dir = src.join("my_pkg");
                create_dir_all(pkg_dir.join("src")).expect("mkdir");
                write(
                        pkg_dir.join("package.xml"),
                        r#"<package format="2"><name>my_pkg</name><version>0.1.0</version><depend>roscpp</depend></package>"#,
                )
                .expect("write package.xml");

                // output path
                let out_json = base.join("out.json");

                let args = commands::analyze::AnalyzeArgs {
                        workspace_path: Some(PathBuf::from(base)),
                        format: "json".to_string(),
                        output: Some(out_json.clone()),
                        rules: None,
                        platform: Some("ros1".to_string()),
                        no_builtin: false,
                        list_rules: false,
                        verbose: 0,
                };

                commands::analyze::run(args).expect("analyze should succeed");

                let bytes = std::fs::read(out_json).expect("read out.json");
                let report: AnalysisReport = serde_json::from_slice(&bytes).expect("parse json");

                assert!(report.summary.get("total_packages").copied().unwrap_or(0) >= 1);
                // total_findings may be 0 depending on builtin rules and workspace contents
        }

        #[test]
        fn report_command_writes_html() {
                let td = tempdir().expect("tempdir");
                let base = td.path();

                let input_json = base.join("report.json");
                let output_html = base.join("report.html");

                let report = AnalysisReport {
                        summary: std::collections::HashMap::from([
                                ("total_packages".to_string(), 1usize),
                                ("total_findings".to_string(), 1usize),
                        ]),
                        packages: Vec::new(),
                        findings: vec![crate::models::Finding {
                                rule_id: "test-rule".to_string(),
                                severity: "warning".to_string(),
                                file: "src/main.cpp".to_string(),
                                line: Some(42),
                                message: "something happened".to_string(),
                                suggestion: Some("try something else".to_string()),
                                effort_hours: None,
                        }],
                };

                std::fs::write(&input_json, serde_json::to_string_pretty(&report).unwrap())
                        .expect("write report.json");

                let args = commands::report::ReportArgs {
                        input: input_json,
                        output: Some(output_html.clone()),
                };

                commands::report::run(args).expect("report should succeed");

                let html = std::fs::read_to_string(output_html).expect("read report.html");
                assert!(html.contains("<h1>Chelonian Report</h1>"));
                assert!(html.contains("test-rule"));
                assert!(html.contains("something happened"));
                assert!(html.contains("try something else"));
        }
}
