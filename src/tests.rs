#[cfg(test)]
mod tests {
    use std::fs::{create_dir_all, write};
    use tempfile::tempdir;

    use crate::plugins::loader::load_rules_from_path;
    use crate::scanner::scan_workspace;
    use crate::analyzer::analyze;

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

        // load rules (pass rules dir)
        let rules = load_rules_from_path(Some(rules_dir.clone()), true).expect("load rules");
        assert!(rules.iter().any(|r| r.id == "ros1-header-ros"));
        assert!(rules.iter().any(|r| r.id == "custom-include-ros"));

        // scan and analyze
        let scan = scan_workspace(base.to_str().unwrap());
        let report = analyze(&scan, &rules);

        // Expect at least two violations: built-in header and dependency (or custom)
        assert!(report.violations.len() >= 1, "expected some violations");

        // check that ros include violation exists
        let has_ros_include = report.violations.iter().any(|v| v.rule_id == "ros1-header-ros" || v.rule_id == "custom-include-ros");
        assert!(has_ros_include, "expected ros include rule to trigger");

        // check that roscpp dependency violation exists
        let has_roscpp = report.violations.iter().any(|v| v.rule_id == "ros1-dep-roscpp");
        assert!(has_roscpp, "expected roscpp dependency rule to trigger");
    }
}
