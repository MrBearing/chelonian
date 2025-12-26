use crate::models::package::{Dependency, Package};
use anyhow::Result;
use roxmltree::Document;
use std::fs;

pub fn parse_package_xml(path: &str) -> Result<Package> {
    let text = fs::read_to_string(path)?;
    let doc = Document::parse(&text)?;
    let root = doc.root_element();

    let mut pkg = Package {
        name: String::new(),
        version: None,
        path: path.to_string(),
        build_type: None,
        dependencies: Vec::new(),
        format: None,
    };

    if let Some(fmt) = root.attribute("format") {
        if let Ok(n) = fmt.parse::<u8>() {
            pkg.format = Some(n);
        }
    }

    for child in root.children().filter(|n| n.is_element()) {
        match child.tag_name().name() {
            "name" => pkg.name = child.text().unwrap_or_default().to_string(),
            "version" => pkg.version = child.text().map(|s| s.to_string()),
            name if name.ends_with("depend") || name == "build_depend" || name == "exec_depend" || name == "depend" => {
                if let Some(dep) = child.text() {
                    pkg.dependencies.push(Dependency { name: dep.to_string(), version: None });
                }
            }
            _ => {}
        }
    }

    // Heuristic build_type
    for d in &pkg.dependencies {
        if d.name == "catkin" {
            pkg.build_type = Some("catkin".to_string());
        }
        if d.name == "ament_cmake" {
            pkg.build_type = Some("ament_cmake".to_string());
        }
        if d.name == "ament_python" {
            pkg.build_type = Some("ament_python".to_string());
        }
    }

    Ok(pkg)
}
