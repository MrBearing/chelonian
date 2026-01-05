use crate::models::cmake::{CMakeInfo, FindPackage};
use anyhow::Result;
use regex::Regex;
use std::fs;

pub fn parse_cmake_lists(path: &str) -> Result<CMakeInfo> {
    let text = fs::read_to_string(path)?;
    let mut info = CMakeInfo::default();

    let re_find =
        Regex::new(r"find_package\s*\(\s*([A-Za-z0-9_:+-]+)(?:\s+([0-9\.]+))?(?:.*REQUIRED)?\)")
            .unwrap();
    for cap in re_find.captures_iter(&text) {
        let name = cap
            .get(1)
            .map(|m| m.as_str().to_string())
            .unwrap_or_default();
        let version = cap.get(2).map(|m| m.as_str().to_string());
        let required = cap
            .get(0)
            .map(|m| m.as_str().contains("REQUIRED"))
            .unwrap_or(false);
        info.find_packages.push(FindPackage {
            name,
            version,
            required,
        });
    }

    if text.contains("catkin_package") {
        info.has_catkin_package = true;
    }
    if text.contains("ament_package") {
        info.has_ament_package = true;
    }

    Ok(info)
}
