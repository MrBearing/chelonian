use crate::models::{CMakeInfo, CppAnalysis, Package};
use crate::parsers::{parse_cmake_lists, parse_cpp_file, parse_package_xml};
use std::collections::HashMap;
use walkdir::WalkDir;

#[derive(Debug, Default)]
pub struct ScanResult {
    pub packages: Vec<Package>,
    pub cmake_map: HashMap<String, CMakeInfo>,
    pub cpp_map: HashMap<String, CppAnalysis>,
}

pub fn scan_workspace(path: &str) -> ScanResult {
    let mut result = ScanResult::default();

    let skip_dirs = [".git", "build", "install", "log", "devel", "target"];

    for entry in WalkDir::new(path).into_iter().filter_map(|e| e.ok()) {
        let p = entry.path();
        if entry.file_type().is_dir() {
            if let Some(name) = p.file_name().and_then(|n| n.to_str()) {
                if skip_dirs.contains(&name) {
                    continue;
                }
            }
        }

        if entry.file_type().is_file() {
            if let Some(name) = p.file_name().and_then(|n| n.to_str()) {
                if name == "package.xml" {
                    if let Some(parent) = p.parent() {
                        match parse_package_xml(p.to_str().unwrap_or_default()) {
                            Ok(mut pkg) => {
                                pkg.path = parent.to_string_lossy().to_string();
                                result.packages.push(pkg);
                            }
                            Err(e) => eprintln!("warning: failed to parse {}: {}", p.display(), e),
                        }
                    }
                } else if name == "CMakeLists.txt" {
                    if let Some(parent) = p.parent() {
                        match parse_cmake_lists(p.to_str().unwrap_or_default()) {
                            Ok(info) => { result.cmake_map.insert(parent.to_string_lossy().to_string(), info); }
                            Err(e) => eprintln!("warning: failed to parse {}: {}", p.display(), e),
                        }
                    }
                } else if name.ends_with(".cpp") || name.ends_with(".cc") || name.ends_with(".hpp") || name.ends_with(".h") {
                    match parse_cpp_file(p.to_str().unwrap_or_default()) {
                        Ok(analysis) => {
                            // parse_cpp_file now stores full file text in analysis.full_text
                            result.cpp_map.insert(p.to_string_lossy().to_string(), analysis);
                        }
                        Err(e) => eprintln!("warning: failed to parse {}: {}", p.display(), e),
                    }
                }
            }
        }
    }

    result
}
