use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct FindPackage {
    pub name: String,
    pub version: Option<String>,
    pub required: bool,
}

#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct CMakeInfo {
    pub find_packages: Vec<FindPackage>,
    pub has_catkin_package: bool,
    pub has_ament_package: bool,
}
