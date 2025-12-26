use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct Dependency {
    pub name: String,
    pub version: Option<String>,
}

#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct Package {
    pub name: String,
    pub version: Option<String>,
    pub path: String,
    pub build_type: Option<String>,
    pub dependencies: Vec<Dependency>,
    pub format: Option<u8>,
}

#[allow(dead_code)]
impl Package {
    pub fn all_dependency_names(&self) -> Vec<String> {
        self.dependencies.iter().map(|d| d.name.clone()).collect()
    }

    pub fn is_ros1(&self) -> bool {
        let ros1_indicators = ["roscpp", "rospy", "message_generation", "message_runtime", "catkin"];
        self.all_dependency_names()
            .iter()
            .any(|d| ros1_indicators.contains(&d.as_str()))
    }

    pub fn is_ros2(&self) -> bool {
        let ros2_indicators = ["rclcpp", "rclpy", "rosidl_default_generators", "ament_cmake"];
        self.all_dependency_names()
            .iter()
            .any(|d| ros2_indicators.contains(&d.as_str()))
    }
}
