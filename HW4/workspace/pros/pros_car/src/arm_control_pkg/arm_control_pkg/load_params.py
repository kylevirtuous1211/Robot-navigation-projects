import os
import yaml
from ament_index_python.packages import get_package_share_directory


class LoadParams:
    def __init__(self, package_name):
        self.package_name = package_name
        self.arm_params = {}
        self._load_arm_parameters()

    def _load_arm_parameters(self):
        """Load arm parameters from YAML file as a dictionary"""
        try:
            # Get the path to the package directory
            package_path = os.path.join(
                get_package_share_directory(self.package_name), "config"
            )
            # Construct the full path to the YAML file
            yaml_file_path = os.path.join(package_path, "arm_config.yaml")

            # Load the YAML file
            with open(yaml_file_path, "r") as file:
                self.arm_params = yaml.safe_load(file)

        except Exception as e:
            raise RuntimeError(f"Failed to load arm parameters: {e}")

    def get_arm_params(self):
        return self.arm_params
