import yaml
yaml_path = '/workspaces/edna2023/src/edna_bringup/config/xbox-real.yaml'
with open(yaml_path, 'r') as f:
    yaml = yaml.safe_load(f)
yaml = yaml['/*']['joint_trajectory_teleop_node']['ros__parameters']

print(str(yaml['function_mapping']['elevator_loading_station']['button']))