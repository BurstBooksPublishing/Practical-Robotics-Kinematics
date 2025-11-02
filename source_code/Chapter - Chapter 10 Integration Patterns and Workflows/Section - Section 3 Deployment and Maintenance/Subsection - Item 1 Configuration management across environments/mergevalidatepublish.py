import yaml
import jsonschema
import rospy

# Load YAML from paths (base + overlay) and return merged dict.
def load_and_merge(paths):
    merged = {}
    for p in paths:
        with open(p, 'r') as f:
            data = yaml.safe_load(f) or {}
        # shallow-merge; extend to deep-merge as needed.
        merged.update(data)
    return merged

# Validate merged config against JSON Schema.
def validate_config(config, schema_path):
    with open(schema_path, 'r') as s:
        schema = yaml.safe_load(s)
    jsonschema.validate(instance=config, schema=schema)  # raises on failure

# Publish merged config to ROS parameter server.
def publish_to_ros(namespace, config):
    for k, v in config.items():
        rospy.set_param(f"{namespace}/{k}", v)  # node-global param namespace

if __name__ == "__main__":
    rospy.init_node('config_loader', anonymous=True)
    paths = ["robot_base.yaml", "overlay_prod.yaml"]  # deterministic order
    merged = load_and_merge(paths)
    validate_config(merged, "config_schema.yaml")
    publish_to_ros("/kinematics", merged)  # used by Pinocchio node
    rospy.loginfo("Published validated kinematics config")