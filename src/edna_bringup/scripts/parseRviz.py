import sys
import yaml
import os

# NAMESPACE = os.environ.get('ROS_NAMESPACE') if 'ROS_NAMESPACE' in os.environ else 'default'

def processRvizFileForNamespace(rviz_file, NAMESPACE):
    rviz_data = None
    raw_rviz_data = None

    with open(rviz_file, 'r') as stream:
        raw_rviz_data = stream.read()
    
    rviz_data = yaml.safe_load(raw_rviz_data)

    # Get the namespace value from the rviz file
    current_namespace = None
    if rviz_data:
        for display in rviz_data['Visualization Manager']['Displays']:
            for k, v in display.items():
                if 'Topic' in k and 'Value' in v:
                    segs = v['Value'].split('/')
                    if len(segs) > 1:
                        current_namespace = segs[1]
                        break
    if current_namespace:
        print('Found namespace: ', current_namespace)
        print(f"mapping {current_namespace} -> {NAMESPACE}")
        new_rviz_data = yaml.safe_load(raw_rviz_data.replace(current_namespace, NAMESPACE))
        with open(rviz_file, 'w') as stream:
            yaml.dump(new_rviz_data, stream)
    else:
        with open('err', 'w') as stream:
            stream.write("Couldn't find namespace in rviz file")


if __name__ == "__main__":
    processRvizFileForNamespace(sys.argv[1], sys.argv[2])