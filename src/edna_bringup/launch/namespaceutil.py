import yaml

def processRvizFileForNamespace(rviz_file, tmp_rviz_file, NAMESPACE):
    rviz_data = None
    with open(rviz_file, 'r') as stream:
        rviz_data = yaml.safe_load(stream)

    for display in rviz_data['Visualization Manager']['Displays']:
        for k, v in display.items():
            if 'Topic' in k and 'Value' in v:
                print(f"mapping {v['Value']} -> /{NAMESPACE}{v['Value']}")
                v['Value'] = f"/{NAMESPACE}{v['Value']}"

    print("Writing tmp rviz file")
    with open(tmp_rviz_file, 'w') as stream:
        yaml.dump(rviz_data, stream)


# Not sure how to use this one yet
def saveNamespaceFile(rviz_file, tmp_rviz_file, NAMESPACE):
    rviz_data = None
    with open(tmp_rviz_file, 'r') as stream:
        rviz_data = yaml.safe_load(stream)

    for display in rviz_data['Visualization Manager']['Displays']:
        for k, v in display.items():
            if 'Topic' in k and 'Value' in v:
                removedNS = v['Value'].split("/")[1] if "/" in v['Value'] else v['Value']
                print(f"mapping {v['Value']} -> {removedNS}")
                v['Value'] = removedNS

    print("Writing changes in tmp rviz file back to main file")
    with open(rviz_file, 'w') as stream:
        yaml.dump(rviz_data, stream)