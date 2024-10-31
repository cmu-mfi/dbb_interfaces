import xmltodict

def load_template(template_file):
    file_text = open(template_file, 'r').read()
    xml_template = xmltodict.parse(file_text)    
    return xml_template

def fill_dict(template: dict, key_level: str = ""):
    complete_dict = {}

    for key, value in template.items():
        if isinstance(value, dict):
            complete_dict[key] = fill_dict(value, key_level + key + ".")
        else:
            complete_dict[key] = input(f"Enter value for '{key_level + key}': ")
    
    return complete_dict

def main():
    # template_file = "/home/mfi/repos/dbb_interfaces/to_mqtt/lfs_mqtt/write_xml/project.xml"
    template_file = "/home/mfi/repos/dbb_interfaces/to_mqtt/lfs_mqtt/write_xml/experiment.xml"
    output_file = "output.xml" 

    template = load_template(template_file)
    
    complete_template = fill_dict(template)
    
    with open(output_file, 'w') as f:
        f.write(xmltodict.unparse(complete_template, pretty=True))

if __name__ == "__main__":
    main()
