import os
import xmltodict
import yaml
import time

def load_template(template_file):
    file_text = open(template_file, 'r').read()
    xml_template = xmltodict.parse(file_text)    
    return xml_template

def fill_dict(template: dict, parent_key: str = "", parent_dict: dict = {}):
    complete_dict = {}

    for key, value in template.items():
        if isinstance(value, dict):
            complete_dict[key] = fill_dict(value, parent_key + key + ".")
        else:
            complete_dict[key] = input(f"\nEnter value for '{parent_key + key}':\n[{template[key]}]\n")
    
    return complete_dict

def main():
    
    # +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++    
    # 1. SELECT TEMPLATE FILE TO FILL OUT
    # +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    print("Select the template XML file to fill out:")
    print("1. project.xml")
    print("2. experiment.xml")
    template_file = input("Enter the number of the template file: ")
    
    script_dir = os.path.dirname(__file__)

    # +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++    
    # 2. LOAD TEMPLATE FILE AND OUTDIR PATH FROM CONFIG FILE
    # +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    
    with open(os.path.join(script_dir, '../config.yaml'), 'r') as file:
        config = yaml.safe_load(file)
    output_dir = config['watch_dir'][0]
    timestamp = time.strftime("%m-%d_%H-%M-%S")
    
    if template_file == "1":
        template_file = os.path.join(script_dir, "project.xml")
        output_file = os.path.join(output_dir, f"project_{timestamp}.xml")
    elif template_file == "2":
        template_file = os.path.join(script_dir, "experiment.xml")
        output_file = os.path.join(output_dir, f"experiment_{timestamp}.xml")
    else:
        print("Invalid selection. Exiting...")
        return

    template = load_template(template_file)
    print("Template loaded successfully")
    
    # +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++    
    # 3. FILL OUT TEMPLATE FILE
    # +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
       
    print("=====================================================")
    print(xmltodict.unparse(template, pretty=True))
    print("=====================================================")
    print("Fill out the template:")
    
    complete_template = fill_dict(template)
    
    # +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++    
    # 4. WRITE XML FILE TO OUTPUT DIRECTORY
    # +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
           
    with open(output_file, 'w') as f:
        f.write(xmltodict.unparse(complete_template, pretty=True))   
    
    print(f"XML file saved to: {output_file}") 

if __name__ == "__main__":
    main()
