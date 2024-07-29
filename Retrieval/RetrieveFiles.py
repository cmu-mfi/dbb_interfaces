import pymssql
import os
import zipfile
import logging
from datetime import datetime

# Configure logging
logging.basicConfig(filename='logging.txt', level=logging.INFO,
                    format='%(asctime)s - %(levelname)s - %(message)s')

# Function to connect to SQL Server
def get_connection():
    try:
        conn = pymssql.connect(server='ZENITH',
                               user='Test',
                               password='Test',
                               database='DBB')
        logging.info("Connected to the SQL Server database successfully.")
        return conn
    except Exception as e:
        logging.error(f"Error connecting to database: {e}")
        raise

# Function to execute a stored procedure and fetch results
def fetch_data(procedure_name, params=None):
    conn = get_connection()
    cursor = conn.cursor()
    
    try:
        if params:
            cursor.callproc(procedure_name, params)
        else:
            cursor.callproc(procedure_name)
        
        rows = cursor.fetchall()
        logging.info(f"Fetched data from procedure {procedure_name}.")
        return rows
    except Exception as e:
        logging.error(f"Error executing procedure {procedure_name}: {e}")
        raise
    finally:
        conn.close()

# Function to convert XML string to a file
def save_xml_to_file(xml_string, filename):
    try:
        with open(filename, 'w') as file:
            file.write(xml_string)
        logging.info(f"XML data saved to {filename}.")
    except Exception as e:
        logging.error(f"Error saving XML to file {filename}: {e}")
        raise

# Function to zip files
def zip_files(file_paths, zip_name, xml_file):
    try:
        with zipfile.ZipFile(zip_name, 'w') as zipf:
            for file in file_paths:
                zipf.write(file, os.path.basename(file))
            zipf.write(xml_file, os.path.basename(xml_file))
        logging.info(f"Files and XML zipped into {zip_name}.")
    except Exception as e:
        logging.error(f"Error creating zip file {zip_name}: {e}")
        raise

# Function to find files in a date range
def find_files_in_date_range(start_time, end_time, directory):
    file_paths = []
    try:
        for root, dirs, files in os.walk(directory):
            for file in files:
                file_path = os.path.join(root, file)
                file_mtime = datetime.fromtimestamp(os.path.getmtime(file_path))
                if start_time <= file_mtime <= end_time:
                    file_paths.append(file_path)
        logging.info(f"Found {len(file_paths)} files in date range.")
        return file_paths
    except Exception as e:
        logging.error(f"Error finding files in date range: {e}")
        raise

# Fetch and display projects
try:
    projects = fetch_data('dbo.ProjectList')
    logging.info("Retrieved project list.")
    print("Projects:")
    for i, project in enumerate(projects, 1):
        print(f"{i}. {project[1]} (ID: {project[0]})")
    
    # Get user selection for project
    project_choice = int(input("Select a project: "))
    selected_project = projects[project_choice - 1]
    selected_project_id = selected_project[0]
    selected_project_name = selected_project[1]
    
    # Fetch and display experiments for the selected project
    experiments = fetch_data('dbo.ExperimentList', (selected_project_id,))
    logging.info(f"Retrieved experiments for project ID {selected_project_id}.")
    print("Experiments:")
    for i, exp in enumerate(experiments, 1):
        start_time = exp[2].strftime('%Y-%m-%d %H:%M:%S')
        end_time = exp[3].strftime('%Y-%m-%d %H:%M:%S')
        print(f"{i}. {exp[1]} [{start_time} - {end_time}]")
    
    # Get user selection for experiment
    experiment_choice = int(input("Select an experiment number: "))
    selected_experiment = experiments[experiment_choice - 1]
    selected_experiment_title = selected_experiment[1]
    start_time = selected_experiment[2]
    end_time = selected_experiment[3]
    
    # Find files in the specified time range
    files = find_files_in_date_range(start_time, end_time, 'E:/')
    
    # Fetch XML data with start_time parameter
    xml_data = fetch_data('dbo.getXML', (start_time,))[0][0]  # Assuming XML data is in the first column of the result
    xml_file = 'Metadata.xml'
    save_xml_to_file(xml_data, xml_file)
    
    # Create ZIP file name
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    zip_file_name = f"{selected_project_name}_{selected_experiment_title}_{timestamp}.zip"
    
    # Create zip file with the selected files and XML
    zip_files(files, zip_file_name, xml_file)
    
    print(f"Files and XML have been zipped into {zip_file_name}")
except Exception as e:
    logging.error(f"An error occurred: {e}")
    print("An error occurred. Please check the log file for details.")
