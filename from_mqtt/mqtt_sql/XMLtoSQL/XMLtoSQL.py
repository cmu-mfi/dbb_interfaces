import pymssql
import logging
import yaml

# Configure logging
logging.basicConfig(
    level=logging.DEBUG,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler('log.txt'),  # Log to a file
        logging.StreamHandler()          # Log to console
    ]
)

logger = logging.getLogger(__name__)

def load_db_config(config_path='SECRET.yaml'):
    """Loads database configuration from a YAML file."""
    try:
        with open(config_path, 'r') as file:
            config = yaml.safe_load(file)
        return config['sql_server']
    except Exception as e:
        logger.error(f"Error loading database configuration: {e}")
        raise

def read_xml_file(file_path):
    """Reads an XML file from the specified path and removes the XML declaration."""
    logger.info(f"Reading XML file from {file_path}")
    try:
        with open(file_path, 'r', encoding='utf-8') as file:
            xml_content = file.read()
        # Remove the XML declaration
        if xml_content.startswith('<?xml'):
            xml_content = xml_content.split('?>', 1)[1]
        logger.debug("XML declaration removed")
        return xml_content
    except Exception as e:
        logger.error(f"Error reading XML file: {e}")
        raise

def send_xml_to_stored_procedure(xml_content, db_config):
    """Sends XML content to a stored procedure in SQL Server."""
    try:
        # Establish connection
        logger.info("Establishing connection to the database")
        connection = pymssql.connect(
            server=db_config['server'],
            user=db_config['username'],
            password=db_config['password'],
            database=db_config['database']
        )
        cursor = connection.cursor()
        
        # Call the stored procedure
        logger.info(f"Calling stored procedure {db_config['stored_procedure']}")
        cursor.callproc(db_config['stored_procedure'], (xml_content,))
        connection.commit()
        logger.info("Stored procedure executed successfully")

    except Exception as e:
        logger.error(f"Error executing stored procedure: {e}")
        raise

    finally:
        # Close the connection
        cursor.close()
        connection.close()
        logger.info("Database connection closed")

def main():
    # Load database configuration
    db_config = load_db_config()

    # Define the path to your XML file (using raw string literal for Windows path)
    xml_file_path = r'C:\Users\ssterenr\Documents\newfile.xml'
    
    try:
        # Read the XML file
        xml_content = read_xml_file(xml_file_path)
        
        # Send XML content to the stored procedure
        send_xml_to_stored_procedure(xml_content, db_config)
        logger.info("XML content sent to stored procedure successfully")
    
    except Exception as e:
        logger.error(f"Error in main: {e}")
        raise

if __name__ == '__main__':
    main()






# import pyodbc

# def read_xml_file(file_path):
#     """Reads an XML file from the specified path and removes the XML declaration."""
#     with open(file_path, 'r', encoding='utf-8') as file:
#         xml_content = file.read()
#     # Remove the XML declaration
#     if xml_content.startswith('<?xml'):
#         xml_content = xml_content.split('?>', 1)[1]
#     return xml_content

# def send_xml_to_stored_procedure(xml_content):
#     """Sends XML content to a stored procedure in SQL Server."""
#     # Define connection parameters
#     server = 'ZENITH'
#     database = 'DBB'
#     username = 'Test'
#     password = 'Test'
#     stored_procedure = 'InsertXmlData'
    
#     # Establish connection
#     connection = pyodbc.connect(f'DRIVER={{ODBC Driver 17 for SQL Server}};SERVER={server};DATABASE={database};UID={username};PWD={password}')
#     cursor = connection.cursor()
    
#     # Call the stored procedure
#     cursor.execute(f"EXEC {stored_procedure} @XmlData = ?", xml_content)
#     connection.commit()

#     # Close the connection
#     cursor.close()
#     connection.close()

# def main():
#     # Define the path to your XML file (using raw string literal for Windows path)
#     xml_file_path = r'C:\Users\ssterenr\Documents\newfile.xml'
    
#     # Read the XML file
#     xml_content = read_xml_file(xml_file_path)
    
#     # Send XML content to the stored procedure
#     send_xml_to_stored_procedure(xml_content)

# if __name__ == '__main__':
#     main()
