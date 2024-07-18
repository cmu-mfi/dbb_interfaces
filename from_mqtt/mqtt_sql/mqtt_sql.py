import argparse
import base64
import logging
import os
import pickle
import time

import paho.mqtt.client as mqtt
import pymssql
import yaml


class SQLStore:
    def __init__(self, config_file) -> None:
        self.db_config = self.load_db_config(config_file)
        logging.basicConfig(
            level=logging.DEBUG,
            format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
            handlers=[
                logging.FileHandler('log.txt'),  # Log to a file
                logging.StreamHandler()          # Log to console
            ]
        )
        self.logger = logging.getself.logger(__name__)        
        # self.connection = pymssql.connect(
        #     server=self.db_config['server'],
        #     user=self.db_config['username'],
        #     password=self.db_config['password'],
        #     database=self.db_config['database']
        # )      
    
    # def __del__(self):
    #     self.connection.close()
    #     self.logger.info("Database connection closed")  
    
    def load_db_config(self, config_path='SECRET.yaml'):
        """Loads database configuration from a YAML file."""
        try:
            with open(config_path, 'r') as file:
                config = yaml.safe_load(file)
            return config['sql_server']
        except Exception as e:
            self.logger.error(f"Error loading database configuration: {e}")
            raise

    def read_xml_content(self, xml_content):
        """Reads an XML file from the specified path and removes the XML declaration."""
        self.logger.info(f"Reading XML content")
        try:
            # Remove the XML declaration
            if xml_content.startswith('<?xml'):
                xml_content = xml_content.split('?>', 1)[1]
            self.logger.debug("XML declaration removed")
            return xml_content
        except Exception as e:
            self.logger.error(f"Error reading XML file: {e}")
            raise

    def send_xml_to_stored_procedure(self, xml_content):
        """Sends XML content to a stored procedure in SQL Server."""
        try:
            # Establish connection
            self.logger.info("Establishing connection to the database")
            connection = pymssql.connect(
                server=self.db_config['server'],
                user=self.db_config['username'],
                password=self.db_config['password'],
                database=self.db_config['database']
            )
            cursor = connection.cursor()
            
            # Call the stored procedure
            self.logger.info(f"Calling stored procedure {self.db_config['stored_procedure']}")
            cursor.callproc(self.db_config['stored_procedure'], (xml_content,))
            connection.commit()
            self.logger.info("Stored procedure executed successfully")

        except Exception as e:
            self.logger.error(f"Error executing stored procedure: {e}")
            raise

        finally:
            # Close the connection
            cursor.close()
            connection.close()
            self.logger.info("Database connection closed")

class MQTTSubscriber:
    
    def __init__(self, secret, config):
        self.broker = secret['broker']
        self.port = secret['port']
        self.username = secret['username']
        self.password = secret['password']
        self.client = mqtt.Client()
        self.client.username_pw_set(self.username, self.password)
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        self.client.connect(self.broker, self.port, 60)
        
        self.config = config
        self.sql_store = SQLStore(secret['sql_server'])
        
        # list of topics to subscribe 
        self.topics = self.config['topics']
        for topic in self.topics:
            self.client.subscribe(topic)

    def on_connect(self, client, userdata, flags, rc):
        print(f"Connected with result code {rc}")

    def on_message(self, client, userdata, message):
        print(f"Received message '{message.payload.decode()}' on topic '{message.topic}'")
        data = base64.b64decode(message.payload)
        dict = pickle.loads(data)
        filename = "untitled"
        expected_ext = message.topic.split('/')[-1]
        ext = expected_ext
        
        if 'name' in dict:
            name = dict['name']
            filename = name.split('.')[0]
            ext = name.split('.')[-1]
            if ext == filename:
                ext = expected_ext
        
        if 'timestamp' in dict:
            timestamp = dict['timestamp']
            filename += f"_{timestamp}"
        
        format = "%Y-%m-%d_%H-%M"
        filename += f"_{time.strftime(format)}" 
        filename += f".{ext}"
        
        path = self.config['output_path']
        path = os.path.join(path, expected_ext)
        if not os.path.exists(path):
            os.makedirs(path)
        
        xml_content = self.sql_store.read_xml_content(dict['file'])
        self.sql_store.send_xml_to_stored_procedure(xml_content)
        
if __name__ == '__main__':
    
    with open('secret.yaml', 'r') as file:
        secret = yaml.safe_load(file)
        
    with open('config.yaml', 'r') as file:
        config = yaml.safe_load(file)

    mqtt_pub = MQTTSubscriber(secret, config)
    
    mqtt_pub.client.loop_forever()
    
