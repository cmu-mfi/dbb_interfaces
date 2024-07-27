# MQTT Broker to SQL

For installation and usage follow steps [here](https://github.com/cmu-mfi/dbb_interfaces?tab=readme-ov-file#install-and-usage)

## Config Files 

**config.yaml**

| Parameter | Details    |
| --------- | --------------------------   |
| `topics`      | MQTT topics to be listening for data <br> The payload is expected to be a [pickle](https://docs.python.org/3/library/pickle.html) serialized dictionary. |
| `message_dict`| Dictionary keys expected for all payloads unless specified explicitly <br> Changing the list requires change in code. |

**setup.yaml**

It contains details required to connect to the MQTT Broker and the SQL Server.


## SQL

...