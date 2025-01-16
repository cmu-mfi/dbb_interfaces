# MQTT Broker to Cloud File Storage

For installation and usage follow steps [here](https://github.com/cmu-mfi/dbb_interfaces?tab=readme-ov-file#install-and-usage)

## Config Files 

**config.yaml**

| Parameter | Details    |
| --------- | --------------------------   |
| `output_path` | Directory where you want files to be saved |
| `topics`      | MQTT topics to be listening for data <br> The payload is expected to be a [pickle](https://docs.python.org/3/library/pickle.html) serialized dictionary. |
| `message_dict`| Dictionary keys expected for all payloads unless specified explicitly. <br> Changing the list requires change in code. |
| *<topic_name>*| Specifies list of keys if different from `message_dict` <br> If custom keys, they need a callback handler (like audio_download.py) |

**setup.yaml**

It contains details required to connect to the MQTT Broker.