# Local File Storage to MQTT Broker

Publishes new files created in the local directories being monitored. 

Files are published to MQTT topic `lfs/<file extension>`. For example, if a new .xml is created, it will be published to `lfs/xml`

For installation and usage follow steps [here](https://github.com/cmu-mfi/dbb_interfaces?tab=readme-ov-file#install-and-usage)

## Config Files 

**config.yaml**

| Parameter | Details    |
| --------- | --------------------------   |
| `watch_dir` | List of directories to watch for new files <br> All subdirectories are included. |
| `message_dict`| Dictionary keys to populate for the message payload. <br> They are serialized using [pickle](https://docs.python.org/3/library/pickle.html).|

**setup.yaml**

It contains details required to connect to the MQTT Broker.