#!/usr/bin/python3

import rclpy
from rclpy.node import Node
import paho.mqtt.publish as publish
from std_msgs.msg import String
import json
from datetime import datetime
import pytz

class BarcodeSubscriber(Node):
    def __init__(self):
        super().__init__('mqtt_publisher')
        self.subscription = self.create_subscription(String, '/encoded_barcode', self.callback, 10)
        self.subscription  # prevent unused variable warning

    def callback(self, msg):
        barcode_data = msg.data
        Date_Time = int(datetime.now(pytz.timezone('UTC')).timestamp())
        print("Barcode Data: ", barcode_data)
        print("Date and Time: ", Date_Time)

        #  MQTT broker details    
        mqttHost = "broker.mqttdashboard.com"  # Replace with your HiveMQ broker address
        mqttPort = 1883  # Use 1883 for MQTT or 8883 for MQTT over SSL/TLS
        mqtt_client_ID = "mynameisarief12345"  # The Client ID can be used to uniquely identify the client connected to the broker.
        
        topic = "mfibarcode/logistic/FYP"
        
        # Create a dictionary with the desired data
        message_data = {
            "barcode_data": barcode_data,
            "log_timestamp": Date_Time
        }
        # Convert dictionary to JSON string
        message_json = json.dumps(message_data)
        print(message_json)
        # Publish data to the HiveMQ broker using Paho MQTT
        try:
            publish.single(topic, message_json, hostname=mqttHost, port=mqttPort, client_id=mqtt_client_ID)
        except KeyboardInterrupt:
            pass
        except Exception as e:
            print("There was an error while publishing the data:", str(e))


def main(args=None):
    rclpy.init(args=args)
    mqtt_publisher = BarcodeSubscriber()
    rclpy.spin(mqtt_publisher)
    mqtt_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()