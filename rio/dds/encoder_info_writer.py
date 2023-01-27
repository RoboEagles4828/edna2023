import rticonnextdds_connector as rti
import inspect
import os

class EncoderInfoWriter:

    def __init__(self, connector):
        # self.connector = rti.Connector(config_name="ROS2_PARTICIPANT_LIB::encoder_info", url=xml_path)
        self.output = connector.get_output("encoder_info_publisher::encoder_info_writer")

        # print("Waiting for subscriptions...")
        # self.output.wait_for_subscriptions()
    
    def sendData(self, encoder_info):
        names = list()
        positions = list()
        velocities = list()

        if encoder_info:
            for module_value in encoder_info.values():
                for joint in module_value.values():
                    names.append(joint['name'])
                    positions.append(float(joint['position']))
                    velocities.append(float(joint['velocity']))

        self.output.instance.set_dictionary({"name": names, "position": positions, "velocity": velocities})
        self.output.write()

    # def closeConnector(self):
    #     print("CLOSING ENCODER CONNECTOR")
    #     self.connector.close()