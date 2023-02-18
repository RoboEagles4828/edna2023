import rticonnextdds_connector as rti
import logging


# Subscribe to DDS topics
class DDS_Subscriber:
    def __init__(self, xml_path, participant_name, reader_name):
        # Connectors are not thread safe, so we need to create a new one for each thread
        self.connector = rti.Connector(config_name=participant_name, url=xml_path)
        self.input = self.connector.get_input(reader_name)

    def read(self) -> dict:
        # Take the input data off of queue and read it
        self.input.take()
        # Return the first valid data sample
        data = None
        for sample in self.input.samples.valid_data_iter:
            data = sample.get_dictionary()
            break
        return data

    def close(self):
        self.connector.close()


# Publish to DDS topics
class DDS_Publisher:
    def __init__(self, xml_path, participant_name, writer_name):
        self.connector = rti.Connector(config_name=participant_name, url=xml_path)
        self.output = self.connector.get_output(writer_name)
    
    def write(self, data) -> None:
        if data:
            self.output.instance.set_dictionary(data)
            self.output.write()
        else:
            logging.warn("No data to write")

    def close(self):
        self.connector.close()