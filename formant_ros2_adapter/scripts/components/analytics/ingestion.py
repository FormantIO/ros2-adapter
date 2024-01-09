import logging
import threading
import os
import time

# Configure logging
logging.basicConfig(filename='ingestion_analytics.log', level=logging.INFO,
                    format='%(asctime)s:%(levelname)s:%(message)s')

class IngestionAnalytics:
    def __init__(self):
        self.received_messages_count = {}
        self.sent_messages_count = {}
        self.enabled = self._check_env_var()
        self.reporting_thread = None
        if self.enabled:
            self.start_reporting_thread()

    def _check_env_var(self):
        return os.getenv('ANALYTICS_INGESTION', 'false').lower() == 'true'

    def log_received_message(self, topic):
        if topic not in self.received_messages_count:
            self.received_messages_count[topic] = 0
        self.received_messages_count[topic] += 1

    def log_sent_message(self, stream):
        if stream not in self.sent_messages_count:
            self.sent_messages_count[stream] = 0
        self.sent_messages_count[stream] += 1

    def report(self):
        logging.info("Ingestion Analytics Report")
        logging.info("Received Messages:")
        for topic, count in self.received_messages_count.items():
            logging.info(f"  Topic: {topic}, Count: {count}")
        logging.info("Sent Messages:")
        for stream, count in self.sent_messages_count.items():
            logging.info(f"  Stream: {stream}, Count: {count}")

        # Clearing out message counts after printing the report
        self.received_messages_count.clear()
        self.sent_messages_count.clear()

    def start_reporting_thread(self):
        self.reporting_thread = threading.Thread(target=self.reporting_loop)
        self.reporting_thread.daemon = True
        self.reporting_thread.start()

    def reporting_loop(self):
        while self.enabled:
            self.report()
            time.sleep(60)

    def stop_reporting_thread(self):
        self.enabled = False
        if self.reporting_thread:
            self.reporting_thread.join()
