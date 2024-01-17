import threading
import os
import time
from datetime import datetime

ANALYTICS_INTERVAL = 60


class IngestionAnalytics:
    def __init__(self):
        self.received_messages_count = {}
        self.sent_messages_count = {}
        self._configure_settings()
        if self.analytics_enabled:
            self._start_reporting_thread()

    def _configure_settings(self):
        self.analytics_enabled = (
            os.getenv("ANALYTICS_INGESTION", "false").lower() == "true"
        )
        self.file_logging_enabled = bool(os.getenv("ANALYTICS_FILE_PATH"))
        self.console_logging_enabled = (
            os.getenv("ANALYTICS_CONSOLE_PRINT", "false").lower() == "true"
        )
        self.log_file_path = os.getenv("ANALYTICS_FILE_PATH", "ingestion_analytics.log")

    def _start_reporting_thread(self):
        self.reporting_thread = threading.Thread(target=self._reporting_loop)
        self.reporting_thread.daemon = True
        self.reporting_thread.start()

    def _reporting_loop(self):
        while True:
            self._generate_and_output_report()
            time.sleep(ANALYTICS_INTERVAL)

    def log_received_message(self, topic):
        self.received_messages_count[topic] = (
            self.received_messages_count.get(topic, 0) + 1
        )

    def log_sent_message(self, stream):
        self.sent_messages_count[stream] = self.sent_messages_count.get(stream, 0) + 1

    def _generate_and_output_report(self):
        report_data = self._generate_report()
        self._output_report(report_data)
        self._reset_message_counts()

    def _generate_report(self):
        current_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        report_lines = [
            f"Ingestion Analytics Report - {current_time}\n",
            "Received Messages:\n",
        ]
        for topic, count in self.received_messages_count.items():
            report_lines.append(f"  Topic: {topic}, Count: {count}\n")

        report_lines.append("Sent Messages:\n")
        for stream, count in self.sent_messages_count.items():
            report_lines.append(f"  Stream: {stream}, Count: {count}\n")

        return "".join(report_lines)

    def _output_report(self, report_data):
        if self.file_logging_enabled:
            with open(self.log_file_path, "a") as file:
                file.write(report_data)
        if self.console_logging_enabled:
            print(report_data)

    def _reset_message_counts(self):
        self.received_messages_count.clear()
        self.sent_messages_count.clear()

    def __del__(self):
        if self.reporting_thread and self.reporting_thread.is_alive():
            self.reporting_thread.join()
