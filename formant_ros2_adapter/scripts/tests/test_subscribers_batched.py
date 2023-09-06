import unittest
from formant.sdk.agent.v1 import Client
from components.subscriber.batched_ingester import (
    BatchIngester,
)  # Replace with the actual import
from queue import Empty
import time


class TestBatchIngester(unittest.TestCase):
    def setUp(self):
        self.fclient = Client()
        self.ingester = BatchIngester(self.fclient, ingest_interval=1, num_threads=1)

    def test_message_ingest(self):
        self.ingester.ingest("msg", str, "stream1", "topic", 12345, {})

        # Checking if the message is added to the correct stream queue
        self.assertEqual(self.ingester._stream_queues["stream1"].qsize(), 1)

    def test_queue_size_limit(self):
        for i in range(15):  # Adding 15 messages
            self.ingester.ingest(f"msg{i}", str, "stream1", "topic", 12345, {})

        # Ingest once
        self.ingester._ingest_once()

        # Checking the remaining queue size (should be 15 - MAX_INGEST_SIZE)
        self.assertEqual(self.ingester._stream_queues["stream1"].qsize(), 5)

    def test_queue_flush(self):
        for i in range(5):  # Adding 5 messages
            self.ingester.ingest(f"msg{i}", str, "stream1", "topic", 12345, {})

        # Wait for the ingest interval to pass (plus a small buffer)
        time.sleep(1.2)

        # The queue should be empty after one ingest interval
        self.assertRaises(Empty, self.ingester._stream_queues["stream1"].get_nowait)


if __name__ == "__main__":
    unittest.main()
