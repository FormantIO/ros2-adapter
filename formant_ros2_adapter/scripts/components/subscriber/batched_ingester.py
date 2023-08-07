from .ingester import Ingester
from formant.sdk.agent.v1 import Client
from queue import LifoQueue
from typing import Dict


MAX_INGEST_SIZE = 10


class Message:
    def __init__(self, msg, msg_type: type, topic: str, msg_timestamp: int, tags: Dict):
        self.msg = msg
        self.msg_type = msg_type
        self.topic = topic
        self.msg_timestamp = msg_timestamp
        self.tags = tags


class BatchIngester(Ingester):
    def __init__(self, _fclient: Client):
        super(BatchIngester, self).__init__(_fclient)
        self._stream_queues: Dict[str, LifoQueue[Message]] = {}
        self._ingest_interval = 1

        # Basically 1 or more threads should be running the flush function

    def batch_ingest(
        self,
        msg,
        msg_type: type,
        formant_stream: str,
        topic: str,
        msg_timestamp: int,
        tags: Dict,
    ):
        message = Message(msg, msg_type, topic, msg_timestamp, tags)
        self._stream_queues[formant_stream].put(message)

    def _flush_once(self):

        for stream, queue in self._stream_queues.items():
            ingest_size = min(len(queue), MAX_INGEST_SIZE)
            for _ in range(ingest_size):
                top_message = queue.get()
                self.ingest(
                    top_message.msg,
                    top_message.msg_type,
                    stream,
                    top_message.topic,
                    top_message.msg_timestamp,
                    top_message.tags,
                )
