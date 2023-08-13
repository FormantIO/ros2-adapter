from .base_ingester import BaseIngester
from formant.protos.agent.v1 import agent_pb2
from formant.protos.model.v1 import datapoint_pb2
from formant.sdk.agent.v1 import Client
from queue import LifoQueue
from typing import Dict, List
import threading
import time

MAX_INGEST_SIZE = 10


class BatchIngester(BaseIngester):
    def __init__(
        self, _fclient: Client, ingest_interval: int = 30, num_threads: int = 2
    ):
        super(BatchIngester, self).__init__(_fclient)
        self._stream_queues: Dict[str, LifoQueue] = {}
        self._ingest_interval = ingest_interval
        self._num_threads = num_threads
        self._threads: List[threading.Thread] = []
        self._terminate_flag = False

        self._start()

    def ingest(
        self,
        msg,
        msg_type: type,
        formant_stream: str,
        topic: str,
        msg_timestamp: int,
        tags: Dict,
    ):
        message = self.prepare(
            msg, msg_type, formant_stream, topic, msg_timestamp, tags
        )
        has_stream = formant_stream in self._stream_queues
        if not has_stream:
            self._stream_queues[formant_stream] = LifoQueue()

        self._stream_queues[formant_stream].put(message)

    def _ingest_once(self):

        for _, queue in self._stream_queues.items():
            ingest_size = min(queue.qsize(), MAX_INGEST_SIZE)
            datapoints = [queue.get() for _ in range(ingest_size)]

            self._fclient.post_data_multi(datapoints)

    def _ingest_continually(self):
        while not self._terminate_flag:
            self._ingest_once()
            time.sleep(self._ingest_interval)

    def _start(self):
        self._terminate_flag = False
        for i in range(self._num_threads):
            self._threads.append(
                threading.Thread(
                    target=self._ingest_continually,
                    daemon=True,
                )
            )
            self._threads[i].start()

    def terminate(self):
        self._terminate_flag = True
