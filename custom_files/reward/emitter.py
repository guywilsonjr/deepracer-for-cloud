import copy
import os

import boto3

from typing import List


SNS_TOPIC_ARN = os.environ['SNS_TOPIC_ARN']
SNS_ACCESS_KEY_ID = os.environ['SNS_ACCESS_KEY_ID']
SNS_SECRET_ACCESS_KEY = os.environ['SNS_SECRET_ACCESS_KEY']
SNS_SESSION_TOKEN = os.environ['SNS_SESSION_TOKEN']


class Emitter:
    def __init__(self) -> None:
        self.messages = []

    def emit(self, message) -> None:
        self.messages.append(message)

    def get_messages(self) -> List[str]:
        return [msg for msg in self.messages]
