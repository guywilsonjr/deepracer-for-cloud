import os

import boto3


SNS_TOPIC_ARN = os.environ['SNS_TOPIC_ARN']
SNS_ACCESS_KEY_ID = os.environ['SNS_ACCESS_KEY_ID']
SNS_SECRET_ACCESS_KEY = os.environ['SNS_SECRET_ACCESS_KEY']
SNS_SESSION_TOKEN = os.environ['SNS_SESSION_TOKEN']


class Emitter:
    def __init__(self) -> None:
        self.session = boto3.Session(
            aws_access_key_id=SNS_ACCESS_KEY_ID,
            aws_secret_access_key=SNS_SECRET_ACCESS_KEY,
            aws_session_token=SNS_SESSION_TOKEN
        )

        self.sns = self.session.client('sns', region_name='us-west-1')

    def emit(self, message) -> None:
        print("Emitting message of size: ", len(message))
        self.sns.publish(
            TopicArn=SNS_TOPIC_ARN,
            Message=message
        )
