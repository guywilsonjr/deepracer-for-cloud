import os

import boto3


TOPIC_ARN = 'arn:aws:sns:us-west-1:936272581790:DeepRacerService-DeepRacerTopic44DD098F-xVgWiMQll1Dv'


class Emitter:
    def __init__(self):
        self.session = boto3.Session(
            aws_access_key_id=os.environ['SNS_ACCESS_KEY_ID'],
            aws_secret_access_key=os.environ['SNS_SECRET_ACCESS_KEY']
        )
        self.sns = self.session.client('sns', region_name='us-west-1')

    def emit(self, message):
        print("FAKE: Emitting message of size: ", len(message))
        return
        self.sns.publish(
            TopicArn=TOPIC_ARN,
            Message=message
        )
