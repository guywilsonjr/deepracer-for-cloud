import json
import os
import boto3
import random


local_profile = os.environ['DR_LOCAL_S3_PROFILE']
s3_bucket_name = os.environ['DR_LOCAL_S3_BUCKET']
world_name = os.environ['DR_WORLD_NAME']
best_model_metric = os.environ['DR_TRAIN_BEST_MODEL_METRIC']
model_prefix = os.environ['DR_LOCAL_S3_MODEL_PREFIX']
SIMULATION_ID = random.SystemRandom().randint(0, 999999)
with open('.simulation', 'w') as f:
    f.write(str(SIMULATION_ID))




minio_session = boto3.Session(profile_name=local_profile)
s3 = minio_session.client('s3', endpoint_url='http://localhost:9000')

hyperparameters_key = 'custom_files/hyperparameters.json'
model_metadata_key = 'custom_files/model_metadata.json'
hyperparameters_txt = s3.get_object(Bucket=s3_bucket_name, Key=hyperparameters_key)['Body'].read().decode()
model_metadata_txt = s3.get_object(Bucket=s3_bucket_name, Key=model_metadata_key)['Body'].read().decode()

hyperparameters = json.loads(hyperparameters_txt)
model_metadata = json.loads(model_metadata_txt)
message_type = 'SIMULATION_START'



sns = boto3.client('sns')
sns_topic_arn = os.environ['DR_SNS_TOPIC_ARN']
sns.publish(
    TopicArn=sns_topic_arn,
    Message=json.dumps({
        'message_type': message_type,
        'sim_id': SIMULATION_ID,
        'hyperparameters': hyperparameters,
        'model_metadata': model_metadata,
        'world_name': world_name,
        'best_model_metric': best_model_metric,
        'model_prefix': model_prefix
    })
)