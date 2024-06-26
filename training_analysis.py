# Ignore deprecation warnings we have no power over
import warnings

from deepracer.logs import AnalysisUtils as au, DeepRacerLog, PlottingUtils as pu, S3FileHandler
from deepracer.tracks import Track, TrackIO


warnings.filterwarnings('ignore')
PREFIX='rl-deepracer-sagemaker-97'   # Name of the model, without trailing '/'
BUCKET='bucket'       # Bucket name is default 'bucket' when training locally
PROFILE='minio'          # The credentials profile in .aws - 'minio' for local training
S3_ENDPOINT_URL='http://minio:9000'
fh = S3FileHandler(bucket=BUCKET, prefix=PREFIX, profile=PROFILE, s3_endpoint_url=S3_ENDPOINT_URL)
log = DeepRacerLog(filehandler=fh)
log.load_training_trace()
df = log.dataframe()
simulation_agg = au.simulation_agg(df)
try:
    if df.nunique(axis=0)['worker'] > 1:
        print("Multiple workers have been detected, reloading data with grouping by unique_episode")
        simulation_agg = au.simulation_agg(df, secondgroup="unique_episode")
except:
    print("Multiple workers not detected, assuming 1 worker")

au.analyze_training_progress(simulation_agg, title='Training progress')
track_name = 'caecer_gp'
tu = TrackIO()
track: Track = tu.load_track(track_name)
pu.plot_track(df, track)
