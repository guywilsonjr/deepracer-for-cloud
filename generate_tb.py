import tensorflow as tf
from tensorflow.compat.v1 import Graph, GraphDef, summary
from tensorflow.python.client import session
from tensorflow.python.platform import gfile


MODEL_FILENAME = "/home/guy/Downloads/model_98.pb"  # change me
LOGDIR = "MY_OUTPUT_LOGDIR/"

with session.Session() as sess:
    with gfile.FastGFile(MODEL_FILENAME, 'rb') as f:
        graph_def = GraphDef()
        graph_def.ParseFromString(f.read())
        g_in = tf.import_graph_def(graph_def)

with Graph().as_default():
    train_writer = summary.FileWriter(LOGDIR)
    train_writer.add_graph(sess.graph)