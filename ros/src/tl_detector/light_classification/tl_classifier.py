import rospy
import cv2
import time
import numpy      as np
import tensorflow as tf
from   styx_msgs.msg            import TrafficLight
from   keras                    import backend as K
from   keras                    import layers
from   keras.models             import load_model
from   keras.applications.vgg16 import preprocess_input


class TLClassifier(object):

    def __init__(self):

        # https://github.com/tensorflow/tensorflow/issues/6698
        config = tf.ConfigProto()
        config.gpu_options.allow_growth = True
        sess = tf.Session(config=config)
        K.set_session(sess)
        self.model = load_model('../../../data_science/models/udacity_vgg_fine_tuning_combined.h5')

        # https://github.com/fchollet/keras/issues/3517
        self.get_output = K.function([self.model.layers[0].input, K.learning_phase()],
                                     [self.model.layers[-1].output])


    def get_classification(self, image):
        image = cv2.resize(image, (224,224), interpolation=cv2.INTER_NEAREST)
        image = np.expand_dims(image, axis=0)
        image = image.astype(dtype=np.float64, copy=False)
        image = preprocess_input(image)

        start_time = time.time()
        pred = self.get_output([image, 0])[0]

        # Get the index with max value that corresponds to the predicted state
        pred = np.argmax(pred)

        # Find class based on prediction index (need to confirm labels are correct order)
        if pred == 0:
            state = TrafficLight.GREEN

        elif pred == 1:
            state = TrafficLight.UNKNOWN # index 1 corresponds to no light

        elif pred == 2:
            state = TrafficLight.RED

        else:
            state = TrafficLight.YELLOW

        return state

