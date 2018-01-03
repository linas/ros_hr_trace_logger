#!/usr/bin/env python

import datetime as dt
import traceback
import os
import rospy
from chatbot.db import get_mongodb, MongoDB
from hr_msgs.msg import TTS
from hr_msgs.msg import EmotionState
from hr_msgs.msg import SetGesture
from ros_face_recognition.msg import Face, Faces
import logging

logger = logging.getLogger('hr.ros_mongo.mongo_logger')
ROBOT_NAME = os.environ.get('NAME', 'default')

class MongoLogger(object):
    def __init__(self):
        try:
            self.mongodb = get_mongodb()
        except Exception as ex:
            self.mongodb = MongoDB()
            logger.error(ex)
        self.run_id = rospy.get_param('/run_id', '')
        rospy.Subscriber('/blender_api/set_emotion_state', EmotionState, self.log_emotion)
        rospy.Subscriber('/blender_api/set_gesture', SetGesture, self.log_gesture)
        rospy.Subscriber('face_recognizer/faces', Faces, self.log_faces)

    def _log(self, collection, record):
        try:
            result = collection.insert_one(record)
            logger.info("Added record to mongodb")
        except Exception as ex:
            self.mongodb.client = None
            logger.error(traceback.format_exc())
            logger.warn("Deactivate mongodb")

    def log_emotion(self, msg):
        record = {
            'Datetime': dt.datetime.now(),
            'RunID': self.run_id,
            'Name': msg.name,
            'Magnitude': msg.magnitude,
            'Duration': msg.duration.nsecs,
        }
        if self.mongodb.client is not None:
            collection = self.mongodb.client[self.mongodb.dbname][ROBOT_NAME]['blender']['emotion_state']
            self._log(collection, record)

    def log_gesture(self, msg):
        record = {
            'Datetime': dt.datetime.now(),
            'RunID': self.run_id,
            'Name': msg.name,
            'Repeat': msg.repeat,
            'Speed': msg.speed,
            'Magnitude': msg.magnitude,
        }
        if self.mongodb.client is not None:
            collection = self.mongodb.client[self.mongodb.dbname][ROBOT_NAME]['blender']['gesture']
            self._log(collection, record)

    def log_faces(self, msg):
        time = dt.datetime.now()
        for face in msg.faces:
            record = {
                'Datetime': time,
                'RunID': self.run_id,
                'FaceID': face.faceid,
                'Left': face.left,
                'Top': face.top,
                'Right': face.right,
                'Bottom': face.bottom,
                'Confidence': face.confidence,
            }
            if self.mongodb.client is not None:
                collection = self.mongodb.client[self.mongodb.dbname][ROBOT_NAME]['face_recognizer']['faces']
                sharecollection = self.mongodb.get_share_collection()
                self._log(collection, record)
                self._log(sharecollection, {'node': 'face_recognizer', 'msg': record})

if __name__ == '__main__':
    rospy.init_node('mongo_logger')
    MongoLogger()
    rospy.spin()
