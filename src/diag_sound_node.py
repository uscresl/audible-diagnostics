#!/usr/bin/python

import rospy
import re
from sound_play.libsoundplay import SoundClient, SoundRequest
from diagnostic_msgs.msg import DiagnosticArray

class AudibleDiagnosticsNode:
    def __init__(self, topic, include_names, exclude_names, play_rate=10):
        self.sound_client = SoundClient()
        self.include_re = [re.compile("^%s.*" % t) for t in include_names]
        self.exclude_re = [re.compile("^%s.*" % t) for t in exclude_names]
        self.check_include = len(self.include_re) != 0
        self.check_exclude = len(self.exclude_re) != 0
        self.play_rate = play_rate
        self.subscriber = rospy.Subscriber(topic, DiagnosticArray, self.callback)
        self.latest_status = None

    def callback(self, diagnostic_array):
        warnings, errors, stale = 0, 0, 0
        statuses = list()
        for status in diagnostic_array.status:
            included = False
            if self.check_include:
                for regex in self.include_re:
                    if regex.match(status.name):
                        included = True
                        statuses.append(status)
                        break
            elif not included and self.check_exclude:
                exclude = False
                for regex in self.exclude_re:
                    if regex.match(status.name) is not None:
                        exclude = True
                        break
                if not exclude:
                    statuses.append(status)
            else:
                statuses = diagnostic_array.status
        for s in statuses:
            if s.level == s.WARN:
                warnings += 1
            elif s.level == s.ERROR:
                errors += 1
            elif s.level == s.STALE:
                stale += 1
        self.latest_status = (warnings, errors, stale)

    def spin(self):
        r = rospy.Rate(self.play_rate)
        while not rospy.is_shutdown():
            rospy.logwarn("(W, E, S):%s" % str(self.latest_status))
            r.sleep()
            if self.latest_status and any(self.latest_status):
                if cool_mode:
                    self.sound_client.play(SoundRequest.NEEDS_PLUGGING_BADLY)
                else:
                    self.sound_client.say('%s warnings, %s errors, %s stale' % self.latest_status)
                


if __name__ == "__main__":
    global cool_mode
    rospy.init_node('diagnostic_sound_play')

    topic_settings = rospy.get_param('~topic_settings', dict())
    exclude_names = topic_settings.get('exclude', list())
    include_names = topic_settings.get('include', list())
    play_rate = rospy.get_param('~play_rate', 0.1)
    diagnostic_topic = rospy.get_param('~diagnostic_topic', '/diagnostics_agg')
    cool_mode = rospy.get_param('~cool_mode', False)
    d = AudibleDiagnosticsNode(diagnostic_topic, include_names, exclude_names, play_rate)
    d.spin()
