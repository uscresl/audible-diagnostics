#!/usr/bin/python

import rospy
import re
from sound_play.libsoundplay import SoundClient, SoundRequest
from diagnostic_msgs.msg import DiagnosticArray

startup_audible = False
ok_rate = 0

class AudibleDiagnosticsNode:
    """Monitors diagnostics and plays audio cues to help easy monitoring"""
    def __init__(self, topic, include_names, exclude_names, play_rate=0.1):
        self.sound_client = SoundClient()
        self.include_re = [re.compile("^%s.*" % t) for t in include_names]
        self.exclude_re = [re.compile("^%s.*" % t) for t in exclude_names]
        self.check_include = len(self.include_re) != 0
        self.check_exclude = len(self.exclude_re) != 0
        self.play_rate = play_rate  # Frequency at which audio cues are made
        self.subscriber = rospy.Subscriber(topic, DiagnosticArray, self.callback)
        self.latest_status = None
        if ok_rate != 0:
            self.ok_timer = rospy.timer.Timer(rospy.Duration(1.0/ok_rate), self.ok_timer_callback)
        self.say_ok = False

    def callback(self, diagnostic_array):
        """Main callback for diagnostic messages. Updates status of warnings/errors"""
        warnings, errors, stale = 0, 0, 0
        statuses = list()  # List of statuses to check finally
        for status in diagnostic_array.status:
            included = False
            if self.check_include:  # If only few topics need monitoring
                for regex in self.include_re:
                    if regex.match(status.name):
                        included = True
                        statuses.append(status)
                        break
            elif not included and self.check_exclude:  # If there's a need to exclude a few
                exclude = False
                for regex in self.exclude_re:
                    if regex.match(status.name) is not None:
                        exclude = True
                        break
                if not exclude:
                    statuses.append(status)
            else:  # Defaults to all status messages
                statuses = diagnostic_array.status
        for s in statuses:
            if s.level == s.WARN:
                warnings += 1
            elif s.level == s.ERROR:
                errors += 1
            elif s.level == s.STALE:
                stale += 1
        self.latest_status = (warnings, errors, stale)

    def ok_timer_callback(self, *args):
        self.say_ok = True

    def spin(self):
        if startup_audible:
            self.sound_client.say('Audible diagnostics started')

        r = rospy.Rate(self.play_rate)
        while not rospy.is_shutdown():
            r.sleep()
            if self.latest_status is not None:  # Then we have received diagnostics
                if any(self.latest_status):
                    self.sound_client.say('%s warnings, %s errors, %s stale' % self.latest_status)
                elif self.say_ok:  # Play sound if everything is ok, once in a while
                    self.sound_client.say('Systems okay')
                self.latest_status = None
            else:  # Then we haven't received diagnostics in the last cycle
                self.sound_client.say('Warning, no diagnostics received')


if __name__ == "__main__":
    rospy.init_node('diagnostic_sound_play')

    topic_settings = rospy.get_param('~topic_settings', dict())
    exclude_names = topic_settings.get('exclude', list())
    include_names = topic_settings.get('include', list())
    play_rate = rospy.get_param('~play_rate', 0.1)
    diagnostic_topic = rospy.get_param('~diagnostic_topic', '/diagnostics_agg')
    startup_audible = rospy.get_param('~startup_audible', False)
    ok_rate = rospy.get_param('~ok_rate', 0)

    d = AudibleDiagnosticsNode(diagnostic_topic, include_names, exclude_names, play_rate)
    d.spin()
