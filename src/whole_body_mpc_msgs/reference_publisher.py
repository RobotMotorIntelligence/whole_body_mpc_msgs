import rospy
from whole_body_mpc_msgs.msg import WholeBodyMpcReference
from whole_body_state_msgs.whole_body_trajectory_publisher import WholeBodyStateInterface
from .state_feedback_gain_interface import StateFeedbackGainInterface


class ReferencePublisher():
    def __init__(self, topic, model, frame_id="world", queue_size=10):
        # Initializing the publisher
        self._pub = rospy.Publisher(topic, WholeBodyMpcReference, queue_size=queue_size)
        self._wb_iface = WholeBodyStateInterface(model, frame_id)
        self._rg_iface = StateFeedbackGainInterface(2 * model.nv, model.njoints - 2)
        self._msg = WholeBodyMpcReference()
        self._msg.header.frame_id = frame_id

    def publish(self, ts, qs, vs, us, ps=dict(), pds=dict(), fs=dict(), ss=dict(), Ks=None):
        self._msg.header.stamp = rospy.Time(ts[0])
        # Check that the length of the lists are consistent
        if len(ts) is not len(qs):
            print("Couldn't publish the message since the length of the qs list is not consistent")
            return
        if vs is not None:
            if len(ts) is not len(vs):
                print("Couldn't publish the message since the length of the vs list is not consistent")
                return
        if us is not None:
            if len(ts) is not len(us):
                print("Couldn't publish the message since the length of the us list is not consistent")
                return
        if ps is not None:
            if len(ts) is not len(ps):
                print("Couldn't publish the message since the length of the ps list is not consistent")
                return
        if pds is not None:
            if len(ts) is not len(pds):
                print("Couldn't publish the message since the length of the pds list is not consistent")
                return
        if fs is not None:
            if len(ts) is not len(fs):
                print("Couldn't publish the message since the length of the fs list is not consistent")
                return
        if ss is not None:
            if len(ts) is not len(ss):
                print("Couldn't publish the message since the length of the ss list is not consistent")
                return

        # Define the whole-body reference
        self._msg.reference = []
        self._msg.gain = []
        for i in range(len(ts)):
            self._msg.reference.append(
                self._wb_iface.writeToMessage(ts[i], qs[i], vs[i], us[i], ps[i], pds[i], fs[i], ss[i]))
            self._msg.gain.append(self._rg_iface.writeToMessage(Ks[i]))
        self._pub.publish(self._msg)
