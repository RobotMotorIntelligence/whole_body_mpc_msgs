import rospy
from riccati_controller.msg import RiccatiControllerReference
from whole_body_state_msgs.whole_body_trajectory_publisher import WholeBodyStateInterface
from .riccati_gain_interface import RiccatiGainInterface


class ReferencePublisher():
    def __init__(self, topic, model):
        # Initializing the publisher
        self._pub = rospy.Publisher(topic, RiccatiControllerReference, queue_size=10)
        self._wb_iface = WholeBodyStateInterface(model)
        self._rg_iface = RiccatiGainInterface(2 * model.nv, model.njoints - 2)
        self._msg = RiccatiControllerReference()
        self._msg.header.frame_id = "world"

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
