import rospy
from riccati_controller.msg import RiccatiControllerReference
from state_msgs import whole_body_interface as wb_iface
from riccati_controller import riccati_gain_interface as rg_iface


class ReferencePublisher():
    def __init__(self, topic, model):
        # Initializing the publisher
        self.pub = rospy.Publisher(topic, RiccatiControllerReference, queue_size=10)
        self.wb_iface = wb_iface.WholeBodyStateInterface(model)
        self.rg_iface = rg_iface.RiccatiGainInterface(2 * model.nv, model.njoints - 2)
        self.msg = RiccatiControllerReference()
        self.msg.header.frame_id = "world"

    def publish(self, t, q, v, tau, p=dict(), pd=dict(), f=dict(), s=dict(), K=None):
        self.msg.header.stamp = rospy.Time(t)
        # Define the whole-body reference
        self.msg.reference = self.wb_iface.writeToMessage(t, q, v, tau, p, pd, f, s)
        self.msg.gain = self.rg_iface.writeToMessage(K)
        self.pub.publish(self.msg)
