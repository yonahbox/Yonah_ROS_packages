import rospy
from despatcher.msg import LinkMessage

uuid = 0

def ack_converter(data, state):
    if data.uuid == 0:
        return None
    else:
        message = LinkMessage()
        message.uuid = data.uuid
        message.id = 0 #@TODO change the message id to the supposed value
        if state == 0:
            message.data = "pending"
        elif state == 1:
            message.data = "single tick"
        elif state == 2:
            message.data = "double tick"
    return message

def increment():
    global uuid
    uuid += 1
    if uuid > 255:
        uuid -= 255
    rospy.logwarn("increment is called, UUID " + str(uuid))
    return uuid