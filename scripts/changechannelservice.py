import rospy
from wiros_csi_node.srv import ConfigureCSI
import time

def configure_csi_service(chan, bw, mac_filter):
	rospy.wait_for_service('/csi_server/configure_csi')
	try:
		#create a handle for calling the service
		configure_csi = rospy.ServiceProxy('/csi_server/configure_csi', ConfigureCSI)

		#Call service with arguments
		resp = configure_csi(chan=chan, bw=bw, mac_filter=mac_filter)
		rospy.loginfo(f"Service called with chan: {chan}, bandwidth: {bw}, mac filter: {mac_filter}")
	except rospy.ServiceException as e:
		rospy.logerr(f"Service call failed: {e}")
	
def main():
	rospy.init_node('csi_configure', anonymous=True)

	#Parameters, I dunno what to put here, should it cycle  through these paramaters or pass in a list of channels to go through?
	config_params = [
		{'chan': 32, 'bw': 20, 'mac_filter': "*:*:*:*:*:*"},
		{'chan': 161, 'bw': 20, 'mac_filter': "*:*:*:*:*:*"},
		{'chan': 165, 'bw': 20, 'mac_filter': "*:*:*:*:*:*"},
	]
	channel_list = [32, 36, 40, 44, 48, 52, 56, 60, 64, 68, 96, 100, 104, 108, 112, 116, 120, 124, 128, 132, 136, 140, 144, 149, 153, 157, 161, 165, 169, 173, 177]

	idx = 0
	rate = rospy.Rate(0.2)
	while not rospy.is_shutdown():
		params = config_params[idx]
		channel_curr = channel_list[idx]
		configure_csi_service(channel_curr, 20, "*:*:*:*:*:*")

		#Wait for 5 minutes 
		rospy.sleep(5)
		#try:
		#	rate.sleep()
		#except rospy.ROSInterruptException:
		#	rospy.loginfo("Shutting down due to Keyboard Interrupt")
		#	break

		idx = (idx+1) % len(channel_list)

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
