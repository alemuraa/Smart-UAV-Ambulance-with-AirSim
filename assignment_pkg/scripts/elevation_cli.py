#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from assignment_pkg.srv import Elevation_srv, Elevation_srvRequest, Elevation_srvResponse

import geopandas as gpd
import fiona
from shapely.geometry import Point
import os

from AirSimWrapper import AirSimWrapper
from Logger import Logger


class ElevationClient:
	def __init__(self, node_name):
		self._node_name = node_name
		rospy.init_node(self._node_name)

		# Init logger
		self.__logger = Logger(self._node_name)
		self.__logger.loginfo("Node started.")

		# Load parameters
		self.__load_params()

		# Initialize AirSim wrapper
		self.airsim = AirSimWrapper(self.host, self.port)

		# Wait for elevation service
		rospy.wait_for_service('elevation_srv')
		self.__logger.loginfo("Elevation service ready.")

		# Error code if elevation can't be retreived
		self.err_code = -5

		# Publishers
		self.elevation_pub = rospy.Publisher('/elevation_limit', Float64, queue_size=1)

		# Initialize timer to check GPS data
		rospy.Timer(rospy.Duration(5), self.check_gps)


	def __load_params(self):
		self.host = rospy.get_param('~host')
		self.port = rospy.get_param('~port')

		if not self.host or not self.port:
			self.__logger.logerr("Host and port parameters are required.")
			rospy.signal_shutdown("Host and port parameters are required.")


	def load_kml(self, filename):
		path_to_file = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), 'maps', filename)
		fiona.supported_drivers['KML'] = 'rw'
		self.restriction_areas = gpd.read_file(path_to_file, driver='KML')
		self.__logger.loginfo('KML file loaded.')


	def check_gps(self, event):
		gps_data = self.airsim.get_gps_data()

		# Check if the drone is within a flight restriction area, and if so, get the current altitude limit in that point
		is_in_area = self.restriction_areas.contains(Point(gps_data.gnss.geo_point.longitude, gps_data.gnss.geo_point.latitude))

		if is_in_area.any():
			self.__logger.logwarn('Drone IN restriction area')
			area_limit = int(self.restriction_areas[is_in_area]['Description'].iloc[0])
		else:
			area_limit = 120 # Max area limit if no limit is defined

		# Call service to get elevation for the given coordinates
		try:
			elevation_cli = rospy.ServiceProxy('elevation_srv', Elevation_srv)
			resp = elevation_cli(float(gps_data.gnss.geo_point.latitude), float(gps_data.gnss.geo_point.longitude))
		except rospy.ServiceException as e:
			self.__logger.logerr(f'Service call failed: {e}')

		# If no elevation has been received, set default elevation to 0
		if resp.elevation == self.err_code:
			resp.elevation = 0
			self.__logger.logerr('Cannot retrieve correct elevation, setting default.')

		curr_limit = resp.elevation + area_limit
		self.elevation_pub.publish(curr_limit)


def main():
	elevation_cli = ElevationClient('elevation_cli')
	elevation_cli.load_kml('flight_restriction_areas.kml')

	rospy.spin()


if __name__ == '__main__':
	main()
