#!/usr/bin/env python

import rospy
import requests
from assignment_pkg.srv import Elevation_srv, Elevation_srvResponse

from Logger import Logger


class ElevationService:
	def __init__(self, node_name):
		self._node_name = node_name
		rospy.init_node(self._node_name)

		# Init logger
		self.__logger = Logger(self._node_name)
		self.__logger.loginfo("Node started.")

		# Error code if elevation can't be retreived
		self.err_code = -5

		# Init service
		rospy.Service('elevation_srv', Elevation_srv, self.get_elevation)	


	def get_elevation(self, req):
		# Make request to Open-elevation API
		api_url = 'https://api.open-elevation.com/api/v1/lookup?locations='
		url = api_url + str(req.latitude) + ',' + str(req.longitude)

		res_srv = Elevation_srvResponse()
		response = requests.get(url)

		if response.status_code == requests.codes.OK:
			data = response.json()
			if 'results' in data and len(data['results']) > 0:
				res_srv.elevation = float(data['results'][0]['elevation'])
			else:
				self.__logger.logerr('No results found for the provided coordinates.')
				res_srv.elevation = self.err_code
		else:
			self.__logger.logerr(f'Error occurred while fetching data: {response.status_code}')
			res_srv.elevation = self.err_code

		return res_srv


def main():
	ElevationService('elevation_service')
	rospy.spin()


if __name__ == '__main__':
    main()

