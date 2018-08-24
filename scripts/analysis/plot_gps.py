#!/usr/bin/env python
import argparse
import scipy.io as sio
from gmplot import gmplot # pip install gmplot
import numpy as np
import pdb

''' Code to convert matfile to Google Map GPS HTML Plot '''
def plot(matfile, save_name):

	data = sio.loadmat(matfile)
	assert(data['mode'] != 'Sim')
	lats = np.ravel(data['lat']).tolist()
	lons = np.ravel(data['lon']).tolist()

	if 't_en' in data.keys():
		t_en = data['t_en']
		t_data = data['t']
		st_ind = np.argmin(np.square(t_data - t_en))
	else:
		st_ind = 0

	gmap = gmplot.GoogleMapPlotter(lats[0], lons[0], 18) # lat, lon, zoom_level
	gmap.scatter([lats[st_ind]], [lons[st_ind]], color="b", size=10, symbol='x') # starting point in blue
	gmap.scatter([lats[-1]],[lons[-1]], color="g", size=10, symbol='x') # ending point in green	
	gmap.plot(lats, lons, color="r", size=10, symbol='x', ls=None) # other points in red line
	gmap.draw(save_name)

if __name__=='__main__':
	parser = argparse.ArgumentParser('Plot GPS map from a processed matfile containing lat/lon history from a path following experiment.')
	parser.add_argument('-f', '--file', type=str, required=True, help='Matfile location.')
	parser.add_argument('-o', '--out', type=str, required=True, help='Map HTML save location.')
	args = parser.parse_args()
	plot(args.file, args.out)
