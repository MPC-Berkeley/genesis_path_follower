#!/usr/bin/env python
import argparse
from anim_utils.anim_track import AnimateTrack
from anim_utils.anim_vehicle import AnimateVehicle

if __name__=='__main__':
	parser = argparse.ArgumentParser('Convert bag file and reference mat file to a video.')
	parser.add_argument('--bag', type=str, required=True, help='Bag file to animate.')
	parser.add_argument('--mat', type=str, required=True, help='Matfile with the reference path.')
	parser.add_argument('--vid', type=str, required=True, help='Video output filename (mp4).')
	parser.add_argument('--type', type=str, choices=['track', 'vehicle'], help='Plot full track or zoomed-in view of vehicle.')
	args = parser.parse_args()

	if args.type == 'track':
		AnimateTrack(args.bag, args.mat, args.vid)
	elif args.type == 'vehicle':
		AnimateVehicle(args.bag, args.mat, args.vid)
	else:
		raise ValueError("Invalid video type!  Accepted types are <track> or <vehicle>.")
