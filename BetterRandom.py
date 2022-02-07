# A simple library dedicated to creating random values suitable for the IA program.

import random

unseenRandoms = {}

def ModifyRandomPercentage(init, percent):
	intPercent = percent / 100
	return init + random.randint((init * percent) * 10, -(init * percent) * 10) / 10

def GetRandomData(init, category, percent, unscreenPercent):
	# Random that based off of unseen variables
	if not category in unseenRandoms:
		unseenRandoms[category] = unscreenPercent

	return ModifyRandomPercentage(ModifyRandomPercentage(init, unscreenPercent), percent)