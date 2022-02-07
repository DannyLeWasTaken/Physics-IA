# A simple library dedicated to creating random values suitable for the IA program.

import random

unseenRandoms = {}

def ModifyRandomPercentage(baseNumber, percent):
	intPercent = percent / 100
	return baseNumber + random.randint((baseNumber * percent) * 10, -(baseNumber * percent) * 10) / 10

def GetRandomData(baseNumber, category, percent, unscreenPercent):
	# Random that based off of controlled and unseen variables
	if not category in unseenRandoms:
		unseenRandoms[category] = unscreenPercent

	return ModifyRandomPercentage(ModifyRandomPercentage(baseNumber, unscreenPercent), percent)

def GetRandomDataAutoUnsceen(baseNumber, category, percent, min, max):
	# Same idea as before but with auto-generated unseen variables
	return GetRandomData(baseNumber, category, percent, (random.randint(min, max)/10))