import cv2
import numpy as np
from os import listdir
from os.path import isdir, join
from pickle import dump

categories = [f for f in listdir('./101_ObjectCategories') if isdir(join('./101_ObjectCategories',f)) and f != 'BACKGROUND_Google']

descriptor_name = 'SIFT'

detector = cv2.FeatureDetector_create(descriptor_name)
extractor = cv2.DescriptorExtractor_create(descriptor_name)

f = open('SIFT_features.pickle','wt')

cache = {}
for c_idx in range(len(categories)):
	print c_idx / float(len(categories))
	category = categories[c_idx]
	images = listdir(join('./101_ObjectCategories',category))
	# only worry about object categories that have at least 50 images
	if len(images) < 50:
		continue
	images = images[0:50]
	print category + " " + str(len(images))
	cache[category] = []
	for image in images:
		file_name = join(join('./101_ObjectCategories',category),image)
		im = cv2.imread(file_name)
		im_bw = cv2.cvtColor(im,cv2.COLOR_BGR2GRAY)
		kp = detector.detect(im_bw)
		dc, des = extractor.compute(im_bw,kp)
		# stash the descriptors in a dictionary for later processing
		cache[category].append(np.array(des,dtype=np.uint8))

dump(cache,f)
f.close()
