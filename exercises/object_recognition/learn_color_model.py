from pickle import load
import sklearn
import numpy as np
from sklearn.ensemble import RandomForestClassifier
from sklearn.linear_model import LogisticRegression
from sklearn.cross_validation import StratifiedKFold
from os import listdir
import cv2
from os.path import isdir, join

categories = [f for f in listdir('./101_ObjectCategories') if isdir(join('./101_ObjectCategories',f)) and f != 'BACKGROUND_Google']
cache = {}
for c_idx in range(len(categories)):
	category = categories[c_idx]
	images = listdir(join('./101_ObjectCategories',category))
	# only worry about object categories that have at least 50 images
	if len(images) < 50:
		continue
	images = images[0:50]
	print category
	cache[category] = []
	for image in images:
		file_name = join(join('./101_ObjectCategories',category),image)
		im = cv2.imread(file_name)
		features = np.mean(np.mean(im,axis=0),axis=0)
		# stash the features in a dictionary for later processing
		cache[category].append(np.array(features,dtype=np.uint8))

print categories

X = np.zeros((0,3))
y = np.zeros((0,1))

categories = cache.keys()
for i in range(len(categories)):
	for data in cache[categories[i]]:
		X = np.vstack((X,data))
		y = np.vstack((y,i*np.ones((1,1))))
y = np.ravel(y)

skf = StratifiedKFold(y,5)
category_accuracies = np.zeros((len(categories),5))

fold = 0
for train, test in skf:
	Xtest = X[test,:]
	ytest = y[test,:]
	model = LogisticRegression()
	model.fit(X[train,:],y[train,:])
	print model.score(Xtest,ytest)
	for i in range(len(categories)):
		category_accuracies[i,fold] = model.score(Xtest[ytest==i],ytest[ytest==i])
	fold += 1

# print out accurcies by category
print zip(categories,np.mean(category_accuracies,axis=1))