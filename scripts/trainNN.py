#!/usr/bin/env python3

import os

os.environ["CUDA_DEVICE_ORDER"] = "PCI_BUS_ID"
os.environ["CUDA_VISIBLE_DEVICES"] = "0"

#import pycuda.autoinit

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import keras
from keras.models import Sequential
from keras.optimizers import Adam
from keras.layers import Convolution2D, Conv2D, MaxPool2D, Dropout, Flatten, Dense
from sklearn.utils import shuffle
from sklearn.model_selection import train_test_split
import cv2
import pandas as pd
import ntpath
import random

datadir = '/code/SpinakerV1Data/2023-01-19-d3-red1'
data = pd.read_csv(os.path.join(datadir, 'data.csv'))
pd.set_option('display.max_colwidth', None)
print(data.head())

## Historgram analysis
num_bins = 25
samples_per_bin = 100
hist, bins = np.histogram(data['omega'], num_bins)
center = (bins[:-1]+bins[1:]) * 0.5
plt.bar(center, hist, width=0.25)
plt.plot((np.min(data['omega']), np.max(data['omega'])), (samples_per_bin, samples_per_bin))
plt.show()
print(bins)
print(center)

## Flatten data
print('total date:', len(data))
remove_list = []
for j in range(num_bins):
  list_ = []
  for i in range(len(data['omega'])):
    if data['omega'][i] >= bins[j] and data['omega'][i] <= bins[j+1]:
      list_.append(i) 
  list_ = shuffle(list_)
  list_ = list_[samples_per_bin:]
  remove_list.extend(list_)

print('removed ', len(remove_list))
data.drop(data.index[remove_list], inplace=True)
print('remaning', len(data))

hist, _ = np.histogram(data['omega'], num_bins)
plt.bar(center, hist, width=0.25)
plt.plot((np.min(data['omega']), np.max(data['omega'])), (samples_per_bin, samples_per_bin))
plt.show()

print(data.iloc[1])
def load_img_stearing(datadir, df):
  image_path = []
  steering = []
  for i in range(len(data)):
    # Save path to image
    image_path.append(os.path.join(datadir, data.iloc[i].filename.strip()))
    # Save omega 
    steering.append(float( data.iloc[i].omega ))

  image_paths = np.array(image_path)
  steerings = np.array(steering)
  return image_paths, steerings                                           

image_paths, steerings = load_img_stearing(datadir, data)

X_train, X_valid, y_train, y_valid = train_test_split(image_paths, steerings, test_size=0.2, random_state=6)
print('Traning Samples: {}\nValid Samples: {}'.format(len(X_train), len(X_valid)))

def img_preprocess(img):
  img = mpimg.imread(img)
  img = img[220:500, :, :]
  img = cv2.cvtColor(img, cv2.COLOR_RGB2YUV)
  img = cv2.GaussianBlur(img, (3, 3), 0)
  img = cv2.resize(img, (200, 66))
  img = (img - np.min(img)) / (np.max(img) - np.min(img))
  return img

print(image_paths[100])

image = image_paths[100]
oryginal_image = mpimg.imread(image)
preprocessed_image = img_preprocess(image)

fig, axs = plt.subplots(1, 2, figsize=(15, 10))
fig.tight_layout()
axs[0].imshow(oryginal_image)
axs[0].set_title('Original Image')
axs[1].imshow(preprocessed_image)
axs[1].set_title('Preprocessed Image')

X_train = np.array(list(map(img_preprocess, X_train)))
X_valid = np.array(list(map(img_preprocess, X_valid)))

plt.imshow(X_train[random.randint(0, len(X_train)-1)])
plt.axis('off')
plt.show()
print(X_train.shape)

def nvidia_model():
 
  model = Sequential()
 
  model.add(Conv2D(24, kernel_size=(5,5), strides=(2,2), input_shape=(66,200,3),activation='relu'))
 
  model.add(Conv2D(36, kernel_size=(5,5), strides=(2,2), activation='elu'))
  model.add(Conv2D(48, kernel_size=(5,5), strides=(2,2), activation='elu'))
  model.add(Conv2D(64, kernel_size=(3,3), activation='elu'))
  model.add(Conv2D(64, kernel_size=(3,3), activation='elu'))

  model.add(Dropout(0.5))
 
  model.add(Flatten())
  model.add(Dense(100, activation='elu'))
  model.add(Dropout(0.5))
 
  model.add(Dense(50, activation='elu'))
  model.add(Dropout(0.5))

  model.add(Dense(10, activation ='elu'))
  model.add(Dropout(0.5))
  
  model.add(Dense(1))
 
  optimizer= Adam(lr=1e-3)
  model.compile(loss='mse', optimizer=optimizer)
 
  return model

model=nvidia_model()
print(model.summary())

history = model.fit(X_train, y_train, epochs=300, validation_data=(X_valid, y_valid), batch_size=50, verbose=1, shuffle=1)

plt.plot(history.history['loss'])
plt.plot(history.history['val_loss'])
plt.legend(['tranning', 'validation'])
plt.title('Loss')
plt.xlabel('Epoch')
plt.show()

model.save('../assets/nn_models/test11.h5')
