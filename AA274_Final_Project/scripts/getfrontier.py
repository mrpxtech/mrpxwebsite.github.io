#!/usr/bin/env python


#--------Include modules---------------
from copy import copy
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np
from nav_msgs.msg import OccupancyGrid
import os
import rospy
from skimage import io
from skimage.morphology import square
from skimage.morphology import dilation
from scipy.misc import imsave
from skimage.morphology import closing
from skimage import measure
from skimage.morphology import label
from skimage.measure import regionprops
from skimage.color import label2rgb
import cv2




#-----------------------------------------------------

def getfrontier(MapData):
    data= MapData.data #image origin starts at the top left
    w = MapData.info.width # x pixel value
    h = MapData.info.height # y pixel values
    resolution = MapData.info.resolution
    Xstartx = MapData.info.origin.position.x
    Xstarty = MapData.info.origin.position.y


    img = np.zeros((h, w), np.uint8)
    
    for i in range(0,h): # for all x pixel values
        for j in range(0,w): # for all y pixel values
            if data[i*w+j]==100: #unoccupied
                img[i,j]=255
            elif data[i*w+j]==0: #occupied
                img[i,j]=0
            elif data[i*w+j]==-1: #unmeasured
                img[i,j]=205

    #imsave('map.png', img)
    #plt.clf()
    #plt.imshow(img, cmap=plt.get_cmap('Greys')) 
    #plt.pause(0.000001)
   
    img_dilated = dilation(img, square(3)) # dilate image

    img_closing = closing(img_dilated, square(7)) #apply closing (expands white)


    mask = img_closing == 0 # is black

    counter = 0

    # ----------------get frontier points------------------
    frontier_points = [] #tuples of coordinates
    for i in range(mask.shape[0]): #for each row
        for j in range(mask.shape[1]): #for each column
            node_neighbors = neighbors((i,j))# getnode neighbors
            
            if mask[i,j] == True:  #if node is black pixel
                #print(node_neighbors)
                gray_edges = 0 #start gray edge counter
                for index in node_neighbors: #check all the nearest neighbors for gray
                    #print(index)
                    if (index[0]==mask.shape[0] or index[1]==mask.shape[1] or index[0] == 0 or index[1]==0 ):
                        continue
                    elif img_closing[index[0],index[1]] == 205: # if there is a gray neighbor
                        gray_edges = gray_edges+1
                        
                
                #print(gray_edges)
                counter = counter +1
                if gray_edges>2:
                    frontier_points.append((i,j))
    #----------- end get frontier points --------------------

    img_updated = np.zeros(img_closing.shape,dtype=np.uint8)


    for index in frontier_points:
        #print(img_updated[index[0],index[1]] )
        img_updated[index[0],index[1]] = 255
        #print(img_updated[index[0],index[1]] )

  

    #plt.imshow(img_updated,cmap=plt.get_cmap('Greys'), vmin=0, vmax=255)

    frontiers_filtered = dilation(img_updated, square(4)) # delate again for segmentation

    # ------------------label and get centroids ----------------
    label_image = label(frontiers_filtered)
    print("frontier_Filtered image shape",frontiers_filtered.shape)
    props = regionprops(label_image)
    props[0]['Centroid'] # centroid of first labelled object

    #image = frontiers_filtered
    #cv2.imshow('cv_img', label_image)
    #cv2.waitKey(2)
    #image_label_overlay = label2rgb(label_image, image=image,alpha=0.3,bg_label=0,bg_color=(0, 0, 0))
    #label_h_maxima, img, alpha=0.7, bg_label=0,
    #                        bg_color=None, colors=[(1, 0, 0)])

    #fig, ax = plt.subplots(figsize=(10, 6))
    
    
    
    #ax.imshow(image_label_overlay)
    #print("image overlay shape",image_label_overlay.shape)
    #plt.pause(0.05)
    #cv2.imshow('cv_img', image_label_overlay)
    #cv2.waitKey(2)

    centroids = []
    k = 0
    # filter based on region area
    for region in props:
        # take regions with large enough areas
        if region.area >= 25:
            print("props_Centroid",props[k]['Centroid'])
            centroids.append( ((props[k]['Centroid'][1]),(props[k]['Centroid'][0])) ) 
            # draw rectangle around segmented coins
            minr, minc, maxr, maxc = region.bbox
            rect = mpatches.Rectangle((minc, minr), maxc - minc, maxr - minr,
                                    fill=False, edgecolor='red', linewidth=2)
            #ax.add_patch(rect)
        k = k+1

    #ax.set_axis_off()
    #print("SHSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSShape",ax.shape)
    #plt.tight_layout()



    '''# Construct RGB version of grey-level image
    img_color = np.dstack((img, img, img))
    # Convert the input image and color mask to Hue Saturation Value (HSV)
    # colorspace
    img_hsv = color.rgb2hsv(img_color)
    color_mask_hsv = color.rgb2hsv(color_mask)

    # Replace the hue and saturation of the original image
    # with that of the color mask
    img_hsv[..., 0] = color_mask_hsv[..., 0]
    img_hsv[..., 1] = color_mask_hsv[..., 1] * alpha

    img_masked = color.hsv2rgb(img_hsv)'''


    plt.show()
   


    #-------------------------------- end get centroids-------------------------
    
    return centroids







def neighbors(x):
    node_neighbors = []
    resolution = 1
    for i in range(-1,2):
        for j in range(-1,2):
            if ( i==0 and j==0 ):
                continue
            else:
                node_neighbors.append((i*resolution+x[0],j*resolution+x[1]))
                
    return node_neighbors
