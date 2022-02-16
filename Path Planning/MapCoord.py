import cv2
import numpy as np
import pickle

# Loads image and resze
img = cv2.imread('Maps/MapImg.png')

scale_percent = 20 # percent of original size
width = int(img.shape[1] * scale_percent / 100)
height = int(img.shape[0] * scale_percent / 100)
dim = (width, height)

img = cv2.resize(img, dim, interpolation = cv2.INTER_AREA) 

# Split RGB img to three channels and initialise dictionary
(B, G, R) = cv2.split(img)
coord = { "green": [], "red": []}


# ----- Find Green Coordinates ----- #


# Most of the green coordinates are black in colour in the red channel
# We can apply thresholding to find the green coordinates
(T,thresh) = cv2.threshold(R,10,255,cv2.THRESH_BINARY_INV)
cv2.imshow("Green coordinates", thresh)

# Once we get a binary image, we use findContour to get an outline of
# the white circle
(cnts, heir) = cv2.findContours(thresh.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
for c in cnts:
    clist = np.array(c)
    clist = clist.reshape(len(clist),2)
    xlist = clist[:,0] 
    ylist = clist[:,1]
    x_coord = int(np.mean(xlist))
    y_coord = int(np.mean(ylist))

    coord['green'].append((x_coord,y_coord))

    text = '(' + str(x_coord) + ',' + str(y_coord) + ')'
    cv2.putText(img, text, (x_coord - 30, y_coord - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.4,
	(0,0,0), 2) 



# ---- Find Red Coordinates ---- #


# Get Red coordinates. Most of the red coordinates are white in colour
# in the green channel
(T2,thresh2) = cv2.threshold(G,10,255,cv2.THRESH_BINARY_INV)

# Once we get a binary image, we use findContour to get an outline of
# the white circle
(cnts, heir) = cv2.findContours(thresh2.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
for c in cnts:
    clist = np.array(c)
    clist = clist.reshape(len(clist),2)
    xlist = clist[:,0] 
    ylist = clist[:,1]
    x_coord = int(np.mean(xlist))
    y_coord = int(np.mean(ylist))

    coord['red'].append((x_coord,y_coord))

    text = '(' + str(x_coord) + ',' + str(y_coord) + ')'
    cv2.putText(img, text, (x_coord - 30, y_coord - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.4,
	(0,0,0), 2) 


# Save dictionary
f = open("OutputFiles/Coordinates.pkl","wb")
pickle.dump(coord,f)
f.close()

# Display image
cv2.imshow("Image with coordinates",img)
cv2.waitKey(0)
cv2.destroyAllWindows