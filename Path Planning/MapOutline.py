import cv2
import pickle

# Loads image and resize image
img = cv2.imread('Maps/MapImg.png')
binMap = cv2.imread('Maps/BinaryMap.png')

scale_percent = 20 # percent of original size
width = int(img.shape[1] * scale_percent / 100)
height = int(img.shape[0] * scale_percent / 100)
dim = (width, height)

img = cv2.resize(img, dim, interpolation = cv2.INTER_AREA) 
binMap = cv2.resize(binMap, dim, interpolation = cv2.INTER_AREA) 

# Convert to grayscale and increase foreground
gray = cv2.cvtColor(binMap, cv2.COLOR_BGR2GRAY)
gray = cv2.dilate(gray.copy(), None, iterations=3)
cv2.imshow('gray',gray)

# Find Contours for binary Map
(cnts, heir) = cv2.findContours(gray.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
clone = img.copy()
cv2.drawContours(clone, cnts, -1, (0, 255, 0), 2)


# Save contours to file
f = open("OutputFiles/Outline.pkl","wb")
pickle.dump(cnts,f)
f.close()

# Display contours showing outline
cv2.imshow("MapOutline",clone)
cv2.waitKey(0)
cv2.destroyAllWindows()



