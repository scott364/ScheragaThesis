import numpy as np
import cv2

def save(image, file):
    np.save(file, image)
    
def load(file):
    image = np.load(file)
    return image
    
def save_jpeg(image, file):
    cv2.imwrite(file, cv2.cvtColor(image, cv2.COLOR_RGB2BGR))
    cv2.imwrite(file, cv2.cvtColor(image, cv2.COLOR_RGB2BGR))
    
def load_jpeg(file):    
    img = cv2.imread(file)
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    return img
    
