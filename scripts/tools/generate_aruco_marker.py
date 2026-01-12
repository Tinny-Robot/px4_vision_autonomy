#!/usr/bin/env python3
import cv2
import numpy as np
import sys
import os

def main():
    if len(sys.argv) > 1:
        marker_id = int(sys.argv[1])
    else:
        marker_id = 0
        
    print(f"Generating ArUco marker ID {marker_id} from DICT_4X4_50")
    
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    marker_img = cv2.aruco.generateImageMarker(aruco_dict, marker_id, 200)
    
    # Add a white border
    marker_img = cv2.copyMakeBorder(marker_img, 20, 20, 20, 20, cv2.BORDER_CONSTANT, value=255)
    
    filename = f"aruco_marker_{marker_id}.png"
    cv2.imwrite(filename, marker_img)
    print(f"Saved to {filename}")
    
    print("\nTo use this in Gazebo:")
    print("1. Create a flat box model.")
    print("2. Create a material script that uses this image as a texture.")
    print("3. Apply the material to the box.")
    print("OR simply use the 'aruco_visual_marker_0' model if available in your gazebo models.")

if __name__ == "__main__":
    main()
