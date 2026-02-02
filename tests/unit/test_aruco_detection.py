import unittest
import cv2
import numpy as np

class TestArucoDetection(unittest.TestCase):
    def test_detection(self):
        # Generate a marker
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        marker_id = 0
        # Support multiple OpenCV versions: prefer generateImageMarker, fall back to drawMarker
        if hasattr(cv2.aruco, 'generateImageMarker'):
            marker_img = cv2.aruco.generateImageMarker(aruco_dict, marker_id, 200)
        elif hasattr(cv2.aruco, 'drawMarker'):
            marker_img = cv2.aruco.drawMarker(aruco_dict, marker_id, 200)
        else:
            self.skipTest('No suitable aruco marker generation API available')
        
        # Create a larger image and place marker
        bg = np.ones((400, 400), dtype=np.uint8) * 255
        bg[100:300, 100:300] = marker_img
        
        # Detect
        # Create detector parameters in a way that's compatible across OpenCV versions
        if hasattr(cv2.aruco, 'DetectorParameters_create'):
            params = cv2.aruco.DetectorParameters_create()
        else:
            params = cv2.aruco.DetectorParameters()
        corners, ids, rejected = cv2.aruco.detectMarkers(bg, aruco_dict, parameters=params)
        
        self.assertIsNotNone(ids)
        self.assertEqual(len(ids), 1)
        self.assertEqual(ids[0][0], marker_id)

if __name__ == '__main__':
    unittest.main()
