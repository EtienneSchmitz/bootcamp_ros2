# Liste de detections pour UNE image.
# header : reprendre le stamp/frame_id de l'image source (camera_optical_frame)
# pour synchroniser avec la TF et la back-projection.

std_msgs/Header   header
Detection[]       detections
