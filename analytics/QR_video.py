# import the necessary packages
#import simple_barcode_image
import QR_code_module
from imutils.video import VideoStream
import argparse
import time
import cv2
import pandas as pd

#Things to declare
start_text = 'L'
#output_video_path = './FinalTesting_withloc/output/output3_1.mp4'
output_csv_path = 'submission.csv'

# Creating a dictionary with keys as the location ids
loc_box_dict = {}

# construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-v", "--video", help="path to the (optional) video file")
args = vars(ap.parse_args())
 
# if the video path was not supplied, grab the reference to the camera
if not args.get("video", False):
    vs = VideoStream(src=0).start()
    time.sleep(2.0)
 
# otherwise, load the video
else:
    vs = cv2.VideoCapture(args["video"])
    # Default resolutions of the frame are obtained.The default resolutions are system dependent.
    # We convert the resolutions from float to integer.
    frame_width = int(vs.get(3))
    frame_height = int(vs.get(4))
    # Define the codec and create VideoWriter object.The output is stored in 'outpy.avi' file.
#    out = cv2.VideoWriter(output_video_path, cv2.VideoWriter_fourcc('M','J','P','G'), 10, (frame_width,frame_height))

# keep looping over the frames
while (True):
    # grab the current frame and then handle if the frame is returned
    # from either the 'VideoCapture' or 'VideoStream' object,
    # respectively
    frame_qr = []
    frame = vs.read()
    frame = frame[1] if args.get("video", False) else frame
    
    # check to see if we have reached the end of the video
    if frame is None:
        break
    # get the height and width of the frame to crop the frame
    h, w = frame.shape[:2]
    frame1 = frame[int(h/4):int(h/1.5), int(w/5):int(w/2)].copy()
 
    # detect the barcode in the image
    decodedObjects, frame_qr = QR_code_module.decode(frame1)
    QR_code_module.display(frame1, decodedObjects)
    
    frame_qr = list(map(lambda x: x.decode("utf-8"), frame_qr))
    # Check if two QR codes are found in a single frame
    if len(frame_qr) == 2:
        # If 2 QR codes are found, check if one of them is a location ID
        for item in frame_qr:
            # If one of them is a location ID, add it as a key in 
            if start_text in item:
                key = item # Make the location ID as key
                frame_qr.remove(item) #pop that value from the list
                value = frame_qr[0]
                loc_box_dict[key] = value # add to the dictionary
                frame_qr = []
    # check if there is a frame with a location ID only
    if len(frame_qr) == 1:
        if (start_text in frame_qr[0]) and (frame_qr[0] not in loc_box_dict):
            loc_box_dict[frame_qr[0]] = ''
            frame_qr = []
                
    # show the frame and record if the user presses a key
    cv2.imshow("Frame", frame1)
#    out.write(frame1)
    key = cv2.waitKey(1)& 0xFF
    
    # if the 'q' key is pressed, stop the loop
    if key == ord("q"):
        break

# if we are not using a video file, stop the video file stream
if not args.get("video", False):
	vs.stop()
 
# otherwise, release the camera pointer
else:
    vs.release()
#    out.release()
 
# close all windows
cv2.destroyAllWindows()

############### GETTING INFO TO CSV #######################

loc_box_lists = [ [k,v] for k, v in loc_box_dict.items() ]
print(loc_box_lists)
# Converting to a pandas dataframe for easily exporting to CSV
df = pd.DataFrame(loc_box_lists, columns=['Location ID','Carton ID'], dtype ='str')
df.to_csv(output_csv_path, index=False)