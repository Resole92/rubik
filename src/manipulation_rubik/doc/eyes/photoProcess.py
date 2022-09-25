import numpy as np
import cv2
import math
import statistics
from cv2 import *
#from google.colab.patches import cv2_imshow

MAX_WIDTH = 70 
MAX_HEIGHT = 70

COLOR_MIN_YELLOW = np.array([20, 100, 0],np.uint8)       # HSV color code lower and upper bounds
COLOR_MAX_YELLOW = np.array([30, 255, 255],np.uint8)       # color yellow 

COLOR_MIN_GREEN = np.array([30, 100, 0],np.uint8)       # HSV color code lower and upper bounds
COLOR_MAX_GREEN = np.array([90, 255, 255],np.uint8)       # color green 

COLOR_MIN_RED = np.array([160, 50, 0],np.uint8)       # HSV color code lower and upper bounds
COLOR_MAX_RED = np.array([180, 255, 255],np.uint8)       # color red 

COLOR_MIN_RED_2 = np.array([0, 100, 0],np.uint8)       # HSV color code lower and upper bounds
COLOR_MAX_RED_2 = np.array([7, 255, 255],np.uint8)       # color red 

COLOR_MIN_BLUE = np.array([100, 120, 0],np.uint8)       # HSV color code lower and upper bounds
COLOR_MAX_BLUE = np.array([130, 255, 255],np.uint8)       # color blue 

COLOR_MIN_ORANGE = np.array([10, 100, 0],np.uint8)       # HSV color code lower and upper bounds
COLOR_MAX_ORANGE = np.array([20, 255, 255],np.uint8)       # color orange 

def get_hsv_image(image):
    #Serve per rendere più cartoon.
    #Il primo parametro serve ad indicare il numero di pixel vicini affetti
    # Gli altri parametri indicano quanto l'effetto dei pixel deve essere mischiato per renderli uniformi.
    image = cv2.bilateralFilter(image,9,250,250)
    #cv2_imshow(im)
    image = cv2.fastNlMeansDenoisingColored(image,None,10,10,7,21)
    hsv_img = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)   # HSV image
    return hsv_img

def find_contours(hsv_image, color_min, color_max, color_name):
  clean_contours = []

  frame_threshed = cv2.inRange(hsv_image, color_min, color_max)     # Thresholding image
  imgray = frame_threshed
  ret,thresh = cv2.threshold(frame_threshed,127,255,0)
  contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
  
  for cnt in contours:
    x,y,w,h = cv2.boundingRect(cnt)

    if(300 > w > MAX_WIDTH and 300 > h > MAX_HEIGHT):
      #cv2.rectangle(im,(x,y),(x+w,y+h),(0,255,0),2)
      #print(str(x) + " "+ str(y) + " " +  str(w) + " " + str(h))
      clean_contours.append((cnt,color_name));
    
  return clean_contours

def print_contours(detailed_contours, image):
  for detailed_contour in detailed_contours:
    cnt = detailed_contour[0]
    color_name = detailed_contour[1]

    x,y,w,h = cv2.boundingRect(cnt)

    # Write some Text
    xText = x + 30;
    yText = math.floor(y + h/2);

    font                   = cv2.FONT_HERSHEY_SIMPLEX
    bottomLeftCornerOfText = (xText, yText)
    fontScale              = 1
    fontColor              = (0,0,0)
    thickness              = 1
    lineType               = 2
    
    cv2.putText(image,color_name, 
    bottomLeftCornerOfText, 
    font, 
    fontScale,
    fontColor,
    thickness,
    lineType)

    cv2.rectangle(image,(x,y),(x+w,y+h),(0,255,0),2)
  
def is_overlap(r1, r2):
  return not ((r1[0]>=r2[2]) or (r1[2]<=r2[0]) or (r1[3]<=r2[1]) or (r1[1]>=r2[3]))

def is_contains(r2, r1):
  return r1[0] < r2[0] < r2[2] < r1[2] and r1[1] < r2[1] < r2[3] < r1[3]

def remove_all_overlap(detailed_contours):
  no_overlaped_contours = []

  for idx_source, source_detailed_contours in enumerate(detailed_contours):
    is_overlaped = False
    source_contours = source_detailed_contours[0]
    source_color = source_detailed_contours[1]
    x_source,y_source,w_source,h_source = cv2.boundingRect(source_contours)
    for idx_target,target_detailed_contours in  enumerate(detailed_contours):
      target_contours = target_detailed_contours[0]
      target_color = target_detailed_contours[1]
      x_target,y_target,w_target,h_target = cv2.boundingRect(target_contours)
      source_data = [x_source, y_source, x_source + w_source, y_source + h_source]
      target_data = [x_target, y_target, x_target + w_target, y_target + h_target]
      if is_contains(source_data, target_data) and idx_source != idx_target and (source_color == "blue" or target_color == "blue") :
        is_overlaped = True
        break
      if is_overlap(source_data, target_data) and idx_source != idx_target and (source_color != "blue" and target_color != "blue") :
          if w_target * h_target > w_source * h_source:
            is_overlaped = True
    if not is_overlaped:
        no_overlaped_contours.append(source_detailed_contours)
  
  return no_overlaped_contours

def check_image_color(image_path):

  im = cv2.imread(image_path)
  hsv_img = get_hsv_image(im)
    
  orange_contours = find_contours(hsv_img, COLOR_MIN_ORANGE, COLOR_MAX_ORANGE, 'orange')
  red_contours = find_contours(hsv_img,COLOR_MIN_RED, COLOR_MAX_RED, 'red')
  red2_contours = find_contours(hsv_img,COLOR_MIN_RED_2, COLOR_MAX_RED_2, 'red')
  green_contours = find_contours(hsv_img,COLOR_MIN_GREEN, COLOR_MAX_GREEN, 'green')
  yellow_contours = find_contours(hsv_img,COLOR_MIN_YELLOW, COLOR_MAX_YELLOW, 'yellow')
  blue_contours = find_contours(hsv_img,COLOR_MIN_BLUE, COLOR_MAX_BLUE, 'blue')

  all_contours = orange_contours + red_contours + red2_contours + green_contours + yellow_contours + blue_contours
  #print_contours(all_contours, im)

  no_overlaped_contours = remove_all_overlap(all_contours)

  #print_contours(no_overlaped_contours, im)
  #cv2_imshow(im)
  #cv2.imwrite(image_path + "extracted.jpg", im)
  cv2.waitKey()
  cv2.destroyAllWindows()

  return no_overlaped_contours

def get_all_position(path):
  # Load iamge, grayscale, adaptive threshold
  image = cv2.imread(path)
  copy = image.copy()
  grey = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
  decrease_noise = cv2.fastNlMeansDenoising(grey, 10, 15, 7, 21)
  blurred = cv2.GaussianBlur(decrease_noise, (3, 3), 0)
  canny = cv2.Canny(blurred, 20, 40)
  thresh = cv2.threshold(canny, 0, 255, cv2.THRESH_OTSU + cv2.THRESH_BINARY)[1]
  contours = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

  contours = contours[0] if len(contours) == 2 else contours[1]

  all_rectangle = []

  for index, c in enumerate(contours):
      # obtain the bounding rectangle coordinates for each square
      x, y, w, h = cv2.boundingRect(c)
      if(300 > w > 140 and 300 > h > 140): 
        # With the bounding rectangle coordinates we draw the green bounding boxes
        all_rectangle.append((c, str(index)))

  clear_rectangle = remove_all_overlap(all_rectangle)
  #print_contours(clear_rectangle, copy)
  #cv2_imshow(copy)
  return clear_rectangle

def is_greater_contour(a,b, threshold):
  x_a,y_a,w_a,h_a = cv2.boundingRect(a)
  x_b,y_b,w_b,h_b = cv2.boundingRect(b)

  center_xa = x_a + w_a/2
  center_ya = y_a + h_a/2

  center_xb = x_b + w_b/2
  center_yb = y_b + h_b/2

  if center_yb - threshold <= center_ya <= center_yb + threshold:
    return center_xa >= center_xb
  return center_ya > center_yb


def selectionSort(array, size, threshold):
    for s in range(size):
        min_idx = s
         
        for i in range(s + 1, size):
             
            # For sorting in descending order
            # for minimum element in each loop
            if is_greater_contour(array[min_idx][0],array[i][0], threshold):
                min_idx = i

        # Arranging min at the correct position
        (array[s], array[min_idx]) = (array[min_idx], array[s])

def order_rectangle(detailed_contours, path):
  image = cv2.imread(path)
  ordered_contours = []
  heights = []
  for detailed_contour in detailed_contours:
    x,y,w,h = cv2.boundingRect(detailed_contour[0])
    heights.append(h)
  mean_height = statistics.mean(heights)
  selectionSort(detailed_contours, len(detailed_contours), mean_height/2)
  
  renamed_position = []
  for index, detailed_contour in enumerate(detailed_contours):
    renamed_position.append((detailed_contour[0],str(index + 1) ))
  
  #print_contours(renamed_position, image)
  #cv2_imshow(image)

  return renamed_position

def colored_position(rectangles, colored_rectangles):
  colored_and_positioned = []
  for rectangle in rectangles:
    color_found = False
    for colored_rectangle in colored_rectangles:
      x_col, y_col, w_col, h_col = cv2.boundingRect(colored_rectangle[0])
      x_pos, y_pos, w_pos, h_pos = cv2.boundingRect(rectangle[0])
      center_xcol = x_col + w_col/2
      center_ycol = y_col + h_col/2
      if x_pos < center_xcol < x_pos + w_col and y_pos < center_ycol < y_pos + h_pos:
        color_found = True
        colored_and_positioned.append((colored_rectangle[0], colored_rectangle[1] +  " " + rectangle[1]))
        break
    if not color_found:
      colored_and_positioned.append((rectangle[0], "white " + rectangle[1]))
  return colored_and_positioned

#for n in range(7,8):
for n in range(0,9):
  print("processing photo number " + str(n))
  path = "src/manipulation_rubik/doc/eyes/photos/rubik_face_" + str(n+1) + ".jpg"
  finalImage = "src/manipulation_rubik/doc/eyes/photos_processed/rubik_face_process_" + str(n+1) + ".jpg"
  rectangles = get_all_position(path)
  ordered_rectangles = order_rectangle(rectangles, path)
  colored_rectagles = check_image_color(path)
  colored_and_positioned = colored_position(ordered_rectangles,colored_rectagles)

  image = cv2.imread(path)
  print_contours(colored_and_positioned, image)
  cv2.imwrite(finalImage, image)
  #cv2_imshow(image)