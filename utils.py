import cv2 as cv
from time import sleep

font = cv.FONT_HERSHEY_SIMPLEX
font_scale = 1
thickness = 2


def display_image(text, image):
    """
    Display an image in a window
    :param text: Title
    :param image: image to display
    :return:
    """
    cv.imshow(text, image)
    cv.waitKey(0)
    cv.destroyAllWindows()


def print_error(text):
    """
    Print a text in red and assert 0
    :param text:
    :return:
    """
    RED = '\033[91m'  # Red text
    RESET = '\033[0m'
    print(RED + text + RESET)
    # assert 0


def draw_on_image(image, objs, text_choice=False):
    """
    get a list of VisionObject and draw them on the image
    :param image: image to draw on
    :param objs: list of VisionObject
    :param text_choice: True if you want to print the text on the image
    """
    # output = image.copy()
    for o in objs:
        cv.drawContours(image, o.contour, -1, o.color, 15)  # output image, contours, id (all if -1), color, thckness
        cv.circle(image, o.center_pixel, 10, o.color, -1)
        if text_choice:
            text, text_position, font_color = o.generate_label()
            cv.putText(image, text, text_position, font, font_scale, font_color, thickness)
    return image


def show_video_and_shoot():
    """
    Show the live camera and wait for the user to press 's' to shoot a photo
    :return: frame
    """
    cap = cv.VideoCapture(0)
    while True:
        # read from camera
        ret, frame = cap.read()

        # show frame
        cv.imshow('Live Camera', frame)

        # if s is pressed, shoot
        if cv.waitKey(1) & 0xFF == ord('s'):
            cap.release()
            break

    sleep(0.2)
    cv.imshow("Capture", frame)
    c = True
    while c:
        key = cv.waitKey(1)
        # wait to press space to continue
        if key & 0xFF == ord(' '):
            c = False

    return frame
