import image_processor
import camera
import MainMovement
import cv2
import time
from enum import Enum

basketcolor = 'b'
movementState = MainMovement.stateMachine()
motion = MainMovement.RobotMotion()

def main_loop():

    current_state = 0
    debug = True
    cam = camera.RealsenseCamera(exposure = 100)
    
    processor = image_processor.ImageProcessor(cam, debug=debug)
    
    motion.__init__()
    motion.open()
    processor.start()

    start = time.time()
    fps = 0
    frame = 0
    frame_cnt = 0
    try:
        while True:
            # has argument aligned_depth that enables depth frame to color frame alignment. Costs performance
            processedData = processor.process_frame(aligned_depth=False)

            # This is where you add the driving behaviour of your robot. It should be able to filter out
            # objects of interest and calculate the required motion for reaching the objects

            frame_cnt +=1

            frame += 1
            frame = 0
            end = time.time()
            fps = 30 / (end - start)
            start = end
            #print("FPS: {}, framecount: {}".format(fps, frame_cnt))
            #print("ball_count: {}".format(len(processedData.balls)))
               

            if debug:
                debug_frame = processedData.debug_frame

                cv2.imshow('debug', debug_frame)

                k = cv2.waitKey(1) & 0xff
                if k == ord('q'):
                    break
            
            if current_state == 0:
                current_state = movementState.spin(processedData)
            elif current_state == 1:
                current_state = movementState.drive(processedData)
            elif current_state == 2:
                current_state = movementState.orbit(processedData,basketcolor)
            elif current_state == 3:
                current_state = movementState.throw(processedData)

    except KeyboardInterrupt:
        print("closing....")
    finally:
        cv2.destroyAllWindows()
        motion.close()
        processor.stop()

main_loop()