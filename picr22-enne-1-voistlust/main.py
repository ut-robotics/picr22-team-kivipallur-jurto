from pickle import FALSE
import image_processor
import camera
import MainMovement
import cv2
import time
from enum import Enum

motion = MainMovement.RobotMotion()

def main_loop():

    current_state = MainMovement.States.spin
    debug = False   #Whether or not to show the debug frame

    cam = camera.RealsenseCamera(exposure = 100)

    basketcolor = 'm'
    movementState = MainMovement.StateMachine(motion)
    processor = image_processor.ImageProcessor(cam, debug=debug)
    
    motion.open()
    print(motion.ser)
    processor.start()
    #Priidu sekeldis cam.open()

    start = time.time()
    fps = 0
    frame = 0
    frame_cnt = 0

    cv2.namedWindow('debug')
    
    
    try:
        while True:
            # has argument aligned_depth that enables depth frame to color frame alignment. Costs performance
            processedData = processor.process_frame(aligned_depth=False)
            rgb = processedData.color_frame
            basket = processedData.basket_b if basketcolor == 'b' else processedData.basket_m
            if basket.exists:
                basketdistcm = round(100*(cam.pixel_distance(basket.x,basket.y)))
                print(basketdistcm)
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
            
            #State machine
            if current_state == MainMovement.States.spin:
                current_state = movementState.spin(processedData)
            elif current_state == MainMovement.States.drive:
                current_state = movementState.drive(processedData)
            elif current_state == MainMovement.States.orbit:
                current_state = movementState.orbit(processedData,basketcolor)
            elif current_state == MainMovement.States.throw:
                current_state = movementState.throw(processedData,basketcolor,basketdistcm)
            #print(current_state)
            #print(f"ball distance: {processedData.balls[-1].distance}")

    except KeyboardInterrupt:
        print("closing....")
    finally:
        cv2.destroyAllWindows()
        #cam.close()
        motion.close()
        processor.stop()

main_loop()
