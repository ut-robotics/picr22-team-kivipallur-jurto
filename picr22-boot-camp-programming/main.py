import image_processor
import camera
import MovementPriit #Priidu sekeldis
import motion
import cv2
import time
import serial

def main_loop():
    #xlist = []
    ylist = []


    debug = True
    #Priidu sekeldis
    
    #Priidu sekeldis
    motion_sim = motion.TurtleRobot()
    motion_sim2 = motion.TurtleOmniRobot()
    #priidusekeldis
    gmove = MovementPriit.RobotMotion()
    #priidu sekeldis

    
    
    #camera instance for normal web cameras
    #cam = camera.OpenCVCamera(id = 2)
    # camera instance for realsense cameras
    cam = camera.RealsenseCamera(exposure = 100)
    
    processor = image_processor.ImageProcessor(cam, debug=debug)

    gmove.open()
    print("Works meaybe")
    processor.start()
    motion_sim.open()
    motion_sim2.open()

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
            
    except KeyboardInterrupt:
        print("closing....")
    finally:
        #Priidu sekeldis
        
        #Priidu sekeldis
        cv2.destroyAllWindows()
        gmove.close()
        processor.stop()
        motion_sim.close()
        motion_sim2.close()

main_loop()
