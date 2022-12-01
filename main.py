from pickle import FALSE
import image_processor
import camera
import MainMovement
import cv2
import time
import ref_commands

motion = MainMovement.RobotMotion()

def main_loop():

    debug = False   #Whether or not to show the debug frame

    referee = ref_commands.Referee_cmd_client()
    cam = camera.RealsenseCamera(exposure = 100)

    #robotid for referee
    robotID = "jurto"

    #predefining first state
    current_state = MainMovement.States.spin

    #class call from other files
    movementState = MainMovement.StateMachine(motion)
    processor = image_processor.ImageProcessor(cam, debug=debug)
    
    #opening all that needs to be opened
    referee.open()
    motion.open()
    #print(motion.ser)
    processor.start()

    start = time.time()
    fps = 0
    frame = 0
    frame_cnt = 0

    cv2.namedWindow('debug')
    
    
    try:
        #movementState.connect(ip,port)
        status = False
        #
        while True:

            #referee command reading
            msg = referee.get_cmd()
            
            if msg is not None and msg['signal'] == 'start':
                if msg["targets"][0] == robotID:
                    if msg["baskets"][0] == "blue":
                        basketcolor = 'b'
                    else:
                        basketcolor = 'm'
                else:
                    if msg["baskets"][1] == "blue":
                        basketcolor = 'b'
                    else: 
                        basketcolor = 'm'
                status = True
            elif msg is not None and msg['signal'] == 'stop':
                status = False
            # has argument aligned_depth that enables depth frame to color frame alignment. Costs performance
            
            #processed data list which we use for movement
            processedData = processor.process_frame(aligned_depth=False)
            rgb = processedData.color_frame

            frame_cnt +=1

            frame += 1
            frame = 0
            end = time.time()
            fps = 30 / (end - start)
            start = end
            #print("FPS: {}, framecount: {}".format(fps, frame_cnt))
            #print("ball_count: {}".format(len(processedData.balls)))

            #whether to show the camera image or not
            if debug:
                debug_frame = processedData.debug_frame

                cv2.imshow('debug', debug_frame)
                k = cv2.waitKey(1) & 0xff
                if k == ord('q'):
                    break


            #State machine
            if status:
                if current_state == MainMovement.States.spin:
                    current_state = movementState.spin(processedData)
                elif current_state == MainMovement.States.drive:
                    current_state = movementState.drive(processedData)
                elif current_state == MainMovement.States.orbit:
                    current_state = movementState.orbit(processedData,basketcolor)
                elif current_state == MainMovement.States.throw:
                    basket = processedData.basket_b if basketcolor == 'b' else processedData.basket_m
                    if basket.exists:
                        basketdistcm = round(100*(cam.pixel_distance(basket.x,basket.y)))
                    current_state = movementState.throw(processedData,basketcolor,basketdistcm)
            #print(current_state)
            

    except KeyboardInterrupt:
        print("closing....")

    except Exception as e: #print errors
        print(e)
    finally:
        #closing/stopping everything we opened
        print("closing")
        cv2.destroyAllWindows()
        referee.close()
        motion.close()
        processor.stop()

main_loop()
