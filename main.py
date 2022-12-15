import image_processor
import camera
import MainMovement
import cv2
import time
import ref_commands
from Color import Color
from xbox360controller import Xbox360Controller


motion = MainMovement.RobotMotion()

def main_loop():

    basketcolor = Color.BLUE # if no referee command is received we use this

    timeoutframes = 0 #every time spin function is done timeoutframes increases, any other function resets it.

    debug = True   #Whether or not to show the debug frame

    referee = ref_commands.Referee_cmd_client()
    cam = camera.RealsenseCamera(exposure = 100)

    try:
        controller = Xbox360Controller(0, axis_threshold=0.2)
        controller_connected = True
    except:
        controller_connected = False

    #robotid for referee
    robotID = "jurto"

    #predefining first state
    current_state = MainMovement.States.spin

    #class call from other files
    movementState = MainMovement.StateMachine(motion,cam.rgb_width,cam.rgb_height)
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
                    basketcolor = Color.BLUE if msg["baskets"][0] == "blue" else Color.MAGENTA
                else:
                    basketcolor = Color.BLUE if msg["baskets"][1] == "blue" else Color.MAGENTA
                status = True
            elif msg is not None and msg['signal'] == 'stop':
                status = False
            # has argument aligned_depth that enables depth frame to color frame alignment. Costs performance
            #print(controller_connected)
            if(controller_connected):
                if (controller.button_a.is_pressed):
                    current_state = MainMovement.States.controller
                    status = True
                if (controller.button_x.is_pressed):
                    current_state = MainMovement.States.spin
                    status = True

                if (controller.button_b.is_pressed):
                    current_state = MainMovement.States.spin
                    status = False
                    controller_connected = False

            
            #processed data list which we use for movement
            processedData = processor.process_frame()

            frame_cnt +=1

            frame += 1
            frame = 0
            end = time.time()
            fps = 1 / (end - start)
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
                    timeoutframes += 1
                    current_state = movementState.spin(processedData,timeoutframes)
                elif current_state == MainMovement.States.drive:
                    current_state = movementState.drive(processedData)
                elif current_state == MainMovement.States.orbit:
                    current_state = movementState.orbit(processedData,basketcolor)
                elif current_state == MainMovement.States.throw:
                    current_state = movementState.throw(processedData,basketcolor)
                elif current_state == MainMovement.States.controller:
                    current_state = movementState.controller(controller)
                elif current_state == MainMovement.States.enemy_basket_spin:
                    current_state = movementState.enemy_basket_spin(processedData, basketcolor)
                elif current_state == MainMovement.States.enemy_basket_approach:
                    current_state = movementState.enemy_basket_approach(processedData, basketcolor)
            #print(current_state)
            if current_state != MainMovement.States.spin:
                timeoutframes = 0  

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
