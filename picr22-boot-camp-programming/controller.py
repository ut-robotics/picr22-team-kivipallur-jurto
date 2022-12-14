import signal
from xbox360controller import Xbox360Controller
import MainMovement as MP
robotmotion = MP.RobotMotion()
onoff = 1


def on_button_pressed(button):
    print('Button {0} was pressed'.format(button.name))


def on_button_released(button):
    print('Button {0} was released'.format(button.name))


def trigger(axis):
    print(axis.value)


def on_axis_moved(axis):
    print('Axis {0} moved to {1} {2}'.format(axis.name, axis.x, axis.y))


try:
    controller = Xbox360Controller(0, axis_threshold=0.2)
    # Button A events
    #controller.button_a.when_pressed = on_button_pressed
    #controller.button_a.when_released = on_button_released
    


    # Left and right axis move event
    #controller.axis_l.when_moved = on_axis_moved
    #controller.axis_r.when_moved = on_axis_moved

    robotmotion.open()

    while(onoff):
    # Button A events
    #controller.button_a.when_pressed = on_button_pressed
    #controller.button_a.when_released = on_button_released
    
        
        #controller.axis_l.when_moved = on_axis_moved
        #controller.axis_r.when_moved = on_axis_moved

        try:
        
            
            throwerspeed = round(controller.trigger_r.value *1900) * speedstate
            print(throwerspeed)
            yspeed = controller.axis_l._value_y * -0.9
            xspeed = controller.axis_l._value_x *0.5

            rotate = controller.axis_r._value_x*-0.75
            if (controller.button_a.is_pressed):

            
            robotmotion.move(xspeed,yspeed,rotate,throwerspeed)
            
        except:
            break
            
        


    signal.pause()
except KeyboardInterrupt:
    robotmotion.close()
    signal.pause()
    
