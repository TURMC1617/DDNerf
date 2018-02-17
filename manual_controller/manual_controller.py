import sys
import sdl2
import sdl2.ext
import serial

def init():
    sdl2.ext.init()
    sdl2.SDL_Init(sdl2.SDL_INIT_JOYSTICK)

    window = sdl2.ext.Window("Gun Control", size=(400, 300))
    window.show()

    joystick = sdl2.SDL_JoystickOpen(0)
    arduino = serial.Serial("/dev/ttyACM2")
    
    return window, joystick, arduino

def loop(window, joystick, arduino):
    running = True
    firing = 1
    axis = 0

    while (running):
        events = sdl2.ext.get_events()

        for event in events:
            if event.type == sdl2.SDL_QUIT:
                running = False
                print "Window exiting"
                break

        window.refresh()

        axis = sdl2.SDL_JoystickGetAxis(joystick, 0) / 32767.0
        angle = (axis * 90) + 90

        message = str(firing) + str(angle) + "\0"
        arduino.write(bytes(message))



def quit(joystick, arduino):
    sdl2.SDL_JoystickClose(joystick)
    sdl2.ext.quit()
    arduino.close()

if __name__ == "__main__":
    window, joystick, arduino = init()
    loop(window, joystick, arduino)
    quit(joystick, arduino)