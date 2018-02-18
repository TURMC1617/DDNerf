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
    arduino = serial.Serial("/dev/ttyACM4")
    arduino.write("190.0\0")
    
    return window, joystick, arduino

def loop(window, joystick, arduino):
    running = True

    while (running):
        events = sdl2.ext.get_events()

        for event in events:
            if event.type == sdl2.SDL_QUIT:
                running = False
                break

        window.refresh()

        axis = sdl2.SDL_JoystickGetAxis(joystick, 0) / 32767.0
        angle = (axis * 90) + 90
        firing = sdl2.SDL_JoystickGetButton(joystick, 0)

        control = 1

        if (arduino.in_waiting):
            message = str(control) + str(firing) + str(angle) + "\0"
            arduino.write(bytes(message))
            arduino.readline()



def quit(joystick, arduino):
    sdl2.SDL_JoystickClose(joystick)
    sdl2.ext.quit()
    arduino.close()

if __name__ == "__main__":
    window, joystick, arduino = init()
    loop(window, joystick, arduino)
    quit(joystick, arduino)