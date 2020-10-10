import glfw
from OpenGL.GL import*
from OpenGL.GLU import*
import simulation as sim
import numpy as np

# Window
window = None

def createWindow(width = 1080, height = 1080, title = "Snake Locomotion"):
    global window
    
    glfw.init()
    window = glfw.create_window(width, height, title, None, None)

    glfw.make_context_current(window)
    glfw.set_key_callback(window, keyCallback)
    glfw.set_cursor_pos_callback(window, cursorPosCallback)
    glfw.set_mouse_button_callback(window, mouseButtonCallback)
    glfw.set_scroll_callback(window, scrollCallback)

def destroyWindow():
    global window
    
    glfw.destroy_window(window)
    glfw.terminate()

def isOpenWindow():
    global window
    
    if window != None:
        if not glfw.window_should_close(window):
            return True
        else:
            destroyWindow()
            return False
    else:
        return False
    
# Input Handler
XPOS = 0
YPOS = 0
BUTTON = 0

VIEW = [0, None, None, None, None]
DRAW = [True, True, False, False, False]
SNAKE = [True, True]
ENVIRONMENT = [False, True]
MUSCLE = True
BOUNDINGBOX = False
GOAL = True

def keyCallback(window, key, scancode, action, mods):
    global AZIMUTH, ELEVATION, DISTANCE, TARGET, VIEW, DRAW, SNAKE, ENVIRONMENT, MUSCLE, BOUNDINGBOX, GOAL

    if action == glfw.PRESS:
        if key == glfw.KEY_R:
            sim.resetSimulation()
        elif key == glfw.KEY_P:
            sim.pauseSimulation()
        elif key == glfw.KEY_V:
            VIEW[0] = (VIEW[0] + 1) % 3
            if VIEW[0] == 0:
                AZIMUTH = VIEW[1]
                ELEVATION = VIEW[2]
                DISTANCE = VIEW[3]
                TARGET = VIEW[4]
            elif VIEW[0] == 1:
                VIEW[4] = TARGET
            elif VIEW[0] == 2:
                VIEW[1] = AZIMUTH
                VIEW[2] = ELEVATION
                VIEW[3] = DISTANCE
        elif key == glfw.KEY_1:
            DRAW[0] = not DRAW[0]
        elif key == glfw.KEY_2:
            DRAW[1] = not DRAW[1]
        elif key == glfw.KEY_3:
            DRAW[2] = not DRAW[2]
        elif key == glfw.KEY_4:
            DRAW[3] = not DRAW[3]
        elif key == glfw.KEY_5:
            DRAW[4] = not DRAW[4]
        elif key == glfw.KEY_S:
            SNAKE[0] = not SNAKE[0]
            if SNAKE[0] == False:
                SNAKE[1] = not SNAKE[1]
        elif key == glfw.KEY_E:
            ENVIRONMENT[0] = not ENVIRONMENT[0]
            if ENVIRONMENT[0] == False:
                ENVIRONMENT[1] = not ENVIRONMENT[1]
        elif key == glfw.KEY_M:
            MUSCLE = not MUSCLE
        elif key == glfw.KEY_B:
            BOUNDINGBOX = not BOUNDINGBOX
        elif key == glfw.KEY_G:
            GOAL = not GOAL

def cursorPosCallback(window, xpos, ypos):
    global XPOS, YPOS, BUTTON, AZIMUTH, ELEVATION, DISTANCE, TARGET, EYE, UP

    if BUTTON == glfw.MOUSE_BUTTON_LEFT + 1:
        AZIMUTH -= (xpos - XPOS) / 100.
        ELEVATION += (ypos - YPOS) / 100.
        EYE = TARGET + DISTANCE * np.array([np.cos(ELEVATION) * np.sin(AZIMUTH), np.sin(ELEVATION), np.cos(ELEVATION) * np.cos(AZIMUTH)])
        UP = np.array([0., np.cos(ELEVATION), 0.])
    elif BUTTON == glfw.MOUSE_BUTTON_RIGHT + 1:
        w = (EYE - TARGET) / np.sqrt(np.dot(EYE - TARGET, EYE - TARGET))
        u = np.cross(UP, w) / np.sqrt(np.dot(np.cross(UP, w), np.cross(UP, w)))
        v = np.cross(w, u)
        TARGET -= (u * (xpos - XPOS) - v * (ypos - YPOS)) * (DISTANCE / 1000.)
        EYE = TARGET + DISTANCE * np.array([np.cos(ELEVATION) * np.sin(AZIMUTH), np.sin(ELEVATION), np.cos(ELEVATION) * np.cos(AZIMUTH)])
            
    XPOS = xpos
    YPOS = ypos

def mouseButtonCallback(window, button, action, mod):
    global BUTTON

    if action == glfw.PRESS:
        BUTTON += (button + 1)
    elif action == glfw.RELEASE:
        BUTTON -= (button + 1)

def scrollCallback(window, xoffset, yoffset):
    global AZIMUTH, ELEVATION, DISTANCE, EYE

    DISTANCE -= yoffset * (DISTANCE / 10.)
    EYE = TARGET + DISTANCE * np.array([np.cos(ELEVATION) * np.sin(AZIMUTH), np.sin(ELEVATION), np.cos(ELEVATION) * np.cos(AZIMUTH)])

# Render
FPS = 60.
TIME = 0.

AZIMUTH = np.pi / 4.
ELEVATION = np.pi / 6.
DISTANCE = np.pi
TARGET = np.array([0., 0., 0.])
EYE = TARGET + DISTANCE * np.array([np.cos(ELEVATION) * np.sin(AZIMUTH), np.sin(ELEVATION), np.cos(ELEVATION) * np.cos(AZIMUTH)])
UP = np.array([0., np.cos(ELEVATION), 0.])

def render():
    global window, FPS, TIME, AZIMUTH, ELEVATION, DISTANCE, TARGET, EYE, UP, VIEW, DRAW, SNAKE, ENVIRONMENT, MUSCLE, BOUNDINGBOX, GOAL

    glfw.poll_events()

    if (glfw.get_time() - TIME) < (1. / FPS):
        return
    TIME = glfw.get_time()

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    glEnable(GL_DEPTH_TEST)
    glClearColor(1., 1., 1., 1.)

    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(45., 1., 0.001, 1000.)

    if VIEW[0] == 0:
        EYE = TARGET + DISTANCE * np.array([np.cos(ELEVATION) * np.sin(AZIMUTH), np.sin(ELEVATION), np.cos(ELEVATION) * np.cos(AZIMUTH)])
        UP = np.array([0., np.cos(ELEVATION), 0.])
    elif VIEW[0] == 1:
        TARGET = sim.getVertexPosition()[int(sim.getNumVertex() / 2)]
        EYE = TARGET + DISTANCE * np.array([np.cos(ELEVATION) * np.sin(AZIMUTH), np.sin(ELEVATION), np.cos(ELEVATION) * np.cos(AZIMUTH)])
        UP = np.array([0., np.cos(ELEVATION), 0.])
    elif VIEW[0] == 2:
        TARGET = sim.getVertexPosition()[2] * 2 - sim.getVertexPosition()[7]
        EYE = sim.getVertexPosition()[2]
        UP = sim.getVertexPosition()[5] - sim.getVertexPosition()[7]

    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()
    gluLookAt(EYE[0], EYE[1], EYE[2], TARGET[0], TARGET[1], TARGET[2], UP[0], UP[1], UP[2])

    sim.drawGoal(GOAL)
    sim.drawBoundingBox(BOUNDINGBOX)
    sim.drawMuscle(MUSCLE)
    sim.drawEnvironment(ENVIRONMENT[0], ENVIRONMENT[1])
    sim.drawSnake(SNAKE[0], SNAKE[1])
    sim.drawVelocity(DRAW[4])
    sim.drawDirection(DRAW[3])
    sim.drawLocalFrame(DRAW[2])
    sim.drawGrid(DRAW[1])
    sim.drawGlobalFrame(DRAW[0])

    glfw.swap_buffers(window)
