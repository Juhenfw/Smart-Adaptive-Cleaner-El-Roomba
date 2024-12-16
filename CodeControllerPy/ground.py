"""  
This supervisor tracks the absolute position  
of the robot and removes the dirty from the  
area given by the robot.  
"""  

from controller import Supervisor  

TIME_STEP = 64  

# Size of the ground  
GROUND_X = 10  
GROUND_Z = 6  

# Oval size to ensure centered drawing  
OVAL_SIZE = 8  

def main():  
    # Initialize Webots  
    supervisor = Supervisor()  

    # Get a handler to devices  
    display = supervisor.getDevice("ground_display")  
    if display is None:  
        print("Error: 'ground_display' device not found.")  
        return  

    # Get the dimensions of the display  
    width = display.getWidth()  
    height = display.getHeight()  

    # Get the robot node and its translation field  
    mybot = supervisor.getFromDef("IROBOT_CREATE")  
    
    if mybot is None:  
        print("Error: Robot 'IROBOT_CREATE' not found.")  
        return  

    translationField = mybot.getField("translation")  

    # Set the background image  
    background = display.imageLoad("D:/Webots/projects/robots/irobot/create/worlds/textures/dirty.jpg")  
    display.imagePaste(background, 0, 0, False)  

    # Set the pen to remove the texture  
    display.setAlpha(0.0)  

    while supervisor.step(TIME_STEP) != -1:  
        # Update the translation field  
        translation = translationField.getSFVec3f()  

        # Normalize coordinates with aspect ratio compensation  
        x_normalized = (translation[0] + GROUND_X / 2.02) / GROUND_X  
        z_normalized = (translation[2] + GROUND_Z / 2.05) / GROUND_Z  

        # Adjust for aspect ratio differences in the display  
        if width > height:  
            # Landscape mode  
            # Scale Z more aggressively  
            x_display = int(width * x_normalized)  
            y_display = int(height * z_normalized * (height / width))  
        else:  
            # Portrait mode  
            # Scale X more aggressively  
            x_display = int(width * x_normalized * (width / height))  
            y_display = int(height * z_normalized)  

        # Debug information to understand coordinate mapping  
        print(f"Raw Translation: X={translation[0]}, Z={translation[2]}")  
        print(f"Normalized: X={x_normalized}, Z={z_normalized}")  
        print(f"Display Coords: X={x_display}, Y={y_display}")  

        # Precise centering calculation  
        x_center = x_display - OVAL_SIZE // 2  
        y_center = y_display - OVAL_SIZE // 2  

        # Boundary checks to prevent drawing outside display  
        if (0 <= x_center < width - OVAL_SIZE and   
            0 <= y_center < height - OVAL_SIZE):  
            display.fillOval(x_center, y_center, OVAL_SIZE, OVAL_SIZE)  
        else:  
            print(f"Warning: Robot position ({x_display}, {y_display}) out of display bounds")  

    supervisor.cleanup()  

if __name__ == "__main__":  
    main()