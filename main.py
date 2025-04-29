from p3dxRobot import P3DXRobot
import json

# Load the JSON file
with open('functions.json', 'r') as f:
    functions = json.load(f)

def display_functions():
    print("Available functions:")
    for index, (func_name, func_data) in enumerate(functions.items()):
        print(f"{index + 1}. {func_name}: Parameters - {', '.join(func_data['params'])}")

def run_selected_function(function_name, robot, *args):
    if function_name in functions:
        func = globals().get(functions[function_name]["functionname"])
        if func:
            func(robot, *args)  # Pass additional parameters
        else:
            print(f"Function {function_name} not found.")
    else:
        print(f"Invalid function name: {function_name}")


def tryMovement(robot, trans_vel, rot_vel):
    """
    Write the function you want the robot to do here
    :param robot: The robot we have initialized
    :return: nothing
    """
    print(f"Will go forward {trans_vel} with rotation {rot_vel}.")
    robot.send_velocity(trans_vel, rot_vel)
    robot.read_response()
    robot.send_velocity(0, 0)

def sendCommands(robot):
    """
    Write the function you want the robot to do here
    :param robot: The robot we have initialized
    :return: nothing
    """
    robot.send_velocity(200, 10)  # Move forward at 200mm/s, rotate 10deg/s
    robot.read_response()  # Read feedback (if any)
    robot.send_velocity(0, 0)


if __name__ == "__main__":
    display_functions()  # Display available functions

    # User selects the function
    selected_index = int(input("Enter the number of the function to run: ").strip()) - 1
    selected_function = list(functions.keys())[selected_index]

    # Get the parameters from JSON for the selected function
    params = functions[selected_function]["params"]

    # Get additional parameter values from the user
    additional_params = []
    for param in params[1:]:  # Skip the first 'robot' parameter
        value = input(f"Enter value for {param}: ")
        additional_params.append(value)

    # Convert numerical parameters if necessary (e.g., for velocity)
    additional_params = [int(p) if p.isdigit() else p for p in additional_params]

    robot = P3DXRobot()
    robot.runRobot(lambda r: run_selected_function(selected_function, r, *additional_params))  # Run selected function
