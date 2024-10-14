import rospy
from controller_manager_msgs.srv import ListControllers, SwitchController, SwitchControllerRequest
from controller_manager_msgs.utils import get_rosparam_controller_names
import time

# Helper function to create or get the 'list_controllers' service
def get_list_controllers_service(timeout=3):
    """Get or create the list_controllers service."""
    service_name = '/controller_manager/list_controllers'
    list_controllers = rospy.ServiceProxy(service_name, ListControllers)
    
    try:
        list_controllers.wait_for_service(timeout)
        print(f"[INFO] Service '{service_name}' is available.")
    except rospy.exceptions.ROSException as err:
        print(f"[ERROR] Service '{service_name}' not available: {err}")
        return None
    return list_controllers

# Helper function to create or get the 'switch_controller' service
def get_switch_controller_service(timeout=3):
    """Get or create the switch_controller service."""
    service_name = '/controller_manager/switch_controller'
    switch_controller = rospy.ServiceProxy(service_name, SwitchController)
    
    try:
        switch_controller.wait_for_service(timeout)
        print(f"[INFO] Service '{service_name}' is available.")
    except rospy.exceptions.ROSException as err:
        print(f"[ERROR] Service '{service_name}' not available: {err}")
        return None
    return switch_controller

# Function to check if the controller is in the parameter server
def check_parameter_server(controller):
    """Check if the controller is in the parameter server."""
    controller_names = get_rosparam_controller_names("/")
    if controller in controller_names:
        print(f"[INFO] Controller '{controller}' found in parameter server.")
        return True
    else:
        print(f"[WARNING] Controller '{controller}' not found in parameter server.")
        return False

# Function to start the given controller
def start_controller(controller):
    """Start the given controller with a best-effort strategy."""
    print(f"[INFO] Attempting to start controller '{controller}' with best-effort strategy.")
    
    switch_controller = get_switch_controller_service()  # Get switch controller service
    
    if switch_controller:
        srv = SwitchControllerRequest()
        srv.start_controllers = [controller]
        srv.strictness = SwitchControllerRequest.BEST_EFFORT
        result = switch_controller(srv)
        
        if result.ok:
            print(f"[INFO] SwitchControllerRequest accepted for starting '{controller}'.")
        else:
            print(f"[ERROR] SwitchControllerRequest failed for starting '{controller}'.")
    else:
        print(f"[ERROR] Could not start controller '{controller}' because service is unavailable.")

# Function to stop the given controller
def stop_controller(controller):
    """Stop the given controller with a best-effort strategy."""
    print(f"[INFO] Attempting to stop controller '{controller}' with best-effort strategy.")
    
    switch_controller = get_switch_controller_service()  # Get switch controller service
    
    if switch_controller:
        srv = SwitchControllerRequest()
        srv.stop_controllers = [controller]
        srv.strictness = SwitchControllerRequest.BEST_EFFORT
        result = switch_controller(srv)
        
        if result.ok:
            print(f"[INFO] SwitchControllerRequest accepted for stopping '{controller}'.")
        else:
            print(f"[ERROR] SwitchControllerRequest failed for stopping '{controller}'.")
    else:
        print(f"[ERROR] Could not stop controller '{controller}' because service is unavailable.")

# Main function to switch on the given controller
def switch_on_controller(controller_name, exclude_controllers=[]):
    """Switches on the given controller, stopping all other known controllers with best-effort strategy."""
    
    list_controllers = get_list_controllers_service()  # Get list controllers service
    switch_controller = get_switch_controller_service()  # Get switch controller service
    
    if list_controllers and switch_controller:
        # Get names of currently active controllers
        listed_controllers = list_controllers()         
        active_controllers = [c.name for c in listed_controllers.controller if c.state == 'running']
        
        # Determine which controllers to stop
        controllers_to_stop = [
            c for c in active_controllers
            if c not in exclude_controllers and c != controller_name
        ]
        
        srv = SwitchControllerRequest()
        srv.stop_controllers = controllers_to_stop
        srv.start_controllers = [controller_name]
        srv.strictness = SwitchControllerRequest.BEST_EFFORT
        result = switch_controller(srv)
        
        # Wait until controller spawner is done
        time.sleep(1)
        
        # Handle success or failure
        if not result.ok:
            raise Exception(f"Failed to switch on controller: '{controller_name}'")
        
        print(f"[INFO] Successfully switched on controller '{controller_name}'.")
        return result.ok
    else:
        print(f"[ERROR] Could not switch controller '{controller_name}' because necessary services are unavailable.")
        return False
