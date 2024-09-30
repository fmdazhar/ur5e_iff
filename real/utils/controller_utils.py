import rospy
from controller_manager_msgs.srv import ListControllers, SwitchController, SwitchControllerRequest
from controller_manager_msgs.utils import get_rosparam_controller_names

class ControllerUtil:
    def __init__(self):
        self.list_controllers = rospy.ServiceProxy('/controller_manager/list_controllers', ListControllers)
        self.switch_controller = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)

    def check_state(self, controller, state):
            """Check the controller's state.

            Return True if the controller's state is `state`, else False.
            Return False if the controller is not listed.
            """
            listed_controllers = self.list_controllers()
            for entry in listed_controllers.controller:
                if entry.name == controller:
                    if entry.state == state:
                        print(f"[INFO] Controller '{controller}' is in state '{state}'.")
                        return True
                    else:
                        print(f"[INFO] Controller '{controller}' is in state '{entry.state}', expected '{state}'.")
                        return False
            print(f"[WARNING] Controller '{controller}' not found in the list of controllers.")
            return False

    def check_parameter_server(self, controller):
        """ Check if the controller is in the parameter server """
        controller_names = get_rosparam_controller_names("/")
        if controller in controller_names:
            print(f"[INFO] Controller '{controller}' found in parameter server.")
            return True
        else:
            print(f"[WARNING] Controller '{controller}' not found in parameter server.")
            return False

    def start_controller(self, controller):
        """Start the given controller with a best-effort strategy."""
        print(f"[INFO] Attempting to start controller '{controller}' with best-effort strategy.")
        srv = SwitchControllerRequest()
        srv.start_controllers = [controller]
        srv.strictness = SwitchControllerRequest.BEST_EFFORT
        result = self.switch_controller(srv)
        if result.ok:
            print(f"[INFO] SwitchControllerRequest accepted for starting '{controller}'.")
        else:
            print(f"[ERROR] SwitchControllerRequest failed for starting '{controller}'.")

    def stop_controller(self, controller):
        """Stop the given controller with a best-effort strategy."""
        print(f"[INFO] Attempting to stop controller '{controller}' with best-effort strategy.")
        srv = SwitchControllerRequest()
        srv.stop_controllers = [controller]
        srv.strictness = SwitchControllerRequest.BEST_EFFORT
        result = self.switch_controller(srv)
        if result.ok:
            print(f"[INFO] SwitchControllerRequest accepted for stopping '{controller}'.")
        else:
            print(f"[ERROR] SwitchControllerRequest failed for stopping '{controller}'.")

    def switch_on_controller(self, controller_name, exclude_controllers=[]):
        """Switches on the given controller stopping all other known controllers with best_effort
        strategy."""
        srv = SwitchControllerRequest()
        listed_controllers = self.list_controllers()         # Get names of currently active controllers
        active_controllers = [c.name for c in listed_controllers if c.state == 'running']
        controllers_to_stop = [
            c for c in active_controllers
            if c not in exclude_controllers and c != controller_name ] # Determine which controllers to stop
        srv.stop_controllers = controllers_to_stop
        srv.start_controllers = [controller_name]
        srv.strictness = SwitchControllerRequest.BEST_EFFORT
        result = self.switch_controller(srv)
        # Handle success or failure
        if not result.ok:
            raise Exception(f"Failed to switch on controller: '{controller_name}'")
        print(f"[INFO] Successfully switched on controller '{controller_name}'.")
        return result.ok
