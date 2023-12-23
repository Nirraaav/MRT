obstacle_list = []
rover_list = []
user_list = []
command_log = []


class Obstacle:
    def __init__(self, location, radius):
        self.location = location
        self.radius = radius
        obstacle_list.append(self)

    def __str__(self):
        return f"Obstacle at {(self.location[0], self.location[1])} with Safety Radius {self.radius}"


def display_obstacles():
    for obstacle in obstacle_list:
        print(obstacle)


def distance(location_1, location_2):
    res = ((location_1[0] - location_2[0]) ** 2 + (location_1[1] - location_2[1]) ** 2) ** 0.5
    return res


class Rover:
    geometry = {
        'length': 1,
        'width': 1,
        'height': 1
    }

    def __init__(self, swarm_id, rover_id, location, battery=100):
        self.swarm_id = swarm_id
        self.rover_id = rover_id
        self.location = location
        self.battery = battery
        rover_list.append(self)

    def display_info(self):
        print(f"Swarm ID: {self.swarm_id}, Rover ID: {self.rover_id}")
        print(f"Location: ({self.location[0]}, {self.location[1]})")
        print(f"Rover Battery : {self.battery}%")
        print(f"Rover Geometry (L x W x H): {self.geometry['length']} x {self.geometry['width']} x {self.geometry['height']}")

    def check_safety(self):
        for obstacle in obstacle_list:
            if distance(obstacle.location, self.location) <= 1e-6:
                print(f"Rover with Swarm ID: {self.swarm_id}, Rover ID: {self.rover_id} died")
                rover_list.remove(self)
                return
        for obstacle in obstacle_list:
            if distance(obstacle.location, self.location) <= obstacle.radius:
                print(f"Rover with Swarm ID: {self.swarm_id}, Rover ID: {self.rover_id} is unsafe")
                return

    def change_location(self, message_swarm_id, message_rover_id, move_amount):
        if self.swarm_id == message_swarm_id and self.rover_id == message_rover_id:
            print(f"Received message for Swarm ID: {message_swarm_id}, Rover ID: {message_rover_id}")
            dist = distance((0, 0), (move_amount[0], move_amount[1]))
            if (self.battery - dist * (0.1)) < 0:
                print("Insufficient juice to move the rover")
                return
            else:
                print(f"Moving by {move_amount} units.")
                self.location = (self.location[0] + move_amount[0], self.location[1] + move_amount[1])
                self.battery = self.battery - dist * (0.1)
                print(f"New Location: {self.location}")
                print(f"Battery left: {self.battery}%")
                self.check_safety()
        else:
            print("You are making a mistake")


def display_rovers():
    for rover in rover_list:
        rover.display_info()


class DaughterRover(Rover):
    geometry = {
        'length': Rover.geometry['length'] // 2,
        'width': Rover.geometry['width'] // 2,
        'height': Rover.geometry['height'] // 2
    }

class User:
    def __init__(self, user_id):
        self.user_id = user_id
        self.controlled_rovers = set()

    def print_user_id(self):
        print(f"User ID: {self.user_id}")

    def view_rover_location(self, rover):
        print(f"{self.user_id} is denied access to View Rover Location") 

    def move_rover(self, rover, move_amount):
        print(f"{self.user_id} is denied access to Move Rover") 

    def add_rover(self, rover):
        print(f"{self.user_id} is denied access to Add Rover") 

    def remove_rover(self, rover):
        print(f"{self.user_id} is denied access to Remove Rover") 

    def get_user_type(self):
        return type(self).__name__  


class Scientist(User):
    def view_rover_location(self, rover):
        print(f"{self.user_id} is viewing the location of Rover ID: {rover.rover_id}")
        print(f"Rover Location: {rover.location}") 

    def get_user_type(self):
        return type(self).__name__  


class Operator(User):
    def move_rover(self, rover):
        x = float(input("Enter the X amount to move the rover: "))
        y = float(input("Enter the Y amount to move the rover: "))
        move_amount = (x, y)
        rover.change_location(rover.swarm_id, rover.rover_id, move_amount) 

    def get_user_type(self):
        return type(self).__name__  


class Manager(User):
    def add_rover(self, rover):
        self.controlled_rovers.add((rover.swarm_id, rover.rover_id))
        print(f"{self.user_id} added Rover ID: {rover.rover_id} to controlled rovers.")

    def remove_rover(self, rover):
        self.controlled_rovers.remove((rover.swarm_id, rover.rover_id))
        print(f"{self.user_id} removed Rover ID: {rover.rover_id} from controlled rovers.")

    def get_user_type(self):
        return type(self).__name__  


class Admin(User):
    def __init__(self, user_id):
        super().__init__(user_id)
        user_list.append(self)

    def view_all_users(self):
        if isinstance(self, Admin):
            print("All Users:")
            for user in user_list:
                print(f"{user.get_user_type()} : {user.user_id}")
        else:
            print("Access denied. Only Admins can view all users.")

    def logout(self):
        print(f"Logging out {self.user_id}...")
        return None  # Return None or perform any other necessary action for logout


def get_rover_by_id(rover_id):
    for rover in rover_list:
        if rover.rover_id == rover_id:
            return rover
    return None


def add_rover_interface():
    swarm_id = input("Enter Swarm ID for the new rover: ")
    rover_id = input("Enter Rover ID for the new rover: ")
    x = float(input("Enter X coordinate for rover's location: "))
    y = float(input("Enter Y coordinate for rover's location: "))
    battery = float(input("Enter battery percentage for the new rover: "))
    new_rover = Rover(swarm_id, rover_id, (x, y), battery)
    print("Rover added successfully.")
    display_rovers()


def remove_rover_interface():
    swarm_id = input("Enter Swarm ID of the rover to remove: ")
    rover_id = input("Enter Rover ID of the rover to remove: ")
    for rover in rover_list:
        if rover.swarm_id == swarm_id and rover.rover_id == rover_id:
            rover_list.remove(rover)
            print(f"Rover {rover_id} removed successfully.")
            display_rovers()
            return
    print("Rover not found.")


def add_obstacle_interface():
    x = float(input("Enter X coordinate for obstacle's location: "))
    y = float(input("Enter Y coordinate for obstacle's location: "))
    radius = float(input("Enter safety radius for the obstacle: "))
    new_obstacle = Obstacle((x, y), radius)
    print("Obstacle added successfully.")
    display_obstacles()


def add_scientist_interface():
    user_id = input("Enter Scientist ID: ")
    new_scientist = Scientist(user_id)
    user_list.append(new_scientist)
    print("Scientist added successfully.")


def add_manager_interface():
    user_id = input("Enter Manager ID: ")
    new_manager = Manager(user_id)
    user_list.append(new_manager)
    print("Manager added successfully.")


def add_operator_interface():
    user_id = input("Enter Operator ID: ")
    new_operator = Operator(user_id)
    user_list.append(new_operator)
    print("Operator added successfully.")


def remove_scientist_interface():
    user_id = input("Enter Scientist ID to remove: ")
    for user in user_list:
        if isinstance(user, Scientist) and user.user_id == user_id:
            user_list.remove(user)
            print(f"Scientist {user_id} removed successfully.")
            return
    print("Scientist not found.")


def remove_manager_interface():
    user_id = input("Enter Manager ID to remove: ")
    for user in user_list:
        if isinstance(user, Manager) and user.user_id == user_id:
            user_list.remove(user)
            print(f"Manager {user_id} removed successfully.")
            return
    print("Manager not found.")


def remove_operator_interface():
    user_id = input("Enter Operator ID to remove: ")
    for user in user_list:
        if isinstance(user, Operator) and user.user_id == user_id:
            user_list.remove(user)
            print(f"Operator {user_id} removed successfully.")
            return
    print("Operator not found.")


def display_command_log():
    print("Command Log:")
    for command in command_log:
        print(command)


def login():
    while True:
        user_id = input("Enter User ID: ")
        for user in user_list:
            if user.user_id == user_id:
                print(f"Welcome, {user_id}!")
                return user
        print("User ID not found. Please try again.")


# Modify interactive_menu() function
def interactive_menu():
    print("Welcome to the Rover Management System!")
    logged_user = login()

    while True:
        if isinstance(logged_user, Admin):
            print("\nAdmin Menu:")
            print("1 - Add Rover")
            print("2 - Remove Rover")
            print("3 - Add Obstacle")
            print("4 - View Rover Location")
            print("5 - Move Rover")
            print("6 - Add Controlled Rover")
            print("7 - Remove Controlled Rover")
            print("8 - Add Scientist")
            print("9 - Remove Scientist")
            print("10 - Add Operator")
            print("11 - Remove Operator")
            print("12 - Add Manager")
            print("13 - Remove Manager")
            print("14 - View All Users")
            print("cmd - View Command Log")
            print("logout - Logout")
            print("0 - Exit")

            # Example:
            admin_choice = input("Enter your choice: ")
            command = f"Admin {logged_user.user_id} - Action: {admin_choice}"
            command_log.append(command)

            if admin_choice == "1":
                add_rover_interface()
            elif admin_choice == "2":
                remove_rover_interface()
            elif admin_choice == "3":
                add_obstacle_interface()
            elif admin_choice == "4":
                add_scientist_interface()
            elif admin_choice == "5":
                add_manager_interface()
            elif admin_choice == "6":
                add_operator_interface()
            elif admin_choice == "7":
                remove_scientist_interface()
            elif admin_choice == "8":
                remove_manager_interface()
            elif admin_choice == "9":
                remove_operator_interface()
            elif admin_choice == "10":
                admin.view_all_users()


interactive_menu()
