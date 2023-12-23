obstacle_list = [] 

class Obstacle:
    def __init__(self, location, radius):
        self.location = location
        self.radius = radius
        obstacle_list.append(self)

def display_obstacles():
    for obstacle in obstacle_list:
        print(f"Obstacle at {(obstacle.location[0], obstacle.location[1])} with Safety Radius {obstacle.radius}")
    print("---------------------------------------------------------")


def distance(location_1, location_2):
    res = ((location_1[0] - location_2[0])**2 + (location_1[1] - location_2[1])**2)**0.5
    return res

rover_list = []

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
            if(distance(obstacle.location, self.location) <= 1e-6):
                print(f"Rover with Swarm ID: {self.swarm_id}, Rover ID: {self.rover_id} died")
                rover_list.remove(self)
                print("---------------------------------------------------------")
                return
        for obstacle in obstacle_list:
            if(distance(obstacle.location, self.location) <= obstacle.radius):
                print(f"Rover with Swarm ID: {self.swarm_id}, Rover ID: {self.rover_id} is unsafe")
                print("---------------------------------------------------------")
                return

    def change_location(self, message_swarm_id, message_rover_id, move_amount):
        if self.swarm_id == message_swarm_id and self.rover_id == message_rover_id:
            print(f"Received message for Swarm ID: {message_swarm_id}, Rover ID: {message_rover_id}")
            dist = distance((0, 0), (move_amount[0], move_amount[1]))
            if((self.battery - dist*(0.1)) < 0):
                print("Insufficient juice to move the rover")
                return
            else:
                print(f"Moving by {move_amount} units.")
                self.location = (self.location[0] + move_amount[0], self.location[1]+move_amount[1])
                self.battery = self.battery - dist*(0.1)
                print(f"New Location: {self.location}")
                print(f"Battery left: {self.battery}%")
                print("---------------------------------------------------------")
                self.check_safety()
        else:
            print("You are making a mistake")
            print("---------------------------------------------------------")


def display_rovers():
    for rover in rover_list:
        rover.display_info()
    print("---------------------------------------------------------")

class DaughterRover(Rover):
    geometry = {
        'length': Rover.geometry['length'] / 2,
        'width': Rover.geometry['width'] / 2,
        'height': Rover.geometry['height'] / 2
    }

    def __init__(self, swarm_id, rover_id, location, battery):
        self.swarm_id = swarm_id
        self.rover_id = rover_id
        self.location = location
        self.battery = battery

    def change_location(self, message_swarm_id, message_rover_id, move_amount):
        if self.swarm_id == message_swarm_id and self.rover_id == message_rover_id:
            print(f"Received message for Swarm ID: {message_swarm_id}, Rover ID: {message_rover_id}")
            dist = distance((0, 0), (move_amount[0], move_amount[1]))
            if((self.battery - dist*(0.1)) < 0):
                print("Insufficient juice to move the rover")
                return
            else:
                print(f"Moving by {move_amount} units.")
                self.location = (self.location[0] + move_amount[0], self.location[1]+move_amount[1])
                self.battery = self.battery - dist*(0.1)
                print(f"New Location: {self.location}")
                print(f"Battery left: {self.battery}%")
                print("---------------------------------------------------------")
                self.check_safety()
        else:
            print("You are making a mistake")
            print("---------------------------------------------------------")


class User:
    def __init__(self, user_id):
        self.user_id = user_id
        self.controlled_rovers = set()

    def print_user_id(self):
        print(f"User ID: {self.user_id}")
        print("---------------------------------------------------------")

    def view_rover_location(self, rover):
        print(f"{self.user_id} is denied access to View Rover Location") 
        print("---------------------------------------------------------")

    def move_rover(self, rover, move_amount):
        print(f"{self.user_id} is denied access to Move Rover") 
        print("---------------------------------------------------------")

    def add_rover(self, rover):
        print(f"{self.user_id} is denied access to Add Rover") 
        print("---------------------------------------------------------")

    def remove_rover(self, rover):
        print(f"{self.user_id} is denied access to Remove Rover") 
        print("---------------------------------------------------------")

class Scientist(User):
    def view_rover_location(self, rover):
        print(f"{self.user_id} is viewing the location of Rover ID: {rover.rover_id}")
        print(f"Rover Location: {rover.location}") 
        print("---------------------------------------------------------")

class Operator(User):
    def move_rover(self, rover, move_amount):
        print(f"{self.user_id} is moving Rover ID: {rover.rover_id} by {move_amount} units.")
        rover.change_location(rover.swarm_id, rover.rover_id, move_amount) 
        # print("---------------------------------------------------------")

class Manager(User):
    def add_rover(self, rover):
        self.controlled_rovers.add((rover.swarm_id, rover.rover_id))
        print(f"{self.user_id} added Rover ID: {rover.rover_id} to controlled rovers.")
        print("---------------------------------------------------------")

    def remove_rover(self, rover):
        self.controlled_rovers.remove((rover.swarm_id, rover.rover_id))
        print(f"{self.user_id} removed Rover ID: {rover.rover_id} from controlled rovers.")
        print("---------------------------------------------------------")

rover1 = Rover("Swarm_1", "Rover_1", (5, 5))
rover2 = Rover("Swarm_2", "Rover_2", (0, 0))

obs1 = Obstacle((10, 10), 10)
obs2 = Obstacle((0, 0), 10)

display_obstacles()
display_rovers()

user_scientist = Scientist("Scientist_1")
user_operator = Operator("Operator_1")
user_manager = Manager("Manager_1")

user_scientist.view_rover_location(rover1)
user_scientist.move_rover(rover1, (10, 2))  
user_scientist.add_rover(rover1)     
user_scientist.remove_rover(rover1)  

user_operator.move_rover(rover1, (10, 0))  

user_manager.add_rover(rover2)     
# user_manager.remove_rover(rover2)  

print("Controlled Rovers by Manager1:", user_manager.controlled_rovers)

rover1.check_safety()
# rover1.change_location("Swarm1", "RoverA", (-5, -5))


