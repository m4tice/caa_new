# Determine whether the vehicle has entered a specified intersection or not using Carla built-in coordinates
def check_enter_carla_int(vehicle, carla_intersection):
    # Get the current location of the vehicle
    location = vehicle.get_location()
    loc_x, loc_y = location.x, location.y

    # Initialize variable that indicates whether the car is at the intersection or not
    ent_int = False

    # check if vehicle is entering an intersection
    for num, x, y, d in carla_intersection:
        if x - d <= loc_x <= x + d and y + d >= loc_y >= y - d:
            ent_int = True
            break

    return ent_int
