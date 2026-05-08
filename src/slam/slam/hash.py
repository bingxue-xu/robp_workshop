import hashlib


def generate_id(x, y, description):
    # Round coordinates to 1 decimal point to ensure a 0.1 m radius
    rounded_x, rounded_y = round_to_half_meter(x, y)

    # Combine x, y, and description
    data = f"{rounded_x},{rounded_y},{description}".encode()

    # Generate unique hash
    hash_object = hashlib.sha1(data)
    unique_id = hash_object.hexdigest()

    return unique_id


def round_to_half_meter(x, y):
    rounded_x = round(x * 2) / 2
    rounded_y = round(y * 2) / 2
    return rounded_x, rounded_y


# Example usage:
"""
x1, y1 = 3.2, 2.8
description1 = "red cube"
id1 = generate_id(x1, y1, description1)
print("ID for", description1, "at coordinates (", x1, ",", y1, "):", id1)

x2, y2 = 2.8, 3.1  # Coordinates within 0.1 m radius of x1, y1
description2 = "red cube"
id2 = generate_id(x2, y2, description2)
print("ID for", description2, "at coordinates (", x2, ",", y2, "):", id2)

x3, y3 = 2.7, 3.4  # Coordinates outside 0.1 m radius of x1, y1
description3 = "red cube"
id3 = generate_id(x3, y3, description3)
print("ID for", description3, "at coordinates (", x3, ",", y3, "):", id3)
"""


"""
# Example usage:
x = 3.8  # Example x coordinate
y = 4.2  # Example y coordinate
rounded_x, rounded_y = round_to_half_meter(x, y)
print("Rounded coordinates:", rounded_x, rounded_y)
"""