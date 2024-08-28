#this stores value of x and y in json only if it hasnt been stored previously with a tolerance of 100%
import json
import os

# Define the JSON file name
json_file_name = 'coordinates.json'

def check(x,y,new_x,new_y):
    diff_x=(abs(x-new_x)/x)*100
    diff_y=(abs(y-new_y)/y)*100
    diff = (diff_x+diff_y)/2
    print('diff is ',diff)
    return diff<100


# Function to check if x and y are already present in the JSON file
def is_value_present(file_name, x_value, y_value):
    if os.path.exists(file_name):  # Check if file exists
        with open(file_name, 'r') as file:
            try:
                data = json.load(file)
                # Check if the current (x, y) pair is already in the list
                for pair in data:
                    return check(x_value,y_value,pair['x'],pair['y'])
            except json.JSONDecodeError:
                # If the file is empty or corrupted, treat as not present
                return False
    return False

# Function to store new (x, y) pair in JSON file
def store_values(file_name, x_value, y_value):
    data = []
    
    # Read existing data if file exists
    if os.path.exists(file_name):
        with open(file_name, 'r') as file:
            try:
                data = json.load(file)
            except json.JSONDecodeError:
                # If the file is empty or corrupted, start with an empty list
                data = []
    
    # Add the new (x, y) pair
    data.append({'x': x_value, 'y': y_value})
    
    # Write the updated list back to the file
    with open(file_name, 'w') as file:
        json.dump(data, file, indent=4)
    
    print(f"Values x={x_value} and y={y_value} added to {file_name}")

def main(x,y):
    # Check if the (x, y) pair is present, if not add them
    if not is_value_present(json_file_name, x, y):
        store_values(json_file_name, x, y)
    else:
        print(f"Values x={x} and y={y} are already present in {json_file_name}")