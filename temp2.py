#temp file (rn it checks if this value of x and y is already present in coordinates.json)
import json
import temp4

# Initial values of x and y
x = 14.5
y = 20

json_file_name = 'coordinates.json'

if not temp4.is_value_present(json_file_name, x, y):
    temp4.store_values(json_file_name, x, y)
else:
    print(f"Values x={x} and y={y} are already present in {json_file_name}")