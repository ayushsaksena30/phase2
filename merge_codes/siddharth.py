import math

def calculate_distance (l,m):
    altitude = 17.5  # in meters
    camera_resolution = (640,640)  # width and height of the image in pixels
    fov_horizontal = 68  # field of view in degrees (horizontal)
    fov_vertical = 68  # field of view in degrees (vertical)
    l-=320
    m-=320

    # camera resolution (dekhte h kya use hoga)
    px, py = camera_resolution
    # radian me change karte h fov ko
    fov_horizontal_rad = math.radians(fov_horizontal)
    fov_vertical_rad = math.radians(fov_vertical)

    # ground ka actual width or length jo camera cover kar sakta h vo nikalte h

    ground_width= 2*altitude* math.tan(fov_horizontal_rad)
    ground_height= 2*altitude*math.tan(fov_vertical_rad)

    # ye upar wale me thoda doubt h ki radian/2 kare ki na kare ye ek baar test karke dekhna padega

    #convertion value for pixel to meter.
    alpha=ground_width/px #alpha x length nikalne k lia 
    beta=ground_height/py #beta y length nikalne k lia

    #formula
    temp=(alpha*(l-ground_width/2))**2 + (beta*(m-ground_height/2))**2
    
    # l and m coordinate hai hotspot ya target ke center ke vo tum paas kara dena
    # or haa ek baar bracket dekh lena temp wale me ki () lagta h ki nhi python me 
    distance =math.sqrt(temp)
    return distance

# # Example 
# altitude = 17.5  # in meters
# camera_resolution = (640,640)  # width and height of the image in pixels
# fov_horizontal = 68  # field of view in degrees (horizontal)
# fov_vertical = 68  # field of view in degrees (vertical)
x=20
y=0

print(calculate_distance(x,y))
#NOTE FOR AYUSH
# easiest tarika yhi mila mujhe real time angle nikalne me laude lag rhe the 
# to check- round bracket dekh lena mujhe python ka confusion h thoda (C++ me to laga dete h round bracket iska nhi yaad )
# temp ka formula yahi h vo shi hai 
# return distance dekh lena shi se chal rha ho 