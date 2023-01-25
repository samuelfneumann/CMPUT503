import os
message = "Hello from %s!" % os.environ['VEHICLE_NAME']
print(message)
