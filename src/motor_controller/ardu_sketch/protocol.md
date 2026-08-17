# Arduino communication protocol

## Send
### R [float newX] [float newY] [float newTheta]
Sets internal x, y, and theta values with new ones 

### S [float leftSpeed] [float rightSpeed]
Sets target leftSpeed and rightSpeed

## Receive
Script periodically sends position and speeds values in this format:
[float x] [float y] [float theta] [float v] [float omega]