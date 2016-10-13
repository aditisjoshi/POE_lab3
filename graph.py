import matplotlib.pyplot as plt


lines = [line.rstrip('\n') for line in open('data2.txt')]


lines = filter(lambda a: a != "STRAIGHT", lines)
lines = filter(lambda a: a != "LEFT", lines)
lines = filter(lambda a: a != "RIGHT", lines)

# for line in lines:
#     if line == "STRAIGHT":
#         lines.remove("STRAIGHT")
#     elif line == "LEFT":
#         lines.remove("LEFT")
#     elif line == "RIGHT":
#         lines.remove("RIGHT")
#     else:
#         pass

left_motor = []
right_motor = []
left_sensor = []
right_sensor = []
for i, line in enumerate(lines[0:200]):
    if ((i + 1) % 4) == 0:
        right_sensor.append(int(line[-3:]))
    elif ((i + 2) % 4) == 0:
        left_sensor.append(int(line[-3:]))
    elif ((i + 3) % 4) == 0:
        if "NEG" in line:
            right_motor.append(-1*(int(line[-2:])))
        else: 
            right_motor.append((int(line[-2:])))
    elif ((i + 4) % 4) == 0:
        if "NEG" in line:
            left_motor.append(-1*(int(line[-2:])))
        else: 
            left_motor.append((int(line[-2:])))
    else:
        pass



fig, ax1 = plt.subplots()
ax1.plot(left_motor, 'b.')
ax1.set_xlabel('time (s)')
# Make the y-axis label and tick labels match the line color.
ax1.set_ylabel('motor speed', color='b')
ax1.set_ylim([-50,50])
for tl in ax1.get_yticklabels():
    tl.set_color('b')

ax2 = ax1.twinx()
ax2.plot(left_sensor, 'r.')
ax2.plot([0, len(left_sensor)],[600,600],'r-',linewidth=2)
ax2.set_ylabel('sensor reading', color='r')
ax2.set_ylim([450,850])
for tl in ax2.get_yticklabels():
    tl.set_color('r')
plt.show()



